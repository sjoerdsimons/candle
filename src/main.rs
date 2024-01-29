#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::Cell;

use attiny_hal::port::{Pin, PB3, PB4};
use attiny_hal::simple_pwm::{Prescaler, Timer1Pwm};
use avr_device::attiny85::tc0::tccr0b::CS0_A;
use avr_device::interrupt::Mutex;
// old embedded hal
use avr_hal_generic::hal::digital::v2::InputPin;
use avr_hal_generic::port::mode::{Floating, Input, Output, PwmOutput};
use avr_hal_generic::simple_pwm::IntoPwmPin;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::ErrorType;
use embedded_hal::pwm::SetDutyCycle;
use infrared::protocol::nec::NecCommand;
use infrared::protocol::{Nec, NecDebug};
use infrared::Receiver;
use panic_halt as _;
use tinyrand::RandRange;

type IrPin = FlipPin<Pin<Input<Floating>, PB3>>;
type IrProto = NecDebug;

//static mut PWM: Option<Pin<PwmOutput<Timer1Pwm>, PB4>> = None;
static mut LED: Option<Pin<Output, PB4>> = None;
static mut IR: Option<Receiver<IrProto, IrPin>> = None;
static mut CLOCK: Option<Clock> = None;
static mut TIME: u32 = 0;
static LAST: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    //let pwm = unsafe { PWM.as_mut().unwrap() };
    let led = unsafe { LED.as_mut().unwrap() };
    let ir = unsafe { IR.as_mut().unwrap() };
    let clock = unsafe { CLOCK.as_ref().unwrap() };
    let now = clock.now();
    led.toggle();

    let pin = ir.pin_mut();

    if pin.is_high().unwrap() {
        //led.set_high();
        let prev = unsafe { TIME };
        if now > prev {
            let dt = now - prev;
            // Less then one second
            if dt < 1000 {
                /*
                avr_device::interrupt::free(|cs| {
                    let last = LAST.borrow(cs);
                    if last.get() == 0 {
                        last.set(dt);
                    }
                })
                */
            }
        }
        unsafe { TIME = now };
    } else {
        //led.set_low();
    }

    match ir.event_instant(now) {
        Ok(Some(cmd)) => avr_device::interrupt::free(|cs| {
            //led.toggle();
            if cmd.repeat {
                let l = LAST.borrow(cs);
                if l.get() == 0 {
                    LAST.borrow(cs).set(0x2);
                }
            } else {
                LAST.borrow(cs).set(cmd.bits & 0xff);
            }
        }),
        Ok(None) => (),
        _ => avr_device::interrupt::free(|cs| {
            //led.toggle();
            LAST.borrow(cs).set(0x1);
        }),
    }
}

#[attiny_hal::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins = attiny_hal::pins!(dp);

    let mut rng = tinyrand::StdRand::default();

    // Monotonic clock
    let clock = Clock::new(dp.TC0);
    clock.start();

    let mut timer = Timer1Pwm::new(dp.TC1, Prescaler::Direct);
    let ir = Receiver::with_pin(Clock::FREQ, FlipPin::new(pins.pb3));

    //let led = pins.pb4.into_output();
    let mut led0 = pins.pb0.into_output();
    let mut led1 = pins.pb1.into_output();
    let led4 = pins.pb4.into_output();

    //let mut pwm = pins.pb4.into_output().into_pwm(&mut timer);
    //pwm.enable();
    //pwm.set_duty_cycle_fully_off();

    // Safety: Only set before interrupts are enabled
    unsafe {
        CLOCK.replace(clock);
        LED.replace(led4);
        //PWM.replace(pwm);
        IR.replace(ir);
    }

    // Enable pin change interrupt on pb3
    dp.EXINT.pcmsk.write(|w| w.pcint3().set_bit());
    // Enable pin change interrupt
    dp.EXINT.gimsk.write(|w| w.pcie().set_bit());
    // Enable interrupts
    unsafe { avr_device::interrupt::enable() };

    //loop {
    //    avr_device::asm::sleep()
    //}

    let mut delay = attiny_hal::delay::Delay::<attiny_hal::clock::MHz1>::new();
    loop {
        let last = avr_device::interrupt::free(|cs| LAST.borrow(cs).get());
        if last == 0x0 {
            led0.set_high();
            led1.set_low();
            for _ in 0..5 {
                delay.delay_ms(200);
                led0.toggle();
                //led1.toggle();
            }
        } else if last == 0x1 || last == 0x2 {
            led0.set_high();
            led1.set_low();
            for i in 0..20 {
                delay.delay_ms(200);
                led0.toggle();
                if i % last == 0 {
                    led1.toggle();
                }
            }
        } else {
            led0.set_low();
            for i in 0..100 {
                led1.toggle();
                delay.delay_ms(last);
            }
        }
        delay.delay_ms(1000);
    }
}

struct FlipPin<P> {
    pin: P,
}

impl<P> FlipPin<P> {
    fn new(pin: P) -> Self {
        FlipPin { pin }
    }
}

impl<P> ErrorType for FlipPin<P>
where
    P: ErrorType,
{
    type Error = P::Error;
}

/*
impl<P> InputPin for FlipPin<P>
where
    P: InputPin + ErrorType,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }
}
*/

impl<P> avr_hal_generic::hal::digital::v2::InputPin for FlipPin<P>
where
    P: avr_hal_generic::hal::digital::v2::InputPin,
{
    type Error = P::Error;

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }
}

/// Based on uno-infra example from avr-hal
#[avr_device::interrupt(attiny85)]
fn TIMER0_COMPA() {
    unsafe { CLOCK.as_ref().unwrap().tick() };
}

struct Clock {
    tc0: Mutex<attiny_hal::pac::TC0>,
    cntr: Mutex<Cell<u32>>,
}

impl Clock {
    // Assuming CLK IO is at 1mhz
    const FREQ: u32 = 125_000;
    const PRESCALER: CS0_A = CS0_A::PRESCALE_8;
    const TOP: u8 = 255;

    pub const fn new(tc0: attiny_hal::pac::TC0) -> Clock {
        Clock {
            tc0: Mutex::new(tc0),
            cntr: Mutex::new(Cell::new(0)),
        }
    }

    pub fn start(&self) {
        avr_device::interrupt::free(|cs| {
            let tc0 = self.tc0.borrow(cs);
            // Configure the timer for the above interval (in CTC mode)
            // Clear Timer on Compare match; top in 0cr0a
            tc0.tccr0a.write(|w| w.wgm0().ctc());
            tc0.ocr0a.write(|w| w.bits(Self::TOP));
            tc0.tccr0b.write(|w| w.cs0().variant(Self::PRESCALER));

            // Enable interrupt
            tc0.timsk.write(|w| w.ocie0a().set_bit());
        })
    }

    pub fn now(&self) -> u32 {
        avr_device::interrupt::free(|cs| {
            let c = self.cntr.borrow(cs);
            let tc0 = self.tc0.borrow(cs);

            c.get() + (tc0.tcnt0.read().bits() as u32)
        })
    }

    pub fn tick(&self) {
        avr_device::interrupt::free(|cs| {
            let c = self.cntr.borrow(cs);

            let v = c.get();
            c.set(v.wrapping_add(258));
        });
    }
}
