#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::Cell;

use attiny_hal::port::{Pin, PB3, PB4};
use attiny_hal::simple_pwm::{Prescaler, Timer1Pwm};
use avr_device::attiny85::tc0::tccr0b::CS0_A;
use avr_device::interrupt::Mutex;
use avr_hal_generic::hal::blocking::delay::DelayUs;
use avr_hal_generic::hal::digital::v2::{InputPin, ToggleableOutputPin};
// old embedded hal
use avr_hal_generic::port::mode::{Floating, Input, Output, PwmOutput};
use avr_hal_generic::simple_pwm::IntoPwmPin;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{ErrorType, StatefulOutputPin};
use embedded_hal::pwm::SetDutyCycle;
use infrared::protocol::nec::NecCommand;
use infrared::protocol::Nec;
use infrared::Receiver;
use panic_halt as _;
use tinyrand::RandRange;

type IrPin = FlipPin<Pin<Input<Floating>, PB3>>;
type IrProto = Nec;

//static mut PWM: Option<Pin<PwmOutput<Timer1Pwm>, PB4>> = None;
static mut LED: Option<Pin<Output, PB4>> = None;
static mut IR: Option<Receiver<IrProto, IrPin>> = None;
static mut CLOCK: Option<Clock> = None;

#[derive(Default, Clone, Copy)]
struct Data {
    offset: u8,
    cmd: u8,
    stamps: [u32; 32],
}

static mut DATA: Mutex<Data> = Mutex::new(Data {
    offset: 0,
    cmd: 0,
    stamps: [0; 32],
});

#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    //let pwm = unsafe { PWM.as_mut().unwrap() };
    let led = unsafe { LED.as_mut().unwrap() };
    let ir = unsafe { IR.as_mut().unwrap() };
    let clock = unsafe { CLOCK.as_ref().unwrap() };
    let now = clock.now();

    let pin = ir.pin_mut();
    let stamp = if pin.is_high().unwrap() {
        now | 0x8000_0000
    } else {
        now
    };

    let cmd = match ir.event_instant(now) {
        Ok(Some(cmd)) => {
            if cmd.repeat {
                0x2 // Repeat
            } else {
                led.toggle();
                cmd.cmd
            }
        }
        Ok(None) => 0x0,
        Err(_) => 0x1, // error
    };

    avr_device::interrupt::free(|cs| {
        let data = unsafe { DATA.get_mut() };
        if data.cmd == 0 {
            data.cmd = cmd;
            if data.offset < data.stamps.len() as u8 {
                data.stamps[data.offset as usize] = stamp;
                data.offset += 1;
            }
        }
    });
}

fn send_u8<L, D>(led: &mut L, mut delay: &mut D, byte: u8)
where
    L: StatefulOutputPin,
    D: DelayUs<u16>,
{
    for i in 0..8 {
        let bit = (byte >> (i)) & 0x1;
        led.toggle();
        // high
        delay.delay_us(530);
        // low
        led.toggle();
        if bit == 0 {
            delay.delay_us(530)
        } else {
            delay.delay_us(1600)
        }
    }
}

fn send<L, D>(mut led: L, delay: &mut D, address: u8, command: u8)
where
    L: StatefulOutputPin,
    D: DelayUs<u16>,
{
    // header high
    led.toggle();
    delay.delay_us(9000);

    // header low
    led.toggle();
    delay.delay_us(4500);

    send_u8(&mut led, delay, address);
    send_u8(&mut led, delay, !address);
    send_u8(&mut led, delay, command);
    send_u8(&mut led, delay, !command);
    // final burst
    led.toggle();
    delay.delay_us(560);
    led.toggle();
    delay.delay_us(1000);
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
    led0.toggle();

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
        led1.toggle();
        let data = avr_device::interrupt::free(|cs| *unsafe { DATA.borrow(cs) });
        led1.toggle();
        // START sequence
        send(&mut led0, &mut delay, 0xff, 0xee);
        send(&mut led0, &mut delay, 0xee, 0xff);
        // Send command byte if any
        send(&mut led0, &mut delay, 0xcc, data.cmd);
        send(&mut led0, &mut delay, 0xcc, data.cmd);
        for i in 0..data.offset {
            let byte = data.stamps[i as usize].to_be_bytes();
            send(&mut led0, &mut delay, byte[0], byte[1]);
            send(&mut led0, &mut delay, byte[2], byte[3]);
        }
        // le end
        send(&mut led0, &mut delay, 0xee, 0xff);
        send(&mut led0, &mut delay, 0xff, 0xee);
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

#[avr_device::interrupt(attiny85)]
fn TIMER0_OVF() {
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

    pub const fn new(tc0: attiny_hal::pac::TC0) -> Clock {
        Clock {
            tc0: Mutex::new(tc0),
            cntr: Mutex::new(Cell::new(0)),
        }
    }

    pub fn start(&self) {
        avr_device::interrupt::free(|cs| {
            let tc0 = self.tc0.borrow(cs);
            tc0.tccr0b.write(|w| w.cs0().variant(Self::PRESCALER));

            // Enable interrupt on overflow
            tc0.timsk.write(|w| w.toie0().set_bit());
        })
    }

    pub fn now(&self) -> u32 {
        avr_device::interrupt::free(|cs| {
            let tc0 = self.tc0.borrow(cs);
            let cnt = tc0.tcnt0.read().bits() as u32;
            let overflow = tc0.tifr.read().tov0().bit_is_set();

            let ticks = self.cntr.borrow(cs).get();
            // If there was an overflow but the counter is
            // already close to the limit assume the overflow
            // happened just after reading the cnt bits
            if overflow && cnt < 200 {
                ticks.wrapping_add(256 + cnt)
            } else {
                ticks.wrapping_add(cnt)
            }
        })
    }

    pub fn tick(&self) {
        avr_device::interrupt::free(|cs| {
            let c = self.cntr.borrow(cs);
            let v = c.get();
            c.set(v.wrapping_add(256));
        });
    }
}
