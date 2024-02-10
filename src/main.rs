#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::Cell;

use attiny_hal::port::{Pin, PB4};
use attiny_hal::simple_pwm::{Prescaler, Timer1Pwm};
use avr_device::interrupt::Mutex;
// old embedded hal
use avr_hal_generic::port::mode::{Floating, Input};
use avr_hal_generic::simple_pwm::IntoPwmPin;
use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
use infrared::protocol::Nec;
use infrared::Receiver;
use panic_halt as _;
use tinyrand::RandRange;

type IrPin = Pin<Input<Floating>, PB4>;
type IrProto = Nec;

mod clock;
use clock::MonotonicClock;

#[derive(Copy, Clone)]
enum State {
    On,
    Off,
}

// Clock speed configured in fuses
type Speed = attiny_hal::clock::MHz1;

static mut IR: Option<Receiver<IrProto, IrPin>> = None;
static mut CLOCK: Option<clock::MonotonicClock<Speed>> = None;
static STATE: Mutex<Cell<State>> = Mutex::new(Cell::new(State::On));

#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    let ir = unsafe { IR.as_mut().unwrap() };
    let clock = unsafe { CLOCK.as_ref().unwrap() };
    let now = clock.now();

    let new_state = match ir.event_instant(now) {
        Ok(Some(cmd)) => {
            if cmd.repeat {
                None
            } else {
                match cmd.cmd {
                    // On
                    0x40 => Some(State::On),
                    // Off
                    0x19 => Some(State::Off),
                    _ => None,
                }
            }
        }
        _ => None,
    };

    if let Some(new_state) = new_state {
        avr_device::interrupt::free(|cs| {
            STATE.borrow(cs).set(new_state);
        });
    }
}

#[avr_device::interrupt(attiny85)]
fn TIMER0_OVF() {
    unsafe { CLOCK.as_ref().unwrap().overflow() };
}

#[attiny_hal::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins = attiny_hal::pins!(dp);

    let mut rng = tinyrand::StdRand::default();

    // Monotonic clock
    let clock = MonotonicClock::new(dp.TC0);
    clock.start();

    let timer = Timer1Pwm::new(dp.TC1, Prescaler::Direct);
    let ir = Receiver::with_pin(clock.freq(), pins.pb4);

    let mut pwm = pins.pb3.into_output().into_pwm(&timer);
    pwm.enable();
    let _ = pwm.set_duty_cycle_fully_off();

    // Safety: Only set before interrupts are enabled
    unsafe {
        CLOCK.replace(clock);
        IR.replace(ir);
    }

    // Enable pin change interrupt on pb4 for the IR receiver
    dp.EXINT.pcmsk.write(|w| w.pcint4().set_bit());
    // Enable pin change interrupt
    dp.EXINT.gimsk.write(|w| w.pcie().set_bit());
    // Enable interrupts
    unsafe { avr_device::interrupt::enable() };

    let mut delay = attiny_hal::delay::Delay::<attiny_hal::clock::MHz1>::new();
    loop {
        let state = avr_device::interrupt::free(|cs| STATE.borrow(cs).get());
        match state {
            State::On => {
                let duty: u16 = rng.next_range(0..24);
                if duty > 12 {
                    pwm.set_duty_cycle_fully_on().unwrap();
                } else {
                    pwm.set_duty_cycle_percent((22 + duty * 5) as u8).unwrap();
                }
                delay.delay_ms(72);
            }
            State::Off => {
                pwm.set_duty_cycle_fully_off().unwrap();
                // Sleep until an interrupt arrives
                avr_device::asm::sleep();
            }
        }
    }
}
