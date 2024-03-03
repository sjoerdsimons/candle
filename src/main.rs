#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell::RefCell;

use attiny_hal::port::{Pin, PB4};
use avr_device::interrupt::Mutex;
// old embedded hal
use avr_hal_generic::port::mode::{Floating, Input};
use embedded_hal::delay::DelayNs;
use infrared::protocol::Nec;
use infrared::Receiver;
use panic_halt as _;
use tinyrand::RandRange;

type IrPin = Pin<Input<Floating>, PB4>;
type IrProto = Nec;

mod clock;
use clock::MonotonicClock;

mod pwm;
use pwm::ComplementaryOCR1BPwm;

mod led;
use led::Led;

// Clock speed configured in fuses
type Speed = attiny_hal::clock::MHz1;

static mut IR: Option<Receiver<IrProto, IrPin>> = None;
static mut CLOCK: Option<clock::MonotonicClock<Speed>> = None;
static NEW_SETTINGS: Mutex<RefCell<Option<Settings>>> =
    Mutex::new(RefCell::new(Some(Settings::new())));

#[derive(Copy, Clone)]
struct Settings {
    state: State,
    brightness: u8,
    flicker: u8,
}

impl Settings {
    const MAX_BRIGHTNESS: u8 = 10;
    const MAX_FLICKER: u8 = 10;

    const fn new() -> Self {
        Self {
            state: State::On,
            brightness: Self::MAX_BRIGHTNESS,
            flicker: Self::MAX_FLICKER,
        }
    }

    fn on(&mut self) {
        self.state = State::On;
    }

    fn off(&mut self) {
        self.state = State::Off;
    }

    fn inc_flicker(&mut self) {
        self.flicker = (self.flicker + 1).min(Self::MAX_FLICKER);
    }

    fn dec_flicker(&mut self) {
        if self.flicker > 0 {
            self.flicker -= 1;
        }
    }

    fn inc_brightness(&mut self) {
        self.brightness = (self.brightness + 1).min(Self::MAX_BRIGHTNESS);
    }

    fn dec_brightness(&mut self) {
        if self.brightness > 0 {
            self.brightness -= 1;
        }
    }
}

#[derive(Copy, Clone)]
enum State {
    On,
    Off,
}

#[derive(Copy, Clone)]
enum Button {
    On,
    Off,
    Plus,
    Minus,
    Sl,
    Fl,
}

#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    // The interrupt macro magically turns this into a &mut Button for us in the function
    static mut LAST: (u32, Button) = (0, Button::Off);
    static mut CURRENT: Settings = Settings::new();
    let ir = unsafe { IR.as_mut().unwrap() };
    let clock = unsafe { CLOCK.as_ref().unwrap() };
    let now = clock.now();

    let cmd = match ir.event_instant(now) {
        Ok(Some(cmd)) => {
            if cmd.repeat {
                if now.saturating_sub(LAST.0) < (clock.freq() / 4) {
                    Some(LAST.1)
                } else {
                    None
                }
            } else {
                if cmd.addr != 0 {
                    return;
                }
                match cmd.cmd {
                    0x40 => Some(Button::On),
                    0x19 => Some(Button::Off),
                    0x44 => Some(Button::Plus),
                    0x16 => Some(Button::Minus),
                    0x0D => Some(Button::Fl),
                    0x43 => Some(Button::Sl),
                    _ => None,
                }
            }
        }
        _ => None,
    };

    if let Some(cmd) = cmd {
        *LAST = (now, cmd);
        match cmd {
            Button::On => CURRENT.on(),
            Button::Off => CURRENT.off(),
            Button::Plus => CURRENT.inc_brightness(),
            Button::Minus => CURRENT.dec_brightness(),
            Button::Fl => CURRENT.inc_flicker(),
            Button::Sl => CURRENT.dec_flicker(),
        }
        avr_device::interrupt::free(|cs| NEW_SETTINGS.borrow(cs).replace(Some(*CURRENT)));
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

    let ir = Receiver::with_pin(clock.freq(), pins.pb4);
    let pwm = ComplementaryOCR1BPwm::new(dp.TC1, pins.pb3.into_output());
    let mut led = Led::new(pwm);
    led.off();

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
    let mut settings = Settings::new();
    loop {
        let new_settings =
            avr_device::interrupt::free(|cs| NEW_SETTINGS.borrow(cs).borrow_mut().take());

        if let Some(new) = new_settings {
            settings = new;
            let duty: u16 =
                (u8::MAX as u16 * settings.brightness as u16) / Settings::MAX_BRIGHTNESS as u16;
            led.set_max_duty(duty as u8);

            // Short quick flicker when settings change
            for _ in 0..3 {
                led.off();
                delay.delay_ms(20);
                led.on();
                delay.delay_ms(20);
            }
        }

        match settings.state {
            State::On => {
                if settings.flicker == 0 {
                    led.on();
                    avr_device::asm::sleep();
                } else {
                    let duty: u16 = rng
                        .next_range(0..20 + 2 * (Settings::MAX_FLICKER - settings.flicker) as u16);
                    if duty > 10 {
                        led.on();
                    } else {
                        led.set_brightness_percent((20 + duty * 4) as u8);
                    }
                    delay.delay_ms(72);
                }
            }
            State::Off => {
                led.off();
                // Sleep until an interrupt arrives
                avr_device::asm::sleep();
            }
        }
    }
}
