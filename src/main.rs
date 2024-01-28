#![no_std]
#![no_main]
use attiny_hal::simple_pwm::{Prescaler, Timer1Pwm};
use avr_hal_generic::simple_pwm::IntoPwmPin;
use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
use panic_halt as _;
use tinyrand::RandRange;

#[attiny_hal::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins = attiny_hal::pins!(dp);

    let mut rng = tinyrand::StdRand::default();

    let mut timer = Timer1Pwm::new(dp.TC1, Prescaler::Direct);
    let mut pwm = pins.pb4.into_output().into_pwm(&mut timer);

    let mut delay = attiny_hal::delay::Delay::<attiny_hal::clock::MHz1>::new();
    pwm.enable();
    loop {
        let duty: u16 = rng.next_range(0..24);
        if duty > 12 {
            pwm.set_duty_cycle_percent(0).unwrap();
        } else {
            pwm.set_duty_cycle_percent((78 - duty * 5) as u8).unwrap();
        }
        delay.delay_ms(72);
    }
}
