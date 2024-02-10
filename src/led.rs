use embedded_hal::pwm::SetDutyCycle;

use crate::pwm::ComplementaryOCR1BPwm;

pub struct Led {
    pwm: ComplementaryOCR1BPwm,
}

impl Led {
    pub fn new(mut pwm: ComplementaryOCR1BPwm) -> Self {
        pwm.enable();
        Self { pwm }
    }

    pub fn on(&mut self) {
        let _ = self.pwm.set_duty_cycle_fully_on();
    }

    pub fn off(&mut self) {
        let _ = self.pwm.set_duty_cycle_fully_off();
    }

    pub fn brightness(&mut self, percent: u8) {
        let percent = percent.min(100);
        self.pwm.set_duty_cycle((55 + 2 * percent).into()).unwrap();
    }
}
