use embedded_hal::pwm::SetDutyCycle;

use crate::pwm::ComplementaryOCR1BPwm;

pub struct Led {
    pwm: ComplementaryOCR1BPwm,
    max: u8,
}

impl Led {
    // Minimal duty cycle with still a reasonable glow
    const MIN_DUTY: u16 = 20;
    pub fn new(mut pwm: ComplementaryOCR1BPwm) -> Self {
        pwm.enable();
        Self { pwm, max: u8::MAX }
    }

    pub fn on(&mut self) {
        let _ = self.pwm.set_duty_cycle(self.max as u16);
    }

    pub fn off(&mut self) {
        let _ = self.pwm.set_duty_cycle_fully_off();
    }

    pub fn set_max_duty(&mut self, max: u8) {
        self.max = max;
    }

    // Brightness within the max range
    pub fn set_brightness_percent(&mut self, percent: u8) {
        let percent = percent.min(100);
        let duty = Self::MIN_DUTY + (((self.max as u16 - Self::MIN_DUTY) * percent as u16) / 100);

        let _ = self.pwm.set_duty_cycle(duty);
    }
}
