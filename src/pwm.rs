use attiny_hal::port::{Pin, PB3};
use avr_device::attiny85::TC1;
use avr_hal_generic::port::mode::Output;
use embedded_hal::pwm::{ErrorKind, SetDutyCycle};

/// Complementary OCR1B pwm implementation to use on PB3; as side-effect that means PB4 will also
/// become a PWM output *if* set to output mode...
pub struct ComplementaryOCR1BPwm {
    timer: TC1,
    _pin: Pin<Output, PB3>,
}

impl ComplementaryOCR1BPwm {
    pub fn new(timer: TC1, _pin: Pin<Output, PB3>) -> Self {
        // Turn on pwm mode for ocr1b with no clock prescaler
        timer.gtccr.modify(|_, w| w.pwm1b().set_bit());
        timer.tccr1.modify(|_, w| w.cs1().direct());

        Self { timer, _pin }
    }

    pub fn enable(&mut self) {
        self.timer.gtccr.modify(|_, w| w.com1b().bits(0b01));
    }

    #[allow(dead_code)]
    pub fn disable(&mut self) {
        self.timer.gtccr.modify(|_, w| w.com1b().disconnected());
    }
}

impl SetDutyCycle for ComplementaryOCR1BPwm {
    fn max_duty_cycle(&self) -> u16 {
        u8::MAX as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        if duty > u8::MAX as u16 {
            Err(PwmError::DutyCycleTooLarge)
        } else {
            self.timer.ocr1b.write(|w| w.bits(duty as u8));
            Ok(())
        }
    }
}

#[derive(Debug)]
pub enum PwmError {
    DutyCycleTooLarge,
}

impl embedded_hal::pwm::Error for PwmError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl embedded_hal::pwm::ErrorType for ComplementaryOCR1BPwm {
    type Error = PwmError;
}
