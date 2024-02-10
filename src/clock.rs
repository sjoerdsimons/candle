use avr_device::attiny85::tc0::tccr0b::CS0_A;
use avr_device::interrupt::Mutex;
use avr_hal_generic::clock::Clock;
use core::cell::Cell;
use core::marker::PhantomData;

/// Monotonic wrapping counting clock at the given rate
/// of `MonotonicClock::FREQ`
///
/// Users of this clocks are expected to call the `overflow` function
/// when the TIMER0_OVF interrupt is triggered.
pub struct MonotonicClock<SPEED> {
    tc0: Mutex<attiny_hal::pac::TC0>,
    cntr: Mutex<Cell<u32>>,
    _speed: PhantomData<SPEED>,
}

impl<SPEED: Clock> MonotonicClock<SPEED> {
    /// Scale by 8 by default; this gives a 8 us precision at a 1Mhz clock
    pub const FREQ: u32 = SPEED::FREQ / 8;
    const PRESCALER: CS0_A = CS0_A::PRESCALE_8;

    pub const fn new(tc0: attiny_hal::pac::TC0) -> Self {
        Self {
            tc0: Mutex::new(tc0),
            cntr: Mutex::new(Cell::new(0)),
            _speed: PhantomData,
        }
    }

    pub const fn freq(&self) -> u32 {
        Self::FREQ
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

    pub fn overflow(&self) {
        avr_device::interrupt::free(|cs| {
            let c = self.cntr.borrow(cs);
            let v = c.get();
            c.set(v.wrapping_add(256));
        });
    }
}
