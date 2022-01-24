use super::{General, Timer};
use crate::afio::MAPR;
use crate::bb;
use crate::rcc::Clocks;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use fugit::TimerDurationU32;

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
use crate::pac::TIM1;
#[cfg(feature = "medium")]
use crate::pac::TIM4;
use crate::pac::{TIM2, TIM3};

pub use crate::pwm::{Channel, Pins, PwmChannel, C1, C2, C3, C4};
use crate::timer::sealed::Remap;

pub trait PwmExt<P, PINS>
where
    Self: Sized,
{
    fn pwm<REMAP, const FREQ: u32>(
        self,
        clocks: &Clocks,
        pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<FREQ>,
    ) -> Pwm<Self, REMAP, P, PINS, FREQ>
    where
        REMAP: Remap<Periph = Self>,
        PINS: Pins<REMAP, P>;

    fn pwm_us<REMAP>(
        self,
        clocks: &Clocks,
        pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<1_000_000>,
    ) -> Pwm<Self, REMAP, P, PINS, 1_000_000>
    where
        REMAP: Remap<Periph = Self>,
        PINS: Pins<REMAP, P>,
    {
        self.pwm::<REMAP, 1_000_000>(clocks, pins, mapr, time)
    }
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
impl<const FREQ: u32> Timer<TIM1, FREQ> {
    pub fn pwm<REMAP, P, PINS>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<FREQ>,
    ) -> Pwm<TIM1, REMAP, P, PINS, FREQ>
    where
        REMAP: Remap<Periph = TIM1>,
        PINS: Pins<REMAP, P>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim1_remap().bits(REMAP::REMAP) });

        // TIM1 has a break function that deactivates the outputs, this bit automatically activates
        // the output when no break input is present
        self.tim.bdtr.modify(|_, w| w.aoe().set_bit());

        tim1(self, _pins, time)
    }
}

macro_rules! pwm_ext {
    ($TIMX:ty) => {
        impl<P, PINS> PwmExt<P, PINS> for $TIMX
        where
            Self: Sized,
        {
            fn pwm<REMAP, const FREQ: u32>(
                self,
                clocks: &Clocks,
                pins: PINS,
                mapr: &mut MAPR,
                time: TimerDurationU32<FREQ>,
            ) -> Pwm<Self, REMAP, P, PINS, FREQ>
            where
                REMAP: Remap<Periph = Self>,
                PINS: Pins<REMAP, P>,
            {
                Timer::<_, FREQ>::new(self, clocks).pwm(pins, mapr, time)
            }
        }
    };
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity"))]
pwm_ext!(crate::pac::TIM1);

impl<const FREQ: u32> Timer<TIM2, FREQ> {
    pub fn pwm<REMAP, P, PINS>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<FREQ>,
    ) -> Pwm<TIM2, REMAP, P, PINS, FREQ>
    where
        REMAP: Remap<Periph = TIM2>,
        PINS: Pins<REMAP, P>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim2_remap().bits(REMAP::REMAP) });

        tim2(self, _pins, time)
    }
}

pwm_ext!(crate::pac::TIM2);

impl<const FREQ: u32> Timer<TIM3, FREQ> {
    pub fn pwm<REMAP, P, PINS>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<FREQ>,
    ) -> Pwm<TIM3, REMAP, P, PINS, FREQ>
    where
        REMAP: Remap<Periph = TIM3>,
        PINS: Pins<REMAP, P>,
    {
        mapr.modify_mapr(|_, w| unsafe { w.tim3_remap().bits(REMAP::REMAP) });

        tim3(self, _pins, time)
    }
}

pwm_ext!(crate::pac::TIM3);

#[cfg(feature = "medium")]
impl<const FREQ: u32> Timer<TIM4, FREQ> {
    pub fn pwm<REMAP, P, PINS>(
        self,
        _pins: PINS,
        mapr: &mut MAPR,
        time: TimerDurationU32<FREQ>,
    ) -> Pwm<TIM4, REMAP, P, PINS>
    where
        REMAP: Remap<Periph = TIM4>,
        PINS: Pins<REMAP, P>,
    {
        mapr.modify_mapr(|_, w| w.tim4_remap().bit(REMAP::REMAP == 1));

        tim4(self, _pins, time)
    }
}

#[cfg(feature = "medium")]
pwm_ext!(crate::pac::TIM4);

pub struct Pwm<TIM, REMAP, P, PINS, const FREQ: u32>
where
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    timer: Timer<TIM, FREQ>,
    _pins: PhantomData<(REMAP, P, PINS)>,
}

impl<TIM, REMAP, P, PINS, const FREQ: u32> Deref for Pwm<TIM, REMAP, P, PINS, FREQ>
where
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    type Target = Timer<TIM, FREQ>;
    fn deref(&self) -> &Self::Target {
        &self.timer
    }
}

impl<TIM, REMAP, P, PINS, const FREQ: u32> DerefMut for Pwm<TIM, REMAP, P, PINS, FREQ>
where
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timer
    }
}

impl<TIM, REMAP, P, PINS, const FREQ: u32> Pwm<TIM, REMAP, P, PINS, FREQ>
where
    REMAP: Remap<Periph = TIM>,
    PINS: Pins<REMAP, P>,
{
    pub fn split(self) -> PINS::Channels {
        PINS::split()
    }
}

macro_rules! hal {
    ($($TIMX:ty: ($timX:ident),)+) => {
        $(
            fn $timX<REMAP, P, PINS, const FREQ: u32>(
                mut timer: Timer<$TIMX, FREQ>,
                _pins: PINS,
                time: TimerDurationU32<FREQ>,
            ) -> Pwm<$TIMX, REMAP, P, PINS, FREQ>
            where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP, P>,
            {
                if PINS::C1 {
                    timer.tim.ccmr1_output()
                        .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1() );
                }

                if PINS::C2 {
                    timer.tim.ccmr1_output()
                        .modify(|_, w| w.oc2pe().set_bit().oc2m().pwm_mode1() );
                }

                if PINS::C3 {
                    timer.tim.ccmr2_output()
                        .modify(|_, w| w.oc3pe().set_bit().oc3m().pwm_mode1() );
                }

                if PINS::C4 {
                    timer.tim.ccmr2_output()
                        .modify(|_, w| w.oc4pe().set_bit().oc4m().pwm_mode1() );
                }
                timer.tim.set_auto_reload(time.ticks() - 1).unwrap();

                // The psc register is buffered, so we trigger an update event to update it
                // Sets the URS bit to prevent an interrupt from being triggered by the UG bit
                timer.tim.cr1.modify(|_, w| w.urs().set_bit());
                timer.tim.egr.write(|w| w.ug().set_bit());
                timer.tim.cr1.modify(|_, w| w.urs().clear_bit());

                timer.tim.cr1.write(|w|
                    w.cms()
                        .bits(0b00)
                        .dir()
                        .clear_bit()
                        .opm()
                        .clear_bit()
                        .cen()
                        .set_bit()
                );

                Pwm {
                    timer,
                    _pins: PhantomData
                }
            }

            /*
            The following implemention of the embedded_hal::Pwm uses Hertz as a time type.  This was choosen
            because of the timescales of operations being on the order of nanoseconds and not being able to
            efficently represent a float on the hardware.  It might be possible to change the time type to
            a different time based using such as the nanosecond.  The issue with doing so is that the max
            delay would then be at just a little over 2 seconds because of the 32 bit depth of the number.
            Using milliseconds is also an option, however, using this as a base unit means that only there
            could be resolution issues when trying to get a specific value, because of the integer nature.

            To find a middle ground, the Hertz type is used as a base here and the Into trait has been
            defined for several base time units.  This will allow for calling the set_period method with
            something that is natural to both the MCU and the end user.
            */
            impl<REMAP, P, PINS, const FREQ: u32> embedded_hal::Pwm for Pwm<$TIMX, REMAP, P, PINS, FREQ> where
                REMAP: Remap<Periph = $TIMX>,
                PINS: Pins<REMAP, P>,
            {
                type Channel = Channel;
                type Duty = u16;
                type Time = TimerDurationU32<FREQ>;

                fn enable(&mut self, channel: Self::Channel) {
                    match PINS::check_used(channel) {
                        Channel::C1 => unsafe { bb::set(&self.tim.ccer, 0) },
                        Channel::C2 => unsafe { bb::set(&self.tim.ccer, 4) },
                        Channel::C3 => unsafe { bb::set(&self.tim.ccer, 8) },
                        Channel::C4 => unsafe { bb::set(&self.tim.ccer, 12) }
                    }
                }

                fn disable(&mut self, channel: Self::Channel) {
                    match PINS::check_used(channel) {
                        Channel::C1 => unsafe { bb::clear(&self.tim.ccer, 0) },
                        Channel::C2 => unsafe { bb::clear(&self.tim.ccer, 4) },
                        Channel::C3 => unsafe { bb::clear(&self.tim.ccer, 8) },
                        Channel::C4 => unsafe { bb::clear(&self.tim.ccer, 12) },
                    }
                }

                fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
                    match PINS::check_used(channel) {
                        Channel::C1 => self.tim.ccr1.read().ccr().bits(),
                        Channel::C2 => self.tim.ccr2.read().ccr().bits(),
                        Channel::C3 => self.tim.ccr3.read().ccr().bits(),
                        Channel::C4 => self.tim.ccr4.read().ccr().bits(),
                    }
                }

                fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
                    match PINS::check_used(channel) {
                        Channel::C1 => self.tim.ccr1.write(|w| w.ccr().bits(duty)),
                        Channel::C2 => self.tim.ccr2.write(|w| w.ccr().bits(duty)),
                        Channel::C3 => self.tim.ccr3.write(|w| w.ccr().bits(duty)),
                        Channel::C4 => self.tim.ccr4.write(|w| w.ccr().bits(duty)),
                    }
                }

                /// If `0` returned means max_duty is 2^16
                fn get_max_duty(&self) -> Self::Duty {
                    self.tim.arr.read().arr().bits().wrapping_add(1)
                }

                fn get_period(&self) -> Self::Time {
                    TimerDurationU32::from_ticks((self.tim.arr.read().arr().bits() as u32) + 1)
                }

                fn set_period<T>(&mut self, period: T) where T: Into<Self::Time> {
                    self.tim.set_auto_reload(period.into().ticks() - 1).unwrap();
                }
            }
        )+
    }
}

#[cfg(any(feature = "stm32f100", feature = "stm32f103", feature = "connectivity",))]
hal! {
    crate::pac::TIM1: (tim1),
}

hal! {
    crate::pac::TIM2: (tim2),
    crate::pac::TIM3: (tim3),
}

#[cfg(feature = "medium")]
hal! {
    crate::pac::TIM4: (tim4),
}
