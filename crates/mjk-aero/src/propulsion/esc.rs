//! The electronic speed controller ([#19](https://github.com/MatthewK84/MJKPhysics/issues/19)),
//! Eq. (MOT-9): a command from 0 to 1 becomes a duty cycle after a latency and through a slew
//! limit, once per physics step, and the duty cycle times the supply voltage is the average
//! voltage at the motor.
//!
//! The ESC is discrete, as real ESCs are: it runs before each step and its duty cycle is held for
//! the step, so the motor's equations stay smooth inside the integrator's stages.

use std::fmt;

use super::motor::Drive;

/// An ESC's timing and drive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Esc {
    latency_ticks: u32,
    slew_per_tick: Option<f64>,
    drive: Drive,
}

/// Why an ESC's parameters are not usable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EscError {
    /// The latency is negative, not finite, or more ticks than a `u32` holds.
    Latency {
        /// The latency, s.
        latency_s: f64,
    },
    /// The slew rate is not positive and finite.
    SlewRate {
        /// The slew rate, full scale per second.
        slew_rate_per_s: f64,
    },
    /// The current limit is not positive and finite.
    CurrentLimit {
        /// The limit, A.
        current_limit_a: f64,
    },
    /// The physics rate is zero.
    Rate,
}

impl fmt::Display for EscError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Latency { latency_s } => write!(
                f,
                "a latency must be finite, not negative, and a whole number of ticks that fits \
                 in 32 bits; found {latency_s} s"
            ),
            Self::SlewRate { slew_rate_per_s } => write!(
                f,
                "a slew rate must be positive and finite; found {slew_rate_per_s} per second"
            ),
            Self::CurrentLimit { current_limit_a } => write!(
                f,
                "a current limit must be positive and finite; found {current_limit_a} A"
            ),
            Self::Rate => f.write_str("the physics rate must be at least 1 Hz"),
        }
    }
}

/// `value` rounded to the nearest whole number, halves away from zero, if that fits in a `u32`.
///
/// The conversion decodes the rounded double's bits, so it needs no `as` cast: an integer
/// `n ≥ 1` below 2³² is `1.m × 2^e` with `e ≤ 31`, and `n` is the significand shifted right by
/// `52 − e`.
fn nearest_whole(value: f64) -> Option<u32> {
    let rounded = libm::round(value);
    if !(0.0..=f64::from(u32::MAX)).contains(&rounded) {
        return None;
    }
    if rounded < 1.0 {
        return Some(0);
    }
    let bits = rounded.to_bits();
    let exponent = ((bits >> 52) & 0x7ff).checked_sub(1023)?;
    let significand = (bits & 0x000f_ffff_ffff_ffff) + 0x0010_0000_0000_0000;
    let whole = significand >> 52_u64.checked_sub(exponent)?;
    u32::try_from(whole).ok()
}

impl Esc {
    /// An ESC at physics rate `physics_hz`.
    ///
    /// `latency_s` is rounded to the nearest whole number of ticks, halves up; `slew_rate_per_s`
    /// is the fastest change of duty cycle, in full scale per second, and none means
    /// instantaneous.
    ///
    /// # Errors
    ///
    /// An [`EscError`] for a latency, slew rate or current limit that is not usable, or a zero
    /// physics rate.
    pub fn new(
        latency_s: f64,
        slew_rate_per_s: Option<f64>,
        drive: Drive,
        physics_hz: u32,
    ) -> Result<Self, EscError> {
        if physics_hz == 0 {
            return Err(EscError::Rate);
        }
        let ticks = (latency_s >= 0.0)
            .then(|| nearest_whole(latency_s * f64::from(physics_hz)))
            .flatten()
            .ok_or(EscError::Latency { latency_s })?;
        if let Some(rate) = slew_rate_per_s.filter(|rate| !(rate.is_finite() && *rate > 0.0)) {
            return Err(EscError::SlewRate {
                slew_rate_per_s: rate,
            });
        }
        let bad_limit = drive
            .current_limit_a
            .filter(|limit| !(limit.is_finite() && *limit > 0.0));
        if let Some(limit) = bad_limit {
            return Err(EscError::CurrentLimit {
                current_limit_a: limit,
            });
        }
        Ok(Self {
            latency_ticks: ticks,
            slew_per_tick: slew_rate_per_s.map(|rate| rate / f64::from(physics_hz)),
            drive,
        })
    }

    /// The latency, in whole ticks.
    #[must_use]
    pub const fn latency_ticks(&self) -> u32 {
        self.latency_ticks
    }

    /// The largest change of duty cycle in one tick, or `None` for no slew limit.
    #[must_use]
    pub const fn slew_per_tick(&self) -> Option<f64> {
        self.slew_per_tick
    }

    /// Braking and current limit.
    #[must_use]
    pub const fn drive(&self) -> Drive {
        self.drive
    }

    /// The ESC at rest at duty cycle `duty`, with every delayed command equal to it. Allocates
    /// the delay line once; updating it never allocates.
    #[must_use]
    pub fn start(&self, duty: f64) -> EscState {
        let duty = saturated(duty);
        let length = usize::try_from(self.latency_ticks).unwrap_or(usize::MAX);
        EscState {
            delay: vec![duty; length],
            next: 0,
            duty,
            slew_per_tick: self.slew_per_tick,
        }
    }
}

/// A command held to the range 0 to 1; a command that is not a number, or is −0, reads as 0.
fn saturated(command: f64) -> f64 {
    if command > 0.0 { command.min(1.0) } else { 0.0 }
}

/// An ESC's commands in flight and its duty cycle.
#[derive(Clone, Debug, PartialEq)]
pub struct EscState {
    delay: Vec<f64>,
    next: usize,
    duty: f64,
    slew_per_tick: Option<f64>,
}

impl EscState {
    /// Takes this tick's `command` and returns the duty cycle for the step, Eq. (MOT-9): the
    /// command from `latency` ticks ago, saturated to 0–1, approached by at most the slew limit.
    pub fn update(&mut self, command: f64) -> f64 {
        let target = self.delayed(saturated(command));
        let step = self.slew_per_tick.unwrap_or(f64::INFINITY);
        // The target, held within one step of the duty cycle: within reach, exactly the target.
        self.duty = target.min(self.duty + step).max(self.duty - step);
        self.duty
    }

    /// The duty cycle now.
    #[must_use]
    pub const fn duty(&self) -> f64 {
        self.duty
    }

    /// Puts `command` into the delay line and returns the one leaving it.
    fn delayed(&mut self, command: f64) -> f64 {
        let length = self.delay.len();
        let Some(slot) = self.delay.get_mut(self.next) else {
            return command;
        };
        let leaving = *slot;
        *slot = command;
        self.next = (self.next + 1) % length;
        leaving
    }
}
