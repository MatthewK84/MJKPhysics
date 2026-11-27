//! Simulation time and subsystem rates (ADR 0002).
//!
//! Time is an integer count of physics steps. Seconds are derived from it, never accumulated by
//! repeated addition, so the time after ten million steps is exactly ten million steps' worth.

use std::fmt;

use mjk_math::{StateHash, StateHasher};

/// The default physics rate.
pub const DEFAULT_PHYSICS_HZ: u32 = 1000;

/// Why a rate cannot be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RateError {
    /// A rate of zero.
    Zero,
    /// A subsystem rate above the physics rate.
    FasterThanPhysics {
        /// The subsystem rate.
        rate_hz: u32,
        /// The physics rate.
        physics_hz: u32,
    },
    /// A subsystem rate that does not divide the physics rate.
    NotADivisor {
        /// The subsystem rate.
        rate_hz: u32,
        /// The physics rate.
        physics_hz: u32,
    },
}

impl fmt::Display for RateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Zero => write!(f, "a rate must be at least 1 Hz"),
            Self::FasterThanPhysics {
                rate_hz,
                physics_hz,
            } => write!(
                f,
                "{rate_hz} Hz is faster than the {physics_hz} Hz physics rate"
            ),
            Self::NotADivisor {
                rate_hz,
                physics_hz,
            } => write!(
                f,
                "{rate_hz} Hz does not divide the {physics_hz} Hz physics rate; \
                 subsystems run every whole number of physics steps"
            ),
        }
    }
}

/// Simulation time: the number of physics steps taken, at a fixed rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clock {
    tick: u64,
    rate_hz: u32,
}

impl Clock {
    /// A clock at tick zero.
    ///
    /// # Errors
    ///
    /// [`RateError::Zero`] for a rate of zero.
    pub const fn new(rate_hz: u32) -> Result<Self, RateError> {
        Self::at(rate_hz, 0)
    }

    /// A clock at `tick`, for resuming a run.
    ///
    /// # Errors
    ///
    /// [`RateError::Zero`] for a rate of zero.
    pub const fn at(rate_hz: u32, tick: u64) -> Result<Self, RateError> {
        if rate_hz == 0 {
            return Err(RateError::Zero);
        }
        Ok(Self { tick, rate_hz })
    }

    /// Physics steps taken.
    #[must_use]
    pub const fn tick(&self) -> u64 {
        self.tick
    }

    /// The physics rate.
    #[must_use]
    pub const fn rate_hz(&self) -> u32 {
        self.rate_hz
    }

    /// The physics step.
    #[must_use]
    pub fn dt_s(&self) -> f64 {
        1.0 / f64::from(self.rate_hz)
    }

    /// The time: tick divided by rate, a single correctly rounded division.
    #[must_use]
    pub fn time_s(&self) -> f64 {
        exact_f64(self.tick) / f64::from(self.rate_hz)
    }

    /// Moves to the next tick.
    pub(crate) const fn advance(&mut self) {
        self.tick = self.tick.saturating_add(1);
    }
}

impl StateHash for Clock {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.tick.hash_state(hasher);
        self.rate_hz.hash_state(hasher);
    }
}

/// `n` as an `f64`, exactly for every `n` below 2⁵³ (285 000 years of steps at 1 kHz).
fn exact_f64(n: u64) -> f64 {
    let high = u32::try_from(n >> 32).unwrap_or(u32::MAX);
    let low = u32::try_from(n & 0xFFFF_FFFF).unwrap_or(u32::MAX);
    // Both terms and their sum are exact below 2⁵³.
    f64::from(high) * 4_294_967_296.0 + f64::from(low)
}

/// How often a subsystem runs: every `divisor` physics steps, starting at tick zero.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rate {
    divisor: u32,
}

impl Rate {
    /// The rate `rate_hz` within physics running at `physics_hz`.
    ///
    /// # Errors
    ///
    /// A [`RateError`] when either rate is zero, the subsystem is faster than the physics, or
    /// its rate does not divide the physics rate exactly. Rates are never rounded.
    pub const fn new(physics_hz: u32, rate_hz: u32) -> Result<Self, RateError> {
        if physics_hz == 0 || rate_hz == 0 {
            return Err(RateError::Zero);
        }
        if rate_hz > physics_hz {
            return Err(RateError::FasterThanPhysics {
                rate_hz,
                physics_hz,
            });
        }
        if !physics_hz.is_multiple_of(rate_hz) {
            return Err(RateError::NotADivisor {
                rate_hz,
                physics_hz,
            });
        }
        Ok(Self {
            divisor: physics_hz / rate_hz,
        })
    }

    /// Physics steps between runs.
    #[must_use]
    pub const fn divisor(&self) -> u32 {
        self.divisor
    }

    /// True on the ticks this subsystem runs: zero, then every `divisor` ticks.
    #[must_use]
    pub fn is_due(&self, tick: u64) -> bool {
        tick.is_multiple_of(u64::from(self.divisor))
    }
}
