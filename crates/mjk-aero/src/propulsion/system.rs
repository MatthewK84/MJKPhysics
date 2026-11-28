//! A motor turning a load, as a system the integrators can step
//! ([#19](https://github.com/MatthewK84/MJKPhysics/issues/19)).
//!
//! The speed is an ordinary number, so the chart that RKMK4 steps in is plain addition and every
//! integrator treats the motor alike. A vehicle joins its motors' speeds to the rigid body once
//! rotor forces exist ([#21](https://github.com/MatthewK84/MJKPhysics/issues/21)).

use std::ops::{Add, Mul};

use mjk_dynamics::{Coordinates, LieSystem, System};
use mjk_math::{StateHash, StateHasher};

use super::motor::{DriveInput, MotorModel};

/// The torque a load needs to turn at a speed.
pub trait LoadTorque {
    /// The torque, N·m, at `speed_rad_s`.
    fn torque_n_m(&self, speed_rad_s: f64) -> f64;

    /// How fast that torque grows with speed, N·m per rad/s, for the small-signal time constant.
    fn slope(&self, speed_rad_s: f64) -> f64;
}

/// A load that needs the same torque at every speed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConstantLoad {
    /// The torque, N·m.
    pub torque_n_m: f64,
}

impl LoadTorque for ConstantLoad {
    fn torque_n_m(&self, _: f64) -> f64 {
        self.torque_n_m
    }

    fn slope(&self, _: f64) -> f64 {
        0.0
    }
}

/// A propeller's drag torque, `coefficient · ω²`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PropellerLoad {
    /// The coefficient, N·m·s².
    pub coefficient_n_m_s2: f64,
}

impl LoadTorque for PropellerLoad {
    fn torque_n_m(&self, speed_rad_s: f64) -> f64 {
        self.coefficient_n_m_s2 * speed_rad_s * speed_rad_s
    }

    fn slope(&self, speed_rad_s: f64) -> f64 {
        2.0 * self.coefficient_n_m_s2 * speed_rad_s
    }
}

/// A rotor's speed, rad/s; the same type holds its rate of change.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorSpeed {
    /// The speed, rad/s.
    pub speed_rad_s: f64,
}

impl Add for MotorSpeed {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            speed_rad_s: self.speed_rad_s + rhs.speed_rad_s,
        }
    }
}

impl Mul<f64> for MotorSpeed {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            speed_rad_s: self.speed_rad_s * rhs,
        }
    }
}

impl Coordinates for MotorSpeed {
    fn positions(self) -> Self {
        Self::default()
    }

    fn velocities(self) -> Self {
        self
    }

    /// A rotor turns one way: a step that overshoots zero leaves it at rest, at +0. A NaN
    /// passes through, to be seen.
    fn project(self) -> Self {
        if self.speed_rad_s <= 0.0 {
            return Self::default();
        }
        self
    }
}

impl StateHash for MotorSpeed {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.speed_rad_s.hash_state(hasher);
    }
}

/// A motor turning a load, with the ESC's output held for the step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorSystem<L: LoadTorque> {
    /// The motor.
    pub motor: MotorModel,
    /// The load.
    pub load: L,
    /// Duty cycle, supply voltage and drive, held for the step.
    pub input: DriveInput,
}

impl<L: LoadTorque> System for MotorSystem<L> {
    type Coordinates = MotorSpeed;

    fn rate(&self, _: f64, y: &MotorSpeed) -> MotorSpeed {
        let load = self.load.torque_n_m(y.speed_rad_s);
        MotorSpeed {
            speed_rad_s: self.motor.acceleration(y.speed_rad_s, &self.input, load),
        }
    }
}

impl<L: LoadTorque> LieSystem for MotorSystem<L> {
    type Tangent = MotorSpeed;

    fn retract(&self, base: &MotorSpeed, u: &MotorSpeed) -> MotorSpeed {
        *base + *u
    }

    fn tangent_rate(&self, time_s: f64, base: &MotorSpeed, u: &MotorSpeed) -> MotorSpeed {
        self.rate(time_s, &self.retract(base, u))
    }
}
