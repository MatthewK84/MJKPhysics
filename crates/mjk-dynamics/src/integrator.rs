//! Fixed-step integrators, Eqs. (INT-1) to (INT-3); see ADR 0002.
//!
//! An integrator advances any [`System`] of ordinary differential equations by one step. Systems
//! work in [`Coordinates`]: a vector space, so stages can be added and scaled, split into
//! position-like and velocity-like parts for semi-implicit Euler. The rigid body is one system;
//! motor and battery states join it in later milestones without changing the integrators.

use std::ops::{Add, Mul};

/// The coordinates an integrator advances. The same type holds their rate of change.
pub trait Coordinates: Copy + Add<Output = Self> + Mul<f64, Output = Self> {
    /// Only the position-like components (positions, attitude), the others zero.
    #[must_use]
    fn positions(self) -> Self;

    /// Only the velocity-like components (velocities, angular velocities, first-order states),
    /// the others zero.
    #[must_use]
    fn velocities(self) -> Self;

    /// Restores the invariants of the coordinates after a step, such as a unit attitude
    /// quaternion. Called once per step, never on intermediate stages.
    #[must_use]
    fn project(self) -> Self;
}

/// A system of ordinary differential equations ẏ = f(t, y).
pub trait System {
    /// The coordinates `y`.
    type Coordinates: Coordinates;

    /// The rate ẏ at time `time_s`. The position-like part of the rate depends only on the
    /// coordinates (it is kinematics), never on time.
    fn rate(&self, time_s: f64, y: &Self::Coordinates) -> Self::Coordinates;
}

/// A method that advances a [`System`] by one fixed step.
pub trait Integrator {
    /// The coordinates at `time_s + dt_s`, starting from `y` at `time_s`.
    fn step<S: System>(
        &self,
        system: &S,
        time_s: f64,
        y: &S::Coordinates,
        dt_s: f64,
    ) -> S::Coordinates;
}

/// Explicit (forward) Euler, Eq. (INT-1). First order; a reference method only.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ExplicitEuler;

impl Integrator for ExplicitEuler {
    fn step<S: System>(
        &self,
        system: &S,
        time_s: f64,
        y: &S::Coordinates,
        dt_s: f64,
    ) -> S::Coordinates {
        // Eq. (INT-1)
        (*y + system.rate(time_s, y) * dt_s).project()
    }
}

/// Semi-implicit (symplectic) Euler, Eq. (INT-2): velocities first, then positions from the new
/// velocities. First order, but energy errors stay bounded on oscillators.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SemiImplicitEuler;

impl Integrator for SemiImplicitEuler {
    fn step<S: System>(
        &self,
        system: &S,
        time_s: f64,
        y: &S::Coordinates,
        dt_s: f64,
    ) -> S::Coordinates {
        // Eq. (INT-2)
        let kicked = *y + system.rate(time_s, y).velocities() * dt_s;
        (kicked + system.rate(time_s, &kicked).positions() * dt_s).project()
    }
}

/// The classical fourth-order Runge–Kutta method, Eq. (INT-3).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Rk4;

impl Integrator for Rk4 {
    fn step<S: System>(
        &self,
        system: &S,
        time_s: f64,
        y: &S::Coordinates,
        dt_s: f64,
    ) -> S::Coordinates {
        // Eq. (INT-3)
        let half = 0.5 * dt_s;
        let k1 = system.rate(time_s, y);
        let k2 = system.rate(time_s + half, &(*y + k1 * half));
        let k3 = system.rate(time_s + half, &(*y + k2 * half));
        let k4 = system.rate(time_s + dt_s, &(*y + k3 * dt_s));
        (*y + (k1 + (k2 + k3) * 2.0 + k4) * (dt_s / 6.0)).project()
    }
}
