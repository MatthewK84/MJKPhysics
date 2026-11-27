//! Fixed-step integrators, Eqs. (INT-1) to (INT-3) and (INT-7); see ADR 0002.
//!
//! An integrator advances a [`System`] of ordinary differential equations by one step. Systems
//! work in [`Coordinates`]: a vector space, so stages can be added and scaled, split into
//! position-like and velocity-like parts for semi-implicit Euler. A [`LieSystem`] also offers a
//! chart in which part of its state lives on a Lie group, which the Runge–Kutta–Munthe-Kaas
//! method ([`Rkmk4`]) steps in. The rigid body is one system; motor and battery states join it in
//! later milestones without changing the integrators.

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

/// A system whose state is partly on a Lie group, with a chart around any base point.
///
/// Near a base point `y₀`, the state is `retract(y₀, u)` for a tangent vector `u`: group-valued
/// parts move by the exponential map, the others by addition. In the chart the equations of
/// motion become an ordinary differential equation for `u` in a vector space, Eq. (INT-6).
pub trait LieSystem: System {
    /// Tangent coordinates of the chart. The default value is the zero vector, which maps to the
    /// base point itself.
    type Tangent: Coordinates + Default;

    /// The state reached from `base` by the tangent vector `u`.
    fn retract(&self, base: &Self::Coordinates, u: &Self::Tangent) -> Self::Coordinates;

    /// The rate of the chart coordinates, u̇, at the state `retract(base, u)` and time `time_s`.
    fn tangent_rate(
        &self,
        time_s: f64,
        base: &Self::Coordinates,
        u: &Self::Tangent,
    ) -> Self::Tangent;
}

/// A method that advances a system of type `S` by one fixed step.
///
/// Every method implements this one trait (ADR 0002), so methods can be swapped and compared;
/// [`Rkmk4`] implements it for systems that offer a Lie-group chart.
pub trait Integrator<S: System> {
    /// The coordinates at `time_s + dt_s`, starting from `y` at `time_s`.
    fn step(&self, system: &S, time_s: f64, y: &S::Coordinates, dt_s: f64) -> S::Coordinates;
}

/// Explicit (forward) Euler, Eq. (INT-1). First order; a reference method only.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ExplicitEuler;

impl<S: System> Integrator<S> for ExplicitEuler {
    fn step(&self, system: &S, time_s: f64, y: &S::Coordinates, dt_s: f64) -> S::Coordinates {
        // Eq. (INT-1)
        (*y + system.rate(time_s, y) * dt_s).project()
    }
}

/// Semi-implicit (symplectic) Euler, Eq. (INT-2): velocities first, then positions from the new
/// velocities. First order, but energy errors stay bounded on oscillators.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SemiImplicitEuler;

impl<S: System> Integrator<S> for SemiImplicitEuler {
    fn step(&self, system: &S, time_s: f64, y: &S::Coordinates, dt_s: f64) -> S::Coordinates {
        // Eq. (INT-2)
        let kicked = *y + system.rate(time_s, y).velocities() * dt_s;
        (kicked + system.rate(time_s, &kicked).positions() * dt_s).project()
    }
}

/// The classical fourth-order Runge–Kutta method, Eq. (INT-3).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Rk4;

impl<S: System> Integrator<S> for Rk4 {
    fn step(&self, system: &S, time_s: f64, y: &S::Coordinates, dt_s: f64) -> S::Coordinates {
        // Eq. (INT-3)
        let half = 0.5 * dt_s;
        let k1 = system.rate(time_s, y);
        let k2 = system.rate(time_s + half, &(*y + k1 * half));
        let k3 = system.rate(time_s + half, &(*y + k2 * half));
        let k4 = system.rate(time_s + dt_s, &(*y + k3 * dt_s));
        (*y + (k1 + (k2 + k3) * 2.0 + k4) * (dt_s / 6.0)).project()
    }
}

/// The Runge–Kutta–Munthe-Kaas method of order four, Eq. (INT-7): classical RK4 applied to the
/// chart coordinates around the state at the start of the step, then mapped back with the
/// exponential map. The group-valued part never leaves the group, so nothing is renormalized.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Rkmk4;

impl<S: LieSystem> Integrator<S> for Rkmk4 {
    fn step(&self, system: &S, time_s: f64, y: &S::Coordinates, dt_s: f64) -> S::Coordinates {
        // Eq. (INT-7)
        let half = 0.5 * dt_s;
        let k1 = system.tangent_rate(time_s, y, &S::Tangent::default());
        let k2 = system.tangent_rate(time_s + half, y, &(k1 * half));
        let k3 = system.tangent_rate(time_s + half, y, &(k2 * half));
        let k4 = system.tangent_rate(time_s + dt_s, y, &(k3 * dt_s));
        let u = (k1 + (k2 + k3) * 2.0 + k4) * (dt_s / 6.0);
        system.retract(y, &u)
    }
}

/// An integrator chosen at run time, for example from a scenario file.
///
/// Dispatches with `match` rather than a trait object (Power of Ten rule 9). Every method is
/// available, so the system must offer a Lie-group chart for [`Rkmk4`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegratorKind {
    /// [`ExplicitEuler`].
    ExplicitEuler,
    /// [`SemiImplicitEuler`].
    SemiImplicitEuler,
    /// [`Rk4`].
    Rk4,
    /// [`Rkmk4`].
    Rkmk4,
}

impl IntegratorKind {
    /// Every method.
    pub const ALL: [Self; 4] = [
        Self::ExplicitEuler,
        Self::SemiImplicitEuler,
        Self::Rk4,
        Self::Rkmk4,
    ];

    /// The name used in scenario files.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::ExplicitEuler => "explicit-euler",
            Self::SemiImplicitEuler => "semi-implicit-euler",
            Self::Rk4 => "rk4",
            Self::Rkmk4 => "rkmk4",
        }
    }

    /// The method with this scenario-file name, if there is one.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|kind| kind.name() == name)
    }
}

impl<S: LieSystem> Integrator<S> for IntegratorKind {
    fn step(&self, system: &S, time_s: f64, y: &S::Coordinates, dt_s: f64) -> S::Coordinates {
        match self {
            Self::ExplicitEuler => ExplicitEuler.step(system, time_s, y, dt_s),
            Self::SemiImplicitEuler => SemiImplicitEuler.step(system, time_s, y, dt_s),
            Self::Rk4 => Rk4.step(system, time_s, y, dt_s),
            Self::Rkmk4 => Rkmk4.step(system, time_s, y, dt_s),
        }
    }
}
