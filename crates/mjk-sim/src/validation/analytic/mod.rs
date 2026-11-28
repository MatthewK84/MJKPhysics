//! The analytic validation suite of [#16](https://github.com/MatthewK84/MJKPhysics/issues/16):
//! the simulation loop against closed-form solutions.
//!
//! Every physics check runs the real `World` loop with RKMK4 at the default 1 kHz: free fall, a
//! projectile with linear drag, a torque-free axisymmetric body and a body spinning near its
//! intermediate axis. The convergence-order checks run every integrator at four step sizes.
//!
//! Each tolerance comes from an error estimate computed before the run, never from the result:
//! [`tolerance_from`] is ten times the estimate, rounded up to a power of ten. Every check states
//! its estimate in the published table.

mod bodies;
mod elliptic;
mod motion;
mod order;

use std::fmt;

use mjk_dynamics::mass::MassError;
use mjk_dynamics::{DEFAULT_PHYSICS_HZ, IntegratorKind, RateError};

pub use super::check::{Check, Measure, markdown, tolerance_from};
pub use elliptic::complete_elliptic_k;
pub use motion::{axisymmetric_precession, free_fall, intermediate_axis, projectile_with_drag};
pub use order::{convergence_orders, fitted_order, nominal_order};

/// The physics rate of the physics checks.
pub const PHYSICS_HZ: u32 = DEFAULT_PHYSICS_HZ;

/// Why the suite could not run. None of these happen with its fixed configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AnalyticError {
    /// A test body's mass properties are not physical.
    Mass(MassError),
    /// A physics rate is zero.
    Rate(RateError),
}

impl fmt::Display for AnalyticError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Mass(error) => write!(f, "a test body is not physical: {error}"),
            Self::Rate(error) => write!(f, "a physics rate is not usable: {error}"),
        }
    }
}

impl From<MassError> for AnalyticError {
    fn from(error: MassError) -> Self {
        Self::Mass(error)
    }
}

impl From<RateError> for AnalyticError {
    fn from(error: RateError) -> Self {
        Self::Rate(error)
    }
}

/// True if every check passed.
#[must_use]
pub fn passed(checks: &[Check]) -> bool {
    checks.iter().all(Check::passed)
}

/// Every check, physics first, then convergence orders.
///
/// # Errors
///
/// Only if a test body were not physical, which tests rule out.
pub fn analytic_study() -> Result<Vec<Check>, AnalyticError> {
    let method = IntegratorKind::Rkmk4;
    let mut checks = Vec::new();
    checks.extend(free_fall(method)?);
    checks.extend(projectile_with_drag(method)?);
    checks.extend(axisymmetric_precession(method)?);
    checks.extend(intermediate_axis(method)?);
    checks.extend(convergence_orders()?);
    Ok(checks)
}
