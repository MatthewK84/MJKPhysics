//! The measured convergence order of every integrator, on a translation and a rotation that both
//! have closed forms: linear drag, and the precession of an axisymmetric body, Eq. (VAL-9).

use mjk_dynamics::IntegratorKind;
use mjk_dynamics::mass::MassProperties;
use mjk_math::{Body, Mat3, Vec3, World};

use super::bodies::{TorqueFree, WeightAndDrag, angle_between, run, simulation, start};
use super::motion::{Precession, attitude, drag_closed_form};
use super::{AnalyticError, Check};

/// Physics rates: steps from 1/16 s to 1/128 s, halving.
const RATES_HZ: [u32; 4] = [16, 32, 64, 128];

/// The length of each run, s.
const DURATION_S: u32 = 4;

/// The tolerance on a measured order.
const ORDER_TOLERANCE: f64 = 0.2;

/// The drag problem's time constant, s.
const TAU_S: f64 = 1.0;

/// The rotation problem's moments and initial body rates.
const TRANSVERSE: f64 = 0.02;
const AXIAL: f64 = 0.035;
const OMEGA: [f64; 3] = [0.3, 0.0, 1.0];

/// The order of every method on both problems.
///
/// # Errors
///
/// Only if a test body were not physical, which tests rule out.
pub fn convergence_orders() -> Result<Vec<Check>, AnalyticError> {
    let mut checks = Vec::with_capacity(2 * IntegratorKind::ALL.len());
    let why = why();
    for method in IntegratorKind::ALL {
        let nominal = nominal_order(method);
        let translation = fitted_order(translation_errors(method)?);
        let rotation = fitted_order(rotation_errors(method)?);
        checks.push(
            Check::order(
                name(method, "translation (linear drag)"),
                nominal,
                translation,
            )
            .with_tolerance(ORDER_TOLERANCE, why.clone()),
        );
        checks.push(
            Check::order(name(method, "rotation (precession)"), nominal, rotation)
                .with_tolerance(ORDER_TOLERANCE, why.clone()),
        );
    }
    Ok(checks)
}

fn name(method: IntegratorKind, problem: &str) -> String {
    let method = match method {
        IntegratorKind::ExplicitEuler => "explicit Euler",
        IntegratorKind::SemiImplicitEuler => "semi-implicit Euler",
        IntegratorKind::Rk4 => "RK4",
        IntegratorKind::Rkmk4 => "RKMK4",
    };
    format!("Convergence order: {method}, {problem}")
}

/// The order each method is built to have.
#[must_use]
pub const fn nominal_order(method: IntegratorKind) -> f64 {
    match method {
        IntegratorKind::ExplicitEuler | IntegratorKind::SemiImplicitEuler => 1.0,
        IntegratorKind::Rk4 | IntegratorKind::Rkmk4 => 4.0,
    }
}

/// Why ±0.2: how far from the asymptotic regime the largest step is.
fn why() -> String {
    let h = 1.0 / f64::from(RATES_HZ[0]);
    let precession = Precession::of(TRANSVERSE, AXIAL, Vec3::<Body>::from_array(OMEGA));
    let fastest = precession.space_rate();
    format!(
        "Slope of ln(error) against ln(h) over h = 1/16 to 1/128 s. At the largest step \
         h·ω = {:.2} and h/τ = {:.4}: the next term of the error expansion shifts the slope by \
         about that much, and ±0.2 is about twice it.",
        h * fastest,
        h / TAU_S
    )
}

/// Position errors under gravity and linear drag after the run, at each rate.
fn translation_errors(method: IntegratorKind) -> Result<[f64; 4], AnalyticError> {
    let launch = Vec3::<World>::new(2.0, 0.0, -1.0);
    let mut errors = [0.0; 4];
    for (error, hz) in errors.iter_mut().zip(RATES_HZ) {
        let props =
            MassProperties::new(1.0, Vec3::zero(), Mat3::from_diagonal([0.02, 0.03, 0.04]))?;
        let loads = WeightAndDrag {
            drag_n_s_m: 1.0 / TAU_S,
        };
        let mut sim = simulation(loads, props, start(launch, Vec3::zero()), hz, method)?;
        run(&mut sim, DURATION_S * hz);
        let (position, _) = drag_closed_form(launch, TAU_S, sim.clock().time_s());
        *error = (sim.current().position - position).norm();
    }
    Ok(errors)
}

/// Attitude errors of a torque-free axisymmetric body after the run, at each rate.
fn rotation_errors(method: IntegratorKind) -> Result<[f64; 4], AnalyticError> {
    let omega = Vec3::<Body>::from_array(OMEGA);
    let precession = Precession::of(TRANSVERSE, AXIAL, omega);
    let mut errors = [0.0; 4];
    for (error, hz) in errors.iter_mut().zip(RATES_HZ) {
        let moments = [TRANSVERSE, TRANSVERSE, AXIAL];
        let props = MassProperties::new(1.0, Vec3::zero(), Mat3::from_diagonal(moments))?;
        let mut sim = simulation(TorqueFree, props, start(Vec3::zero(), omega), hz, method)?;
        run(&mut sim, DURATION_S * hz);
        let expected = precession.attitude(sim.clock().time_s());
        *error = angle_between(expected, attitude(sim.current().attitude));
    }
    Ok(errors)
}

/// The least-squares slope of `ln(error)` against `ln(h)`.
#[must_use]
pub fn fitted_order(errors: [f64; 4]) -> f64 {
    let points: Vec<(f64, f64)> = RATES_HZ
        .iter()
        .zip(errors)
        .map(|(hz, error)| (-libm::log(f64::from(*hz)), libm::log(error)))
        .collect();
    let n = 4.0;
    let mean_x = points.iter().map(|p| p.0).sum::<f64>() / n;
    let mean_y = points.iter().map(|p| p.1).sum::<f64>() / n;
    let covariance: f64 = points.iter().map(|p| (p.0 - mean_x) * (p.1 - mean_y)).sum();
    let variance: f64 = points.iter().map(|p| (p.0 - mean_x) * (p.0 - mean_x)).sum();
    covariance / variance
}
