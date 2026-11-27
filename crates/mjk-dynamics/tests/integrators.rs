//! The fixed-step integrators (#11): closed forms that distinguish the three methods,
//! measured convergence order, energy behavior on an oscillator, and the attitude invariant.

use std::ops::{Add, Mul};

use mjk_dynamics::mass::MassProperties;
use mjk_dynamics::{
    Coordinates, ExplicitEuler, Integrator, IntegratorKind, LieSystem, LoadModel, Loads,
    RigidBodyCoordinates, RigidBodyState, RigidBodySystem, RigidBodyTangent, Rk4, Rkmk4,
    SemiImplicitEuler, System,
};
use mjk_math::{Body, Mat3, Quat, Rotation, UnitQuat, Vec3, World};

// ---------------------------------------------------------------- load models

#[derive(Clone, Copy, Debug)]
struct NoLoads;

impl LoadModel for NoLoads {
    fn loads(&self, _: f64, _: &RigidBodyState, _: &MassProperties) -> Loads {
        Loads::default()
    }
}

#[derive(Clone, Copy, Debug)]
struct ConstantForce(Vec3<World>);

impl LoadModel for ConstantForce {
    fn loads(&self, _: f64, _: &RigidBodyState, _: &MassProperties) -> Loads {
        Loads {
            force_w: self.0,
            moment_b: Vec3::zero(),
        }
    }
}

/// A force growing with time, chosen so the vertical velocity is t⁴: F = m·4t³ along z.
#[derive(Clone, Copy, Debug)]
struct QuarticForce;

impl LoadModel for QuarticForce {
    fn loads(&self, time_s: f64, _: &RigidBodyState, props: &MassProperties) -> Loads {
        Loads {
            force_w: Vec3::new(0.0, 0.0, props.mass_kg() * 4.0 * time_s.powi(3)),
            moment_b: Vec3::zero(),
        }
    }
}

/// A linear spring to the origin: F = −k·p.
#[derive(Clone, Copy, Debug)]
struct Spring {
    stiffness_n_m: f64,
}

impl LoadModel for Spring {
    fn loads(&self, _: f64, state: &RigidBodyState, _: &MassProperties) -> Loads {
        Loads {
            force_w: state.position_m * -self.stiffness_n_m,
            moment_b: Vec3::zero(),
        }
    }
}

fn body(mass_kg: f64, inertia: [[f64; 3]; 3]) -> Option<MassProperties> {
    MassProperties::new(mass_kg, Vec3::zero(), Mat3::from_rows(inertia)).ok()
}

fn diagonal(moments: [f64; 3]) -> [[f64; 3]; 3] {
    Mat3::<Body>::from_diagonal(moments).rows()
}

fn start(velocity: Vec3<World>, omega: Vec3<Body>) -> RigidBodyCoordinates {
    let mut state = RigidBodyState::at_rest(Vec3::zero(), Rotation::identity());
    state.velocity_m_s = velocity;
    state.omega_rad_s = omega;
    RigidBodyCoordinates::from(state)
}

fn run<S: System, I: Integrator<S>>(
    method: &I,
    system: &S,
    y0: S::Coordinates,
    dt_s: f64,
    steps: u32,
) -> S::Coordinates {
    (0..steps).fold(y0, |y, n| {
        method.step(system, f64::from(n) * dt_s, &y, dt_s)
    })
}

// ---------------------------------------------------------------- closed forms

#[test]
fn each_method_has_its_own_closed_form_under_constant_acceleration() {
    // a = F/m = 10 m/s² along z, v₀ = 1 m/s along x, h = 1/8 s, n = 8 steps (t = 1 s).
    // Explicit Euler: z = h²·a·n(n−1)/2 = 4.375; semi-implicit: h²·a·n(n+1)/2 = 5.625;
    // RK4 and RKMK4 are exact for a quadratic: ½·a·t² = 5. All move x by v₀·t = 1.
    let props = body(2.0, diagonal([0.02, 0.03, 0.04])).expect("physical");
    let system = RigidBodySystem {
        props,
        loads: ConstantForce(Vec3::new(0.0, 0.0, 20.0)),
    };
    let y0 = start(Vec3::new(1.0, 0.0, 0.0), Vec3::zero());
    let cases: [(f64, RigidBodyCoordinates); 4] = [
        (4.375, run(&ExplicitEuler, &system, y0, 0.125, 8)),
        (5.625, run(&SemiImplicitEuler, &system, y0, 0.125, 8)),
        (5.0, run(&Rk4, &system, y0, 0.125, 8)),
        (5.0, run(&Rkmk4, &system, y0, 0.125, 8)),
    ];
    for (z_m, y) in cases {
        assert!(
            (y.position - Vec3::new(1.0, 0.0, z_m)).norm() < 1e-14,
            "{y:?}"
        );
        assert!(
            (y.velocity - Vec3::new(1.0, 0.0, 10.0)).norm() < 1e-14,
            "{y:?}"
        );
    }
}

/// A one-dimensional quantity with only a position-like part: ẏ = 4t³, so y = t⁴.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Scalar(f64);

impl Add for Scalar {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl Mul<f64> for Scalar {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self(self.0 * rhs)
    }
}

impl Coordinates for Scalar {
    fn positions(self) -> Self {
        self
    }
    fn velocities(self) -> Self {
        Self(0.0)
    }
    fn project(self) -> Self {
        self
    }
}

struct Quartic;

impl System for Quartic {
    type Coordinates = Scalar;
    fn rate(&self, time_s: f64, _: &Scalar) -> Scalar {
        Scalar(4.0 * time_s * time_s * time_s)
    }
}

#[test]
fn rk4_integrates_a_cubic_rate_exactly() {
    // RK4 reduces to Simpson's rule for ẏ = f(t), which is exact for cubics. Explicit Euler
    // gives the left Riemann sum: Σ 4(n/4)³/4 over n = 0..3 = 36/64.
    let rk4 = run(&Rk4, &Quartic, Scalar(0.0), 0.25, 4);
    assert!((rk4.0 - 1.0).abs() < 1e-15);
    let euler = run(&ExplicitEuler, &Quartic, Scalar(0.0), 0.25, 4);
    assert!((euler.0 - 36.0 / 64.0).abs() < 1e-15);
    let semi = run(&SemiImplicitEuler, &Quartic, Scalar(0.0), 0.25, 4);
    assert!((semi.0 - 36.0 / 64.0).abs() < 1e-15);
}

// ---------------------------------------------------------------- convergence order

/// Least-squares slope of ln(error) against ln(step).
fn fitted_order(steps: &[f64], errors: &[f64]) -> f64 {
    let points: Vec<(f64, f64)> = steps
        .iter()
        .zip(errors)
        .map(|(h, e)| (h.ln(), e.ln()))
        .collect();
    let count = f64::from(u32::try_from(points.len()).unwrap_or(u32::MAX));
    let mean_x = points.iter().map(|p| p.0).sum::<f64>() / count;
    let mean_y = points.iter().map(|p| p.1).sum::<f64>() / count;
    let covariance: f64 = points.iter().map(|p| (p.0 - mean_x) * (p.1 - mean_y)).sum();
    let variance: f64 = points.iter().map(|p| (p.0 - mean_x).powi(2)).sum();
    covariance / variance
}

/// Errors of a method at T = 10 s on x″ = −x, x(0) = 1, against x = cos t, v = −sin t.
fn oscillator_errors<I: Integrator<RigidBodySystem<Spring>>>(
    method: &I,
    step_counts: &[u32],
) -> Option<Vec<f64>> {
    let (system, y0) = oscillator()?;
    let exact_x = Vec3::new(10.0_f64.cos(), 0.0, 0.0);
    let exact_v = Vec3::new(-(10.0_f64.sin()), 0.0, 0.0);
    let errors = step_counts.iter().map(|&n| {
        let y = run(method, &system, y0, 10.0 / f64::from(n), n);
        (y.position - exact_x).norm() + (y.velocity - exact_v).norm()
    });
    Some(errors.collect())
}

#[test]
fn measured_order_on_an_oscillator() {
    let counts = [100, 200, 400, 800, 1600];
    let steps: Vec<f64> = counts.iter().map(|&n| 10.0 / f64::from(n)).collect();
    let expected = [
        (
            fitted_order(
                &steps,
                &oscillator_errors(&ExplicitEuler, &counts).expect("physical"),
            ),
            1.0,
        ),
        (
            fitted_order(
                &steps,
                &oscillator_errors(&SemiImplicitEuler, &counts).expect("physical"),
            ),
            1.0,
        ),
        (
            fitted_order(&steps, &oscillator_errors(&Rk4, &counts).expect("physical")),
            4.0,
        ),
        (
            fitted_order(
                &steps,
                &oscillator_errors(&Rkmk4, &counts).expect("physical"),
            ),
            4.0,
        ),
    ];
    for (measured, order) in expected {
        assert!(
            (measured - order).abs() <= 0.2,
            "order {measured} versus {order}"
        );
    }
}

fn tumbling_body() -> Option<RigidBodySystem<NoLoads>> {
    let inertia = [[0.02, 0.001, 0.0], [0.001, 0.03, 0.002], [0.0, 0.002, 0.04]];
    let props = body(1.0, inertia)?;
    Some(RigidBodySystem {
        props,
        loads: NoLoads,
    })
}

/// Attitude angle plus angular-velocity error at T = 2 s against an RK4 run with h = 1/3200 s.
fn tumbling_errors<I: Integrator<RigidBodySystem<NoLoads>>>(
    method: &I,
    step_counts: &[u32],
) -> Option<Vec<f64>> {
    let system = tumbling_body()?;
    let y0 = start(Vec3::zero(), Vec3::new(1.0, 0.5, 2.0));
    let reference = run(&Rk4, &system, y0, 2.0 / 6400.0, 6400);
    let q_ref = UnitQuat::new(reference.attitude)?;
    step_counts
        .iter()
        .map(|&n| {
            let y = run(method, &system, y0, 2.0 / f64::from(n), n);
            let q = UnitQuat::new(y.attitude)?;
            Some(q.angle_to(q_ref) + (y.angular_velocity - reference.angular_velocity).norm())
        })
        .collect()
}

#[test]
fn measured_order_on_a_tumbling_body() {
    let counts = [50, 100, 200, 400];
    let steps: Vec<f64> = counts.iter().map(|&n| 2.0 / f64::from(n)).collect();
    let expected = [
        (
            fitted_order(
                &steps,
                &tumbling_errors(&ExplicitEuler, &counts).expect("physical"),
            ),
            1.0,
        ),
        (
            fitted_order(
                &steps,
                &tumbling_errors(&SemiImplicitEuler, &counts).expect("physical"),
            ),
            1.0,
        ),
        (
            fitted_order(&steps, &tumbling_errors(&Rk4, &counts).expect("physical")),
            4.0,
        ),
        (
            fitted_order(&steps, &tumbling_errors(&Rkmk4, &counts).expect("physical")),
            4.0,
        ),
    ];
    for (measured, order) in expected {
        assert!(
            (measured - order).abs() <= 0.2,
            "order {measured} versus {order}"
        );
    }
}

// ---------------------------------------------------------------- energy and invariants

fn oscillator() -> Option<(RigidBodySystem<Spring>, RigidBodyCoordinates)> {
    let props = body(1.0, diagonal([0.02, 0.03, 0.04]))?;
    let system = RigidBodySystem {
        props,
        loads: Spring { stiffness_n_m: 1.0 },
    };
    let mut y0 = start(Vec3::zero(), Vec3::zero());
    y0.position = Vec3::new(1.0, 0.0, 0.0);
    Some((system, y0))
}

fn energy(y: &RigidBodyCoordinates) -> f64 {
    0.5 * (y.position.norm_squared() + y.velocity.norm_squared())
}

#[test]
fn semi_implicit_euler_keeps_oscillator_energy_bounded() {
    // 10⁶ steps of h = 0.01 s: about 1600 periods.
    let (system, y0) = oscillator().expect("physical");
    let e0 = energy(&y0);
    let mut y = y0;
    let mut worst: f64 = 0.0;
    for n in 0..1_000_000_u32 {
        y = SemiImplicitEuler.step(&system, f64::from(n) * 0.01, &y, 0.01);
        worst = worst.max((energy(&y) - e0).abs() / e0);
    }
    // The error oscillates with amplitude about h·ω/2 and does not drift.
    assert!(worst < 0.006, "worst relative energy error {worst}");
}

#[test]
fn explicit_euler_gains_energy_on_an_oscillator() {
    // With m = k = 1, each step multiplies the energy by exactly 1 + h².
    let (system, y0) = oscillator().expect("physical");
    let y = run(&ExplicitEuler, &system, y0, 0.01, 100_000);
    let growth = energy(&y) / energy(&y0);
    let expected = 1.0001_f64.powi(100_000);
    assert!(growth > 20_000.0);
    assert!(
        (growth / expected - 1.0).abs() < 1e-8,
        "{growth} versus {expected}"
    );
}

#[test]
fn the_attitude_stays_a_unit_quaternion() {
    let system = tumbling_body().expect("physical");
    let y0 = start(Vec3::zero(), Vec3::new(3.0, -4.0, 5.0));
    let after = [
        run(&ExplicitEuler, &system, y0, 0.01, 1000),
        run(&SemiImplicitEuler, &system, y0, 0.01, 1000),
        run(&Rk4, &system, y0, 0.01, 1000),
    ];
    for y in after {
        assert!(
            (y.attitude.norm() - 1.0).abs() < 1e-15,
            "{}",
            y.attitude.norm()
        );
    }
}

#[test]
fn coordinates_split_into_positions_and_velocities() {
    let y = RigidBodyCoordinates {
        position: Vec3::new(1.0, 2.0, 3.0),
        velocity: Vec3::new(4.0, 5.0, 6.0),
        attitude: Quat::new(0.5, 0.5, 0.5, 0.5),
        angular_velocity: Vec3::new(7.0, 8.0, 9.0),
    };
    let positions = y.positions();
    let velocities = y.velocities();
    assert_eq!(positions + velocities, y);
    assert_eq!(positions.velocity, Vec3::zero());
    assert_eq!(positions.angular_velocity, Vec3::zero());
    assert_eq!(velocities.position, Vec3::zero());
    assert_eq!(velocities.attitude, Quat::new(0.0, 0.0, 0.0, 0.0));
    let doubled = y * 2.0;
    assert_eq!(doubled, y + y);
}

#[test]
fn a_degenerate_attitude_is_reported_not_hidden() {
    let mut y = start(Vec3::zero(), Vec3::zero());
    y.attitude = Quat::new(0.0, 0.0, 0.0, 0.0);
    assert!(y.to_state().is_none());
    assert_eq!(y.project().attitude, Quat::new(0.0, 0.0, 0.0, 0.0));
    let rate = tumbling_body().expect("physical").rate(0.0, &y);
    assert!(rate.position.x.is_nan() && rate.angular_velocity.z.is_nan());
}

#[test]
fn round_trip_between_state_and_coordinates() {
    let mut state = RigidBodyState::at_rest(
        Vec3::new(1.0, 2.0, 3.0),
        Rotation::from_unit_quat(UnitQuat::exp([0.1, -0.2, 0.3])),
    );
    state.velocity_m_s = Vec3::new(-1.0, 0.5, 0.25);
    state.omega_rad_s = Vec3::new(0.3, 0.2, 0.1);
    let back = RigidBodyCoordinates::from(state).to_state().expect("unit");
    assert_eq!(back.position_m, state.position_m);
    assert_eq!(back.velocity_m_s, state.velocity_m_s);
    assert_eq!(back.omega_rad_s, state.omega_rad_s);
    assert!(back.q_wb.angle_to(state.q_wb) < 1e-15);
}

// ---------------------------------------------------------------- Lie-group method

#[test]
fn rkmk4_is_exact_for_steady_spin_about_a_principal_axis() {
    // With ω along a principal axis, ω stays constant and θ ∥ ω, so J_r⁻¹(θ)·ω = ω: RKMK4
    // reproduces q(t) = Exp(ω·t) to rounding. RK4 on the quaternion components truncates
    // exp(½Ω·h) after its fourth power at every step.
    let props = body(1.0, diagonal([0.02, 0.03, 0.04])).expect("physical");
    let system = RigidBodySystem {
        props,
        loads: NoLoads,
    };
    let y0 = start(Vec3::zero(), Vec3::new(0.0, 0.0, 3.0));
    let exact = UnitQuat::exp([0.0, 0.0, 15.0]);
    let lie = run(&Rkmk4, &system, y0, 0.05, 100);
    let classical = run(&Rk4, &system, y0, 0.05, 100);
    let lie_error = UnitQuat::new(lie.attitude).expect("unit").angle_to(exact);
    let classical_error = UnitQuat::new(classical.attitude)
        .expect("unit")
        .angle_to(exact);
    assert!(lie_error < 1e-13, "RKMK4 error {lie_error:e}");
    assert!(classical_error > 1e-9, "RK4 error {classical_error:e}");
}

#[test]
fn rkmk4_keeps_a_unit_quaternion_without_renormalizing() {
    // 10⁵ steps of h = 0.01 s on a tumbling body; RKMK4 never projects.
    let system = tumbling_body().expect("physical");
    let y0 = start(Vec3::zero(), Vec3::new(3.0, -4.0, 5.0));
    let y = run(&Rkmk4, &system, y0, 0.01, 100_000);
    let error = (y.attitude.norm() - 1.0).abs();
    assert!(error < 1e-12, "unit-norm error {error:e}");
}

#[test]
fn the_chart_is_centered_on_its_base_point() {
    let system = tumbling_body().expect("physical");
    let mut base = start(Vec3::new(1.0, 2.0, 3.0), Vec3::new(0.3, 0.2, 0.1));
    base.attitude = UnitQuat::exp([0.1, -0.2, 0.3]).quat();
    assert_eq!(system.retract(&base, &RigidBodyTangent::default()), base);
    let u = RigidBodyTangent {
        position: Vec3::new(1.0, -1.0, 0.5),
        velocity: Vec3::new(0.25, 0.5, -0.75),
        rotation: Vec3::new(0.0, 0.0, 0.5),
        angular_velocity: Vec3::new(-0.1, 0.2, 0.3),
    };
    let moved = system.retract(&base, &u);
    assert_eq!(moved.position, base.position + u.position);
    assert_eq!(moved.velocity, base.velocity + u.velocity);
    assert_eq!(
        moved.angular_velocity,
        base.angular_velocity + u.angular_velocity
    );
    let expected = UnitQuat::new(base.attitude * UnitQuat::exp([0.0, 0.0, 0.5]).quat());
    let turned = UnitQuat::new(moved.attitude).expect("unit");
    assert!(turned.angle_to(expected.expect("unit")) < 1e-15);
    assert_eq!(u.positions() + u.velocities(), u);
    assert_eq!(u.positions().velocity, Vec3::zero());
    assert_eq!(u.velocities().rotation, Vec3::zero());
    assert_eq!(u.project(), u);
    assert_eq!(u * 2.0, u + u);
}

#[test]
fn at_the_base_point_the_attitude_rate_is_the_angular_velocity() {
    // J_r⁻¹(0) = I: at θ = 0 the chart rate of the attitude is ω itself.
    let system = tumbling_body().expect("physical");
    let base = start(Vec3::zero(), Vec3::new(0.3, -0.2, 0.1));
    let rate = system.tangent_rate(0.0, &base, &RigidBodyTangent::default());
    assert!((rate.rotation - base.angular_velocity).norm() < 1e-16);
    let mut degenerate = base;
    degenerate.attitude = Quat::new(0.0, 0.0, 0.0, 0.0);
    let nan = system.tangent_rate(0.0, &degenerate, &RigidBodyTangent::default());
    assert!(nan.position.x.is_nan() && nan.rotation.z.is_nan());
}

#[test]
fn stages_are_evaluated_at_their_own_times() {
    // v̇ = 4t³ depends on time alone, so both RK4 and RKMK4 reduce to Simpson's rule for the
    // velocity and are exact: v_z(1) = 1. Wrong stage times break this.
    let props = body(2.0, diagonal([0.02, 0.03, 0.04])).expect("physical");
    let system = RigidBodySystem {
        props,
        loads: QuarticForce,
    };
    let y0 = start(Vec3::zero(), Vec3::zero());
    for y in [
        run(&Rk4, &system, y0, 0.25, 4),
        run(&Rkmk4, &system, y0, 0.25, 4),
    ] {
        assert!((y.velocity.z - 1.0).abs() < 1e-15, "{y:?}");
    }
}

#[test]
fn a_method_chosen_at_run_time_steps_exactly_like_the_method_itself() {
    let system = tumbling_body().expect("physical");
    let y0 = start(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.5, 2.0));
    let direct = [
        run(&ExplicitEuler, &system, y0, 0.01, 50),
        run(&SemiImplicitEuler, &system, y0, 0.01, 50),
        run(&Rk4, &system, y0, 0.01, 50),
        run(&Rkmk4, &system, y0, 0.01, 50),
    ];
    for (kind, expected) in IntegratorKind::ALL.iter().zip(direct) {
        assert_eq!(run(kind, &system, y0, 0.01, 50), expected, "{kind:?}");
    }
    let names = ["explicit-euler", "semi-implicit-euler", "rk4", "rkmk4"];
    for (kind, name) in IntegratorKind::ALL.iter().zip(names) {
        assert_eq!(kind.name(), name);
        assert_eq!(IntegratorKind::from_name(name), Some(*kind));
    }
    assert_eq!(IntegratorKind::from_name("rk45"), None);
}
