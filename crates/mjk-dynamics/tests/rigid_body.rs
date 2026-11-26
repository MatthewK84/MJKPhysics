//! The Newton–Euler equations (#10): closed-form cases and identities that every correct
//! derivative satisfies, for any state, mass properties and loads.

use mjk_dynamics::mass::{Component, MassProperties, Shape, composite};
use mjk_dynamics::{RigidBodyState, derivative};
use mjk_math::{Body, Mat3, Part, Quat, Rotation, UnitQuat, Vec3, World};
use proptest::prelude::*;

const CASES: u32 = 512;

fn principal(moments: [f64; 3]) -> Option<MassProperties> {
    MassProperties::new(2.0, Vec3::zero(), Mat3::from_diagonal(moments)).ok()
}

#[test]
fn translation_is_force_over_mass_and_velocity_passes_through() {
    let props = principal([0.02, 0.03, 0.04]).expect("physical");
    let mut state = RigidBodyState::at_rest(Vec3::new(1.0, 2.0, -3.0), Rotation::identity());
    state.velocity_m_s = Vec3::new(4.0, -5.0, 6.0);
    state.omega_rad_s = Vec3::new(0.1, 0.2, 0.3);
    let rates = derivative(&state, &props, Vec3::new(2.0, -4.0, 19.6), Vec3::zero());
    assert_eq!(rates.velocity_m_s, state.velocity_m_s);
    assert_eq!(rates.acceleration_m_s2, Vec3::new(1.0, -2.0, 9.8));
    assert_eq!(rates.omega_rad_s, state.omega_rad_s);
}

#[test]
fn euler_equations_in_principal_axes() {
    // J₁ω̇₁ = (J₂ − J₃)ω₂ω₃ + M₁, and cyclically.
    let (j1, j2, j3) = (0.02, 0.03, 0.05);
    let props = principal([j1, j2, j3]).expect("physical");
    let mut state = RigidBodyState::at_rest(Vec3::zero(), Rotation::identity());
    state.omega_rad_s = Vec3::new(1.0, -2.0, 3.0);
    let moment = Vec3::new(0.01, 0.02, -0.03);
    let rates = derivative(&state, &props, Vec3::zero(), moment);
    let [w1, w2, w3] = [1.0, -2.0, 3.0];
    let expected = Vec3::new(
        ((j2 - j3) * w2 * w3 + 0.01) / j1,
        ((j3 - j1) * w3 * w1 + 0.02) / j2,
        ((j1 - j2) * w1 * w2 - 0.03) / j3,
    );
    assert!((rates.angular_acceleration_rad_s2 - expected).norm() < 1e-12);
}

#[test]
fn spin_about_a_principal_axis_is_steady() {
    let props = principal([0.02, 0.03, 0.05]).expect("physical");
    let mut state = RigidBodyState::at_rest(Vec3::zero(), Rotation::identity());
    state.omega_rad_s = Vec3::new(0.0, 7.0, 0.0);
    let rates = derivative(&state, &props, Vec3::zero(), Vec3::zero());
    assert!(rates.angular_acceleration_rad_s2.norm() < 1e-15);
}

#[test]
fn energy_and_momentum_of_a_simple_motion() {
    // 2 kg at 3 m/s: 9 J; ω = (0, 0, 2) about J₃ = 0.05: 0.1 J and |H| = 0.1 N·m·s.
    let props = principal([0.02, 0.03, 0.05]).expect("physical");
    let quarter = Rotation::from_unit_quat(UnitQuat::exp([std::f64::consts::FRAC_PI_2, 0.0, 0.0]));
    let mut state = RigidBodyState::at_rest(Vec3::zero(), quarter);
    state.velocity_m_s = Vec3::new(0.0, 3.0, 0.0);
    state.omega_rad_s = Vec3::new(0.0, 0.0, 2.0);
    assert!((state.kinetic_energy_j(&props) - 9.1).abs() < 1e-14);
    // A quarter turn about x maps body z to world −y.
    let h_w = state.angular_momentum_w(&props);
    assert!((h_w - Vec3::<World>::new(0.0, -0.1, 0.0)).norm() < 1e-15);
}

#[test]
fn finiteness_covers_every_part_of_the_state() {
    let good = RigidBodyState::at_rest(Vec3::zero(), Rotation::identity());
    assert!(good.is_finite());
    let parts: [fn(&mut RigidBodyState); 3] = [
        |s| s.position_m = Vec3::new(f64::NAN, 0.0, 0.0),
        |s| s.velocity_m_s = Vec3::new(0.0, f64::INFINITY, 0.0),
        |s| s.omega_rad_s = Vec3::new(0.0, 0.0, f64::NAN),
    ];
    for spoil in parts {
        let mut bad = good;
        spoil(&mut bad);
        assert!(!bad.is_finite());
    }
}

// ---------------------------------------------------------------- identities

fn unit_quat() -> impl Strategy<Value = UnitQuat> {
    (-1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64).prop_filter_map(
        "normalizable",
        |(w, x, y, z)| {
            let q = Quat::new(w, x, y, z);
            (q.norm() > 0.1).then(|| UnitQuat::new(q)).flatten()
        },
    )
}

fn vector<F: mjk_math::Frame>(scale: f64) -> impl Strategy<Value = Vec3<F>> {
    (-scale..scale, -scale..scale, -scale..scale).prop_map(|(x, y, z)| Vec3::new(x, y, z))
}

/// Physical mass properties with products of inertia: a box turned to a random orientation.
fn mass_properties() -> impl Strategy<Value = MassProperties> {
    (
        0.1..5.0_f64,
        0.02..0.6_f64,
        0.02..0.6_f64,
        0.02..0.6_f64,
        unit_quat(),
    )
        .prop_filter_map("physical", |(mass, a, b, c, q)| {
            let turn = Rotation::<Body, Part>::from_unit_quat(q);
            let part = Component::new(mass, Shape::Box { size_m: [a, b, c] }, Vec3::zero());
            composite(&[part.rotated(turn)]).ok()
        })
}

fn state() -> impl Strategy<Value = RigidBodyState> {
    (
        vector::<World>(100.0),
        vector::<World>(30.0),
        unit_quat(),
        vector::<Body>(20.0),
    )
        .prop_map(
            |(position_m, velocity_m_s, q, omega_rad_s)| RigidBodyState {
                position_m,
                velocity_m_s,
                q_wb: Rotation::from_unit_quat(q),
                omega_rad_s,
            },
        )
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(CASES))]

    /// d/dt (½m|v|² + ½ωᵀJω) = F·v + M·ω: the gyroscopic term does no work.
    #[test]
    fn power_balance(
        s in state(),
        props in mass_properties(),
        force in vector::<World>(50.0),
        moment in vector::<Body>(2.0),
    ) {
        let rates = derivative(&s, &props, force, moment);
        let j = props.inertia_kg_m2();
        let energy_rate = props.mass_kg() * s.velocity_m_s.dot(rates.acceleration_m_s2)
            + s.omega_rad_s.dot(j * rates.angular_acceleration_rad_s2);
        let power = force.dot(s.velocity_m_s) + moment.dot(s.omega_rad_s);
        let scale = force.norm() * s.velocity_m_s.norm()
            + moment.norm() * s.omega_rad_s.norm()
            + s.omega_rad_s.norm() * (j * s.omega_rad_s).norm() * s.omega_rad_s.norm();
        prop_assert!((energy_rate - power).abs() <= 1e-12 * scale.max(1e-300));
    }

    /// Without a moment, angular momentum in world axes is constant:
    /// d/dt (R·J·ω) = R·(J·ω̇ + ω × J·ω) = 0.
    #[test]
    fn torque_free_motion_conserves_angular_momentum(s in state(), props in mass_properties()) {
        let rates = derivative(&s, &props, Vec3::zero(), Vec3::zero());
        let j = props.inertia_kg_m2();
        let h = j * s.omega_rad_s;
        let rate_b = j * rates.angular_acceleration_rad_s2 + s.omega_rad_s.cross(h);
        let scale = s.omega_rad_s.norm() * h.norm();
        prop_assert!(rate_b.norm() <= 1e-12 * scale.max(1e-300));
    }

    /// The moment equation holds exactly as written: J·ω̇ = M − ω × (J·ω).
    #[test]
    fn moment_equation_holds(
        s in state(),
        props in mass_properties(),
        moment in vector::<Body>(2.0),
    ) {
        let rates = derivative(&s, &props, Vec3::zero(), moment);
        let j = props.inertia_kg_m2();
        let lhs = j * rates.angular_acceleration_rad_s2;
        let rhs = moment - s.omega_rad_s.cross(j * s.omega_rad_s);
        prop_assert!((lhs - rhs).norm() <= 1e-12 * rhs.norm().max(moment.norm()).max(1e-300));
    }
}
