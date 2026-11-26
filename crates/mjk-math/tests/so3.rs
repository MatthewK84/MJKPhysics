//! Property tests for the SO(3) maps and attitude utilities (#9).

use std::f64::consts::{FRAC_PI_2, PI};

use mjk_math::so3::{left_jacobian, right_jacobian, right_jacobian_inverse};
use mjk_math::{Body, EulerAngles, Mat3, Quat, Rotation, UnitQuat, Vec3, World};
use proptest::prelude::*;

const CASES: u32 = 512;

fn unit_quat() -> impl Strategy<Value = UnitQuat> {
    (-1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64)
        .prop_filter("away from zero", |(w, x, y, z)| {
            w * w + x * x + y * y + z * z > 1e-2
        })
        .prop_filter_map("normalizable", |(w, x, y, z)| {
            UnitQuat::new(Quat::new(w, x, y, z))
        })
}

fn attitude() -> impl Strategy<Value = Rotation<World, Body>> {
    unit_quat().prop_map(Rotation::from_unit_quat)
}

fn direction() -> impl Strategy<Value = Vec3<Body>> {
    (-1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64)
        .prop_map(|(x, y, z)| Vec3::new(x, y, z))
        .prop_filter_map("non-zero", Vec3::normalized)
}

/// Rotation vectors with magnitude log-uniform in [1e-12, max].
fn rotation_vector(max_angle: f64) -> impl Strategy<Value = Vec3<Body>> {
    (direction(), -12.0..max_angle.log10()).prop_map(|(axis, exponent)| axis * 10f64.powf(exponent))
}

fn identity_error(m: Mat3<Body>) -> f64 {
    (m - Mat3::identity()).max_abs()
}

fn rotation_matrix(q: Rotation<Body, Body>) -> Mat3<Body> {
    Mat3::from_rows(q.matrix())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(CASES))]

    #[test]
    fn log_inverts_exp_including_tiny_angles(theta in rotation_vector(PI - 1e-6)) {
        let back = Rotation::<Body, Body>::exp(theta).log();
        let error = (back - theta).norm();
        prop_assert!(error <= 1e-12, "absolute error {error:e} at |θ| = {:e}", theta.norm());
        prop_assert!(error <= 1e-12 * theta.norm(), "relative error at |θ| = {:e}", theta.norm());
    }

    #[test]
    fn exp_inverts_log_and_log_takes_the_short_way(q in attitude()) {
        let theta = Rotation::<World, World>::from_unit_quat(q.unit_quat()).log();
        prop_assert!(theta.norm() <= PI + 1e-15);
        let back = Rotation::<World, World>::exp(theta);
        prop_assert!(back.unit_quat().angle_to(q.unit_quat()) <= 1e-15 * 8.0);
        let negated = UnitQuat::new(-q.unit_quat().quat()).map(UnitQuat::log);
        prop_assert!(negated.is_some_and(|n| (Vec3::<World>::from_array(n) - theta).norm() <= 1e-15 * 8.0));
    }

    #[test]
    fn jacobian_identities(theta in rotation_vector(PI - 1e-3)) {
        let jr = right_jacobian(theta);
        prop_assert!(identity_error(jr * right_jacobian_inverse(theta)) <= 1e-13);
        prop_assert!((left_jacobian(theta) - jr.transpose()).max_abs() <= 1e-15);
        let rotated = rotation_matrix(Rotation::exp(theta)) * jr;
        prop_assert!((left_jacobian(theta) - rotated).max_abs() <= 1e-14);
    }

    #[test]
    fn right_jacobian_linearizes_exp(theta in rotation_vector(PI - 0.05), axis in direction()) {
        let delta = axis * 1e-5;
        let base = Rotation::<Body, Body>::exp(theta);
        let measured = (base.inverse() * Rotation::exp(theta + delta)).log();
        prop_assert!((measured - right_jacobian(theta) * delta).norm() <= 1e-9);
        let back = base.plus(delta).log() - theta;
        prop_assert!((back - right_jacobian_inverse(theta) * delta).norm() <= 1e-9);
    }

    #[test]
    fn plus_and_minus_invert_each_other(r in attitude(), s in attitude(), tau in rotation_vector(PI - 1e-6)) {
        prop_assert!(r.plus(s.minus(r)).angle_to(s) <= 1e-14);
        prop_assert!((r.plus(tau).minus(r) - tau).norm() <= 1e-12);
    }

    #[test]
    fn slerp_moves_at_constant_rate_along_the_short_arc(
        r0 in attitude(),
        r1 in attitude(),
        t in 0.0..=1.0_f64,
    ) {
        let total = r0.angle_to(r1);
        prop_assert!(total <= PI + 1e-15);
        prop_assert!(r0.slerp(r1, 0.0).angle_to(r0) <= 1e-15 * 8.0);
        prop_assert!(r0.slerp(r1, 1.0).angle_to(r1) <= 1e-14);
        prop_assert!((r0.angle_to(r0.slerp(r1, t)) - t * total).abs() <= 1e-13);
    }

    #[test]
    fn euler_angles_round_trip_away_from_gimbal_lock(
        yaw in -PI + 1e-9..PI - 1e-9,
        pitch in -FRAC_PI_2 + 1e-3..FRAC_PI_2 - 1e-3,
        roll in -PI + 1e-9..PI - 1e-9,
    ) {
        let angles = EulerAngles { yaw_rad: yaw, pitch_rad: pitch, roll_rad: roll };
        let back = Rotation::<World, Body>::from_euler(angles).to_euler();
        prop_assert!((back.yaw_rad - yaw).abs() <= 1e-11, "yaw {} vs {yaw}", back.yaw_rad);
        prop_assert!((back.pitch_rad - pitch).abs() <= 1e-11, "pitch {} vs {pitch}", back.pitch_rad);
        prop_assert!((back.roll_rad - roll).abs() <= 1e-11, "roll {} vs {roll}", back.roll_rad);
    }

    #[test]
    fn euler_rotations_round_trip_everywhere_including_gimbal_lock(
        yaw in -PI..PI,
        roll in -PI..PI,
        sign in prop::bool::ANY,
        closeness in prop::option::of(0.0..15.0_f64),
    ) {
        let offset = closeness.map_or(0.0, |k| 10f64.powf(-k));
        let pitch = if sign { FRAC_PI_2 - offset } else { offset - FRAC_PI_2 };
        let angles = EulerAngles { yaw_rad: yaw, pitch_rad: pitch, roll_rad: roll };
        let original = Rotation::<World, Body>::from_euler(angles);
        let rebuilt = Rotation::<World, Body>::from_euler(original.to_euler());
        prop_assert!(original.angle_to(rebuilt) <= 1e-14, "pitch {pitch}");
    }

    #[test]
    fn euler_matches_the_matrix_product(yaw in -PI..PI, pitch in -FRAC_PI_2..FRAC_PI_2, roll in -PI..PI) {
        let (sy, cy) = yaw.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sr, cr) = roll.sin_cos();
        let rz = Mat3::<Body>::from_rows([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]);
        let ry = Mat3::<Body>::from_rows([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]);
        let rx = Mat3::<Body>::from_rows([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]);
        let angles = EulerAngles { yaw_rad: yaw, pitch_rad: pitch, roll_rad: roll };
        let q: Rotation<Body, Body> = Rotation::from_euler(angles);
        prop_assert!((rotation_matrix(q) - rz * ry * rx).max_abs() <= 1e-15 * 8.0);
    }

    #[test]
    fn matrix_conversion_round_trips(q in attitude()) {
        let back = Rotation::<World, Body>::from_matrix(q.matrix());
        prop_assert!(back.is_some_and(|b| b.angle_to(q) <= 1e-15 * 8.0));
    }

    #[test]
    fn non_rotations_are_rejected(q in attitude()) {
        let m = q.matrix();
        let scaled = m.map(|row| row.map(|v| 2.0 * v));
        let reflected = m.map(|row| row.map(|v| -v));
        prop_assert!(Rotation::<World, Body>::from_matrix(scaled).is_none());
        prop_assert!(Rotation::<World, Body>::from_matrix(reflected).is_none());
    }
}
