//! Property tests for the frame-tagged math types (#8).
//!
//! Each property holds for every input; proptest searches for counterexamples and shrinks any
//! failure to a minimal case. Tolerances are a few hundred ulps scaled to the inputs.

use mjk_math::{Body, Mat3, Quat, Rotation, Stability, UnitQuat, Vec3, World};
use proptest::prelude::*;

const CASES: u32 = 512;
/// Relative tolerance for results that go through a handful of rounded operations.
const TOL: f64 = 1e-12;

fn component() -> impl Strategy<Value = f64> {
    -100.0..100.0_f64
}

fn vector<F: mjk_math::Frame>() -> impl Strategy<Value = Vec3<F>> {
    (component(), component(), component()).prop_map(|(x, y, z)| Vec3::new(x, y, z))
}

fn unit_quat() -> impl Strategy<Value = UnitQuat> {
    (-1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64)
        .prop_filter("away from zero", |(w, x, y, z)| {
            w * w + x * x + y * y + z * z > 1e-2
        })
        .prop_filter_map("normalizable", |(w, x, y, z)| {
            UnitQuat::new(Quat::new(w, x, y, z))
        })
}

fn matrix<F: mjk_math::Frame>() -> impl Strategy<Value = Mat3<F>> {
    prop::array::uniform3(prop::array::uniform3(-10.0..10.0_f64)).prop_map(Mat3::from_rows)
}

fn symmetric<F: mjk_math::Frame>() -> impl Strategy<Value = Mat3<F>> {
    prop::array::uniform6(-10.0..10.0_f64).prop_map(|[xx, yy, zz, xy, xz, yz]| {
        Mat3::from_rows([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    })
}

fn close<F: mjk_math::Frame>(a: Vec3<F>, b: Vec3<F>, scale: f64) -> bool {
    (a - b).norm() <= TOL * scale.max(1.0)
}

fn matrices_close(a: [[f64; 3]; 3], b: [[f64; 3]; 3], tol: f64) -> bool {
    a.iter()
        .flatten()
        .zip(b.iter().flatten())
        .all(|(x, y)| (x - y).abs() <= tol)
}

fn mat_mul(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    (Mat3::<Body>::from_rows(a) * Mat3::from_rows(b)).rows()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(CASES))]

    #[test]
    fn rotation_preserves_norm(q in unit_quat(), v in vector::<Body>()) {
        let q_wb: Rotation<World, Body> = Rotation::from_unit_quat(q);
        let rotated = q_wb.rotate(v);
        prop_assert!((rotated.norm() - v.norm()).abs() <= TOL * v.norm().max(1.0));
    }

    #[test]
    fn quaternion_times_inverse_is_identity(q in unit_quat()) {
        let product = (q * q.inverse()).quat() - Quat::identity();
        prop_assert!(product.norm() <= 1e-15);
    }

    #[test]
    fn quaternion_composition_matches_matrix_product(p in unit_quat(), q in unit_quat()) {
        let composed = (p * q).matrix();
        prop_assert!(matrices_close(composed, mat_mul(p.matrix(), q.matrix()), 1e-14));
    }

    #[test]
    fn quaternion_rotation_matches_its_matrix(q in unit_quat(), v in vector::<Body>()) {
        let by_matrix = Mat3::<Body>::from_rows(q.matrix()) * v;
        let by_quaternion = Vec3::<Body>::from_array(q.rotate_array(v.to_array()));
        prop_assert!(close(by_matrix, by_quaternion, v.norm()));
    }

    #[test]
    fn typed_composition_applies_right_operand_first(
        a in unit_quat(),
        b in unit_quat(),
        v in vector::<Stability>(),
    ) {
        let q_wb: Rotation<World, Body> = Rotation::from_unit_quat(a);
        let q_bs: Rotation<Body, Stability> = Rotation::from_unit_quat(b);
        prop_assert!(close((q_wb * q_bs).rotate(v), q_wb.rotate(q_bs.rotate(v)), v.norm()));
    }

    #[test]
    fn inverse_rotation_round_trips(q in unit_quat(), v in vector::<Body>()) {
        let q_wb: Rotation<World, Body> = Rotation::from_unit_quat(q);
        prop_assert!(close(q_wb.inverse().rotate(q_wb.rotate(v)), v, v.norm()));
    }

    #[test]
    fn transformed_tensor_acts_consistently(
        q in unit_quat(),
        m in matrix::<Body>(),
        v in vector::<Body>(),
    ) {
        let q_wb: Rotation<World, Body> = Rotation::from_unit_quat(q);
        let m_w = q_wb.transform(m);
        let scale = m.max_abs() * v.norm();
        prop_assert!(close(m_w * q_wb.rotate(v), q_wb.rotate(m * v), scale));
    }

    #[test]
    fn cross_product_is_antisymmetric_and_orthogonal(
        a in vector::<World>(),
        b in vector::<World>(),
    ) {
        prop_assert_eq!(a.cross(b), -b.cross(a));
        let c = a.cross(b);
        let scale = a.norm() * a.norm() * b.norm();
        prop_assert!(c.dot(a).abs() <= TOL * scale.max(1.0));
        prop_assert!(c.dot(b).abs() <= TOL * (b.norm() * scale / a.norm().max(1.0)).max(1.0));
    }

    #[test]
    fn skew_and_outer_products(a in vector::<Body>(), b in vector::<Body>(), c in vector::<Body>()) {
        prop_assert!(close(Mat3::skew(a) * b, a.cross(b), a.norm() * b.norm()));
        let outer = Mat3::outer(a, b) * c;
        prop_assert!(close(outer, a * b.dot(c), a.norm() * b.norm() * c.norm()));
    }

    #[test]
    fn transpose_is_an_involution_and_determinant_is_multiplicative(
        a in matrix::<Body>(),
        b in matrix::<Body>(),
    ) {
        prop_assert_eq!(a.transpose().transpose(), a);
        let scale = libm::pow(a.max_abs() * b.max_abs(), 3.0).max(1.0);
        let det_product = (a * b).determinant() - a.determinant() * b.determinant();
        prop_assert!(det_product.abs() <= TOL * scale);
    }

    #[test]
    fn inverse_of_a_well_conditioned_matrix(m in matrix::<Body>()) {
        let scale = m.max_abs();
        prop_assume!(m.determinant().abs() > 1e-2 * scale * scale * scale);
        let inverse = m.inverse();
        prop_assert!(inverse.is_some());
        if let Some(inverse) = inverse {
            let residual = (m * inverse - Mat3::identity()).max_abs();
            prop_assert!(residual <= 1e-10, "residual {}", residual);
        }
    }

    #[test]
    fn symmetric_eigen_decomposition(m in symmetric::<Body>()) {
        let eigen = m.symmetric_eigen();
        prop_assert!(eigen.is_ok());
        if let Ok(eigen) = eigen {
            let scale = m.max_abs().max(1.0);
            let [l0, l1, l2] = eigen.values;
            prop_assert!(l0 <= l1 && l1 <= l2);
            let [e0, e1, e2] = eigen.vectors;
            for (value, vector) in eigen.values.into_iter().zip(eigen.vectors) {
                prop_assert!(close(m * vector, vector * value, scale));
                prop_assert!((vector.norm() - 1.0).abs() <= TOL);
            }
            prop_assert!(e0.dot(e1).abs() <= TOL && e0.dot(e2).abs() <= TOL && e1.dot(e2).abs() <= TOL);
            prop_assert!((e0.cross(e1).dot(e2) - 1.0).abs() <= TOL);
            let rebuilt = Mat3::outer(e0, e0) * l0 + Mat3::outer(e1, e1) * l1 + Mat3::outer(e2, e2) * l2;
            prop_assert!((rebuilt - m).max_abs() <= TOL * scale);
        }
    }
}
