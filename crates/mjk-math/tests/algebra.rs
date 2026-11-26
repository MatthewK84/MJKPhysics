//! Exact results of the elementary vector, matrix and quaternion operations.
//!
//! Inputs are chosen so every result is exactly representable, so the assertions compare bit
//! patterns rather than tolerances.

use mjk_math::{Body, Mat3, Quat, Vec3};

fn v(x: f64, y: f64, z: f64) -> Vec3<Body> {
    Vec3::new(x, y, z)
}

fn bits<const N: usize>(values: [f64; N]) -> [u64; N] {
    values.map(f64::to_bits)
}

fn quat_bits(q: Quat) -> [u64; 4] {
    bits([q.w, q.x, q.y, q.z])
}

#[test]
fn vector_subtraction_and_compound_assignment() {
    let a = v(1.5, -2.0, 4.0);
    let b = v(0.5, 3.0, -1.0);
    assert_eq!(bits((a - b).to_array()), bits([1.0, -5.0, 5.0]));
    let mut c = a;
    c += b;
    assert_eq!(bits(c.to_array()), bits([2.0, 1.0, 3.0]));
    c -= b;
    assert_eq!(bits(c.to_array()), bits(a.to_array()));
}

#[test]
fn vector_is_finite_checks_every_component() {
    assert!(v(1.0, -2.0, 3.0).is_finite());
    for bad in [f64::NAN, f64::INFINITY] {
        assert!(!v(bad, 0.0, 0.0).is_finite());
        assert!(!v(0.0, bad, 0.0).is_finite());
        assert!(!v(0.0, 0.0, bad).is_finite());
    }
}

fn sample_matrix() -> Mat3<Body> {
    Mat3::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]])
}

#[test]
fn matrix_diagonal_trace_and_negation() {
    let m = sample_matrix();
    assert_eq!(bits(m.diagonal()), bits([1.0, 5.0, 10.0]));
    assert_eq!(m.trace().to_bits(), 16.0_f64.to_bits());
    let negated = (-m).rows().map(bits);
    let expected = [[-1.0, -2.0, -3.0], [-4.0, -5.0, -6.0], [-7.0, -8.0, -10.0]].map(bits);
    assert_eq!(negated, expected);
}

#[test]
fn matrix_is_finite_checks_every_entry() {
    assert!(sample_matrix().is_finite());
    for index in 0..9 {
        let mut rows = sample_matrix().rows();
        rows[index / 3][index % 3] = f64::NAN;
        assert!(!Mat3::<Body>::from_rows(rows).is_finite(), "entry {index}");
    }
}

#[test]
fn matrix_inverse_rejects_singular_and_non_finite_input() {
    let singular = Mat3::<Body>::from_rows([[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 1.0, 1.0]]);
    assert!(singular.inverse().is_none());
    assert!(Mat3::<Body>::zero().inverse().is_none());
    let mut rows = sample_matrix().rows();
    rows[1][1] = f64::INFINITY;
    assert!(Mat3::<Body>::from_rows(rows).inverse().is_none());
    let inverse = Mat3::<Body>::from_diagonal([2.0, 4.0, 0.5]).inverse();
    let diagonal = inverse.map(|m| bits(m.diagonal()));
    assert_eq!(diagonal, Some(bits([0.5, 0.25, 2.0])));
}

#[test]
fn matrix_debug_names_its_frame() {
    let text = format!("{:?}", Mat3::<Body>::from_diagonal([1.0, 2.0, 3.0]));
    assert_eq!(
        text,
        "Mat3<Body>[[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]]"
    );
}

#[test]
fn quaternion_vector_part_addition_and_subtraction() {
    let p = Quat::new(1.0, 2.0, 3.0, 4.0);
    let q = Quat::new(0.5, -1.0, 2.0, -3.0);
    assert_eq!(bits(p.vector()), bits([2.0, 3.0, 4.0]));
    assert_eq!(quat_bits(p + q), bits([1.5, 1.0, 5.0, 1.0]));
    assert_eq!(quat_bits(p - q), bits([0.5, 3.0, 1.0, 7.0]));
}

#[test]
fn quaternion_is_finite_checks_every_component() {
    assert!(Quat::new(1.0, 2.0, 3.0, 4.0).is_finite());
    for bad in [f64::NAN, f64::NEG_INFINITY] {
        assert!(!Quat::new(bad, 0.0, 0.0, 0.0).is_finite());
        assert!(!Quat::new(0.0, bad, 0.0, 0.0).is_finite());
        assert!(!Quat::new(0.0, 0.0, bad, 0.0).is_finite());
        assert!(!Quat::new(0.0, 0.0, 0.0, bad).is_finite());
    }
}
