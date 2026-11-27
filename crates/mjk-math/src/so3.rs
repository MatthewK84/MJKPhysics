//! SO(3) exponential and logarithm maps, Jacobians, ⊕/⊖, interpolation and matrix conversion.
//!
//! Equations are on the book's *SO(3) maps* page, prefix `SO3`. A rotation vector is
//! `θ = θ·u`: unit axis `u`, angle `θ` in radians, right-handed. Increments are applied on the
//! right, in the local frame (CONV-10). Transcendental functions go through `libm` so results
//! are bit-identical on every platform.

use std::f64::consts::TAU;

use libm::{atan2, cos, sin};

use crate::frame::Frame;
use crate::mat3::Mat3;
use crate::quat::{Quat, UnitQuat};
use crate::rotation::Rotation;
use crate::vec3::Vec3;

/// Below this angle (rad), `sin(θ/2)/θ` and the Jacobian coefficients use Taylor series.
/// The truncation error there is below 1e-19, far under rounding.
pub const SMALL_ANGLE_RAD: f64 = 1e-3;

/// Below this vector-part norm, the logarithm uses a Taylor series for `θ/‖q_v‖`.
pub const SMALL_VECTOR_NORM: f64 = 1e-6;

/// Below this angle (rad) the Jacobian coefficients `(θ−sin θ)/θ³` and
/// `1/θ² − cot(θ/2)/(2θ)` use Taylor series: their closed forms lose about `12·ε/θ²` of relative
/// accuracy to cancellation, which at the `θ ≈ 10⁻³` of one integrator step is 9 digits.
pub const JACOBIAN_SERIES_BELOW_RAD: f64 = 1.0;

/// Taylor coefficients of `(θ − sin θ)/θ³` in powers of `θ²`: `(−1)ᵏ/(2k+3)!`, Eq. (SO3-3).
const B_SERIES: [f64; 8] = [
    0.166_666_666_666_666_66,
    -0.008_333_333_333_333_333,
    0.000_198_412_698_412_698_4,
    -2.755_731_922_398_589_3e-6,
    2.505_210_838_544_172e-8,
    -1.605_904_383_682_161_3e-10,
    7.647_163_731_819_816e-13,
    -2.811_457_254_345_520_6e-15,
];

/// Taylor coefficients of `1/θ² − cot(θ/2)/(2θ)` in powers of `θ²`:
/// `(−1)ⁿ⁺¹·B₂ₙ/(2n)!` for `n = 1..=11`, with `B₂ₙ` the Bernoulli numbers, Eq. (SO3-3).
const C_SERIES: [f64; 11] = [
    0.083_333_333_333_333_33,
    0.001_388_888_888_888_889,
    3.306_878_306_878_307e-5,
    8.267_195_767_195_768e-7,
    2.087_675_698_786_81e-8,
    5.284_190_138_687_493e-10,
    1.338_253_653_068_467_9e-11,
    3.389_680_296_322_582_7e-13,
    8.586_062_056_277_845e-15,
    2.174_868_698_558_062e-16,
    5.509_002_828_360_229_5e-18,
];

/// Evaluates `Σ cₖ·xᵏ` by Horner's rule.
fn horner(coefficients: &[f64], x: f64) -> f64 {
    coefficients
        .iter()
        .rev()
        .fold(0.0, |acc, c| libm::fma(acc, x, *c))
}

/// `sin(θ/2)/θ`, with its Taylor series near zero, Eq. (SO3-3).
fn half_sine_over_angle(theta: f64) -> f64 {
    if theta < SMALL_ANGLE_RAD {
        let t2 = theta * theta;
        return 0.5 - t2 / 48.0 + t2 * t2 / 3840.0;
    }
    sin(0.5 * theta) / theta
}

/// Coefficients `(1−cos θ)/θ²` and `(θ−sin θ)/θ³` of the right Jacobian, Eq. (SO3-4).
///
/// The first is computed as `2·(sin(θ/2)/θ)²`, which has no cancellation.
fn jacobian_coefficients(theta: f64) -> (f64, f64) {
    let k = half_sine_over_angle(theta);
    let a = 2.0 * k * k;
    if theta < JACOBIAN_SERIES_BELOW_RAD {
        return (a, horner(&B_SERIES, theta * theta));
    }
    (a, (theta - sin(theta)) / (theta * theta * theta))
}

/// Coefficient `1/θ² − cot(θ/2)/(2θ)` of the inverse right Jacobian, Eq. (SO3-5).
fn inverse_jacobian_coefficient(theta: f64) -> f64 {
    if theta < JACOBIAN_SERIES_BELOW_RAD {
        return horner(&C_SERIES, theta * theta);
    }
    let half = 0.5 * theta;
    1.0 / (theta * theta) - cos(half) / (2.0 * theta * sin(half))
}

impl UnitQuat {
    /// The exponential map: rotation vector to unit quaternion, Eq. (SO3-1).
    #[must_use]
    pub fn exp([vx, vy, vz]: [f64; 3]) -> Self {
        let theta = (vx * vx + vy * vy + vz * vz).sqrt();
        let k = half_sine_over_angle(theta);
        Self::from_unit(Quat::new(cos(0.5 * theta), k * vx, k * vy, k * vz))
    }

    /// The logarithm map: the rotation vector with angle in `[0, π]`, Eq. (SO3-2).
    ///
    /// `q` and `−q` give the same result: the shortest rotation is always chosen.
    #[must_use]
    pub fn log(self) -> [f64; 3] {
        let q = self.quat();
        let q = if q.w < 0.0 { -q } else { q };
        let norm = (q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
        let ratio = if norm < SMALL_VECTOR_NORM {
            // 2·atan(n/w)/n = (2/w)·(1 − n²/3 + O(n⁴)), using w² = 1 − n²; for n < 1e-6 the
            // O(n⁴) terms are below 1e-24, far under rounding. Eq. (SO3-3).
            2.0 / q.w * (1.0 - norm * norm / 3.0)
        } else {
            2.0 * atan2(norm, q.w) / norm
        };
        [ratio * q.x, ratio * q.y, ratio * q.z]
    }

    /// The angle of the rotation between `self` and `other`, in `[0, π]`.
    #[must_use]
    pub fn angle_to(self, other: Self) -> f64 {
        (other.inverse() * self).angle()
    }

    /// The rotation angle in `[0, π]`.
    #[must_use]
    pub fn angle(self) -> f64 {
        let q = self.quat();
        2.0 * atan2((q.x * q.x + q.y * q.y + q.z * q.z).sqrt(), q.w.abs())
    }
}

/// Right Jacobian of SO(3), Eq. (SO3-4). Valid for `‖θ‖ < 2π`.
#[must_use]
pub fn right_jacobian<F: Frame>(theta: Vec3<F>) -> Mat3<F> {
    debug_assert!(
        theta.norm() < TAU,
        "J_r is defined for |θ| < 2π; |θ| = {}",
        theta.norm()
    );
    let (a, b) = jacobian_coefficients(theta.norm());
    let skew = Mat3::skew(theta);
    Mat3::identity() - skew * a + skew * skew * b
}

/// Inverse of the right Jacobian, Eq. (SO3-5). Valid for `‖θ‖ < 2π`.
#[must_use]
pub fn right_jacobian_inverse<F: Frame>(theta: Vec3<F>) -> Mat3<F> {
    debug_assert!(
        theta.norm() < TAU,
        "J_r⁻¹ is defined for |θ| < 2π; |θ| = {}",
        theta.norm()
    );
    let c = inverse_jacobian_coefficient(theta.norm());
    let skew = Mat3::skew(theta);
    Mat3::identity() + skew * 0.5 + skew * skew * c
}

/// Left Jacobian of SO(3): `J_l(θ) = J_r(−θ)`, Eq. (SO3-6).
#[must_use]
pub fn left_jacobian<F: Frame>(theta: Vec3<F>) -> Mat3<F> {
    right_jacobian(-theta)
}

/// Inverse of the left Jacobian: `J_l⁻¹(θ) = J_r⁻¹(−θ)`, Eq. (SO3-6).
#[must_use]
pub fn left_jacobian_inverse<F: Frame>(theta: Vec3<F>) -> Mat3<F> {
    right_jacobian_inverse(-theta)
}

impl<F: Frame> Rotation<F, F> {
    /// The rotation `Exp(θ)` about `θ`'s axis by `‖θ‖`, Eq. (SO3-1).
    #[must_use]
    pub fn exp(theta: Vec3<F>) -> Self {
        Self::from_unit_quat(UnitQuat::exp(theta.to_array()))
    }

    /// The rotation vector `Log(R)` with angle in `[0, π]`, Eq. (SO3-2).
    #[must_use]
    pub fn log(self) -> Vec3<F> {
        Vec3::from_array(self.unit_quat().log())
    }
}

impl<To: Frame, From: Frame> Rotation<To, From> {
    /// `R ⊕ τ = R·Exp(τ)`: perturbs by a rotation vector in the `From` frame, Eq. (SO3-8).
    #[must_use]
    pub fn plus(self, tau: Vec3<From>) -> Self {
        self * Rotation::<From, From>::exp(tau)
    }

    /// `S ⊖ R = Log(R⁻¹·S)`, so that `R ⊕ (S ⊖ R) = S`, Eq. (SO3-8).
    #[must_use]
    pub fn minus(self, other: Self) -> Vec3<From> {
        (other.inverse() * self).log()
    }

    /// Spherical linear interpolation, `R₀ ⊕ t·(R₁ ⊖ R₀)`, along the shortest arc,
    /// Eq. (SO3-9). `t = 0` gives `self`, `t = 1` gives `other`.
    #[must_use]
    pub fn slerp(self, other: Self, t: f64) -> Self {
        debug_assert!(t.is_finite(), "slerp needs a finite t; got {t}");
        self.plus(other.minus(self) * t)
    }

    /// The rotation angle in `[0, π]`.
    #[must_use]
    pub fn angle(self) -> f64 {
        self.unit_quat().angle()
    }

    /// The angle of the rotation between `self` and `other`, in `[0, π]`.
    #[must_use]
    pub fn angle_to(self, other: Self) -> f64 {
        (other.inverse() * self).angle()
    }

    /// The rotation with this row-major matrix, by Shepperd's method, Eq. (SO3-10).
    ///
    /// Returns `None` unless the matrix is a proper rotation: orthonormal to `1e-9` and with
    /// determinant `+1`.
    #[must_use]
    pub fn from_matrix(m: [[f64; 3]; 3]) -> Option<Self> {
        let check = Mat3::<To>::from_rows(m);
        let orthonormal = (check.transpose() * check - Mat3::identity()).max_abs() <= 1e-9;
        if !(orthonormal && check.determinant() > 0.0) {
            return None;
        }
        UnitQuat::new(shepperd(m)).map(Self::from_unit_quat)
    }
}

/// Shepperd's method: take the square root of the largest of `4w², 4x², 4y², 4z²` and recover
/// the other components from off-diagonal sums and differences, Eq. (SO3-10).
fn shepperd(m: [[f64; 3]; 3]) -> Quat {
    let [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]] = m;
    let trace = r11 + r22 + r33;
    let candidates = [
        1.0 + trace,
        1.0 + 2.0 * r11 - trace,
        1.0 + 2.0 * r22 - trace,
        1.0 + 2.0 * r33 - trace,
    ];
    let (largest, value) =
        candidates
            .into_iter()
            .enumerate()
            .fold((0, f64::NEG_INFINITY), |best, (index, v)| {
                if v > best.1 { (index, v) } else { best }
            });
    // The four candidates sum to 4, so the largest is at least 1 for a rotation matrix.
    debug_assert!(
        value >= 0.99,
        "Shepperd's largest candidate is {value}, not a rotation"
    );
    let s = value.sqrt();
    let d = 2.0 * s;
    match largest {
        0 => Quat::new(0.5 * s, (r32 - r23) / d, (r13 - r31) / d, (r21 - r12) / d),
        1 => Quat::new((r32 - r23) / d, 0.5 * s, (r12 + r21) / d, (r13 + r31) / d),
        2 => Quat::new((r13 - r31) / d, (r12 + r21) / d, 0.5 * s, (r23 + r32) / d),
        _ => Quat::new((r21 - r12) / d, (r13 + r31) / d, (r23 + r32) / d, 0.5 * s),
    }
}

#[cfg(test)]
mod tests {
    use super::{inverse_jacobian_coefficient, jacobian_coefficients};

    /// 40-digit references computed with exact rational series and decimal arithmetic.
    const REFERENCES: [(f64, f64, f64, f64); 8] = [
        (
            1e-6,
            0.499_999_999_999_958_3,
            0.166_666_666_666_658_33,
            0.083_333_333_333_334_72,
        ),
        (
            1e-3,
            0.499_999_958_333_334_7,
            0.166_666_658_333_333_52,
            0.083_333_334_722_222_25,
        ),
        (
            0.1,
            0.499_583_472_197_423_4,
            0.166_583_353_171_847_7,
            0.083_347_225_529_927_46,
        ),
        (
            0.5,
            0.489_669_752_438_509_13,
            0.164_595_691_166_375_98,
            0.083_682_635_354_059_9,
        ),
        (
            0.999,
            0.459_775_584_832_508_13,
            0.158_544_897_359_309_35,
            0.084_753_225_558_244_42,
        ),
        (
            1.001,
            0.459_619_736_035_745_9,
            0.158_513_118_657_900_27,
            0.084_759_055_929_927,
        ),
        (
            2.0,
            0.354_036_709_136_785_6,
            0.136_337_821_646_789_8,
            0.089_476_846_016_417_32,
        ),
        (
            3.0,
            0.221_110_277_400_049_5,
            0.105_884_444_145_930_84,
            0.099_291_970_394_002_37,
        ),
    ];

    fn relative_error(value: f64, reference: f64) -> f64 {
        ((value - reference) / reference).abs()
    }

    #[test]
    fn jacobian_coefficients_match_40_digit_references() {
        for (theta, a_ref, b_ref, c_ref) in REFERENCES {
            let (a, b) = jacobian_coefficients(theta);
            let c = inverse_jacobian_coefficient(theta);
            assert!(relative_error(a, a_ref) < 1e-15, "a({theta}) = {a}");
            assert!(relative_error(b, b_ref) < 1e-15, "b({theta}) = {b}");
            assert!(relative_error(c, c_ref) < 1e-15, "c({theta}) = {c}");
        }
    }

    #[test]
    fn inverse_jacobian_coefficient_is_finite_at_pi() {
        let pi = std::f64::consts::PI;
        assert!((inverse_jacobian_coefficient(pi) - 1.0 / (pi * pi)).abs() < 1e-15);
    }
}
