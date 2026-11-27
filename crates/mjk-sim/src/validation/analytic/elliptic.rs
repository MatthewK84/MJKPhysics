//! The complete elliptic integral of the first kind, for the intermediate-axis flip period.

use core::f64::consts::PI;

/// Iterations of the arithmetic–geometric mean: it converges quadratically, so six suffice for
/// double precision at any k² ≤ 1 − 10⁻¹⁰⁰, and this bound is never reached.
const MAX_ITERATIONS: u32 = 32;

/// `K(k) = π / (2·AGM(1, √(1 − k²)))`, Eq. (VAL-8), or `None` unless `0 ≤ k² < 1`.
#[must_use]
pub fn complete_elliptic_k(k_squared: f64) -> Option<f64> {
    if !(0.0..1.0).contains(&k_squared) {
        return None;
    }
    let mut a = 1.0_f64;
    let mut b = (1.0 - k_squared).sqrt();
    for _ in 0..MAX_ITERATIONS {
        if (a - b).abs() <= f64::EPSILON * a {
            break;
        }
        let mean = 0.5 * (a + b);
        b = (a * b).sqrt();
        a = mean;
    }
    Some(PI / (2.0 * a))
}
