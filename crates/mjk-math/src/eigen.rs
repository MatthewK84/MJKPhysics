//! Symmetric eigen-decomposition by the cyclic Jacobi method.
//!
//! Used for principal axes of inertia tensors. Jacobi rotations are applied plane by plane until
//! the off-diagonal mass is negligible relative to the matrix scale. Convergence is quadratic,
//! so a 3×3 matrix needs a handful of sweeps; the bound below is a safety net that reports
//! [`EngineError::Convergence`] instead of looping.
//!
//! Reference: G. H. Golub, C. F. Van Loan, *Matrix Computations*, 4th ed., Johns Hopkins
//! University Press, 2013, §8.5 (Jacobi methods).

use mjk_core::EngineError;

use crate::frame::Frame;
use crate::mat3::Mat3;
use crate::vec3::Vec3;

/// Maximum number of full sweeps over the three planes.
pub const MAX_JACOBI_SWEEPS: u32 = 32;

/// Off-diagonal entries below this fraction of the largest entry count as zero.
pub const JACOBI_TOLERANCE: f64 = 1e-15;

/// Eigenvalues and eigenvectors of a symmetric matrix in frame `F`.
#[derive(Clone, Copy, Debug)]
pub struct SymmetricEigen<F: Frame> {
    /// Eigenvalues in ascending order.
    pub values: [f64; 3],
    /// Unit eigenvectors, matching `values`, forming a right-handed orthonormal basis.
    pub vectors: [Vec3<F>; 3],
}

/// Symmetric matrix entries `[xx, yy, zz, xy, xz, yz]` while rotating.
#[derive(Clone, Copy)]
struct Sym {
    diag: [f64; 3],
    xy: f64,
    xz: f64,
    yz: f64,
}

/// A rotation plane `(p, q)` and the remaining axis `r`, as array positions 0..3.
#[derive(Clone, Copy)]
enum Plane {
    XY,
    XZ,
    YZ,
}

const PLANES: [Plane; 3] = [Plane::XY, Plane::XZ, Plane::YZ];

impl Sym {
    fn off_diagonal_norm(self) -> f64 {
        (self.xy * self.xy + self.xz * self.xz + self.yz * self.yz).sqrt()
    }

    /// True when the off-diagonal entries are negligible against `threshold`.
    fn is_diagonal(self, threshold: f64) -> bool {
        self.off_diagonal_norm() <= threshold
    }

    /// Entries `(a_pp, a_qq, a_pq, a_pr, a_qr)` for a plane.
    fn plane(self, plane: Plane) -> (f64, f64, f64, f64, f64) {
        let [xx, yy, zz] = self.diag;
        match plane {
            Plane::XY => (xx, yy, self.xy, self.xz, self.yz),
            Plane::XZ => (xx, zz, self.xz, self.xy, self.yz),
            Plane::YZ => (yy, zz, self.yz, self.xy, self.xz),
        }
    }

    fn with_plane(self, plane: Plane, (pp, qq, pq, pr, qr): (f64, f64, f64, f64, f64)) -> Self {
        let [xx, yy, zz] = self.diag;
        match plane {
            Plane::XY => Self {
                diag: [pp, qq, zz],
                xy: pq,
                xz: pr,
                yz: qr,
            },
            Plane::XZ => Self {
                diag: [pp, yy, qq],
                xz: pq,
                xy: pr,
                yz: qr,
            },
            Plane::YZ => Self {
                diag: [xx, pp, qq],
                yz: pq,
                xy: pr,
                xz: qr,
            },
        }
    }
}

/// Rotates the columns `p` and `q` of the eigenvector matrix (stored as rows of columns).
fn rotate_vectors(columns: [[f64; 3]; 3], plane: Plane, cos: f64, sin: f64) -> [[f64; 3]; 3] {
    let [c0, c1, c2] = columns;
    let mix = |p: [f64; 3], q: [f64; 3]| {
        let [p0, p1, p2] = p;
        let [q0, q1, q2] = q;
        (
            [
                cos * p0 - sin * q0,
                cos * p1 - sin * q1,
                cos * p2 - sin * q2,
            ],
            [
                sin * p0 + cos * q0,
                sin * p1 + cos * q1,
                sin * p2 + cos * q2,
            ],
        )
    };
    match plane {
        Plane::XY => {
            let (p, q) = mix(c0, c1);
            [p, q, c2]
        }
        Plane::XZ => {
            let (p, q) = mix(c0, c2);
            [p, c1, q]
        }
        Plane::YZ => {
            let (p, q) = mix(c1, c2);
            [c0, p, q]
        }
    }
}

/// One Jacobi rotation that zeroes `a_pq`.
fn jacobi_step(sym: Sym, columns: [[f64; 3]; 3], plane: Plane) -> (Sym, [[f64; 3]; 3]) {
    let (pp, qq, pq, pr, qr) = sym.plane(plane);
    if pq.abs() <= 0.0 {
        return (sym, columns);
    }
    let theta = (qq - pp) / (2.0 * pq);
    let tangent = theta.signum() / (theta.abs() + libm::fma(theta, theta, 1.0).sqrt());
    let cos = 1.0 / libm::fma(tangent, tangent, 1.0).sqrt();
    let sin = tangent * cos;
    let rotated = (
        pp - tangent * pq,
        qq + tangent * pq,
        0.0,
        cos * pr - sin * qr,
        sin * pr + cos * qr,
    );
    (
        sym.with_plane(plane, rotated),
        rotate_vectors(columns, plane, cos, sin),
    )
}

impl<F: Frame> Mat3<F> {
    /// Eigen-decomposition of a symmetric matrix by cyclic Jacobi rotations.
    ///
    /// Only the upper triangle is read; the matrix is assumed symmetric.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::Numerical`] for non-finite entries, and
    /// [`EngineError::Convergence`] if the off-diagonal entries are not negligible after
    /// [`MAX_JACOBI_SWEEPS`] sweeps.
    pub fn symmetric_eigen(self) -> Result<SymmetricEigen<F>, EngineError> {
        if let Some(&value) = self
            .rows()
            .iter()
            .flatten()
            .find(|value| !value.is_finite())
        {
            return Err(EngineError::Numerical {
                sim_time_s: None,
                quantity: "symmetric eigen-decomposition input entry",
                value,
            });
        }
        debug_assert!(
            (self - self.transpose()).max_abs() <= 1e-9 * self.max_abs().max(1.0),
            "symmetric_eigen reads only the upper triangle; the input must be symmetric"
        );
        let [[xx, xy, xz], [_, yy, yz], [_, _, zz]] = self.rows();
        let mut sym = Sym {
            diag: [xx, yy, zz],
            xy,
            xz,
            yz,
        };
        let mut columns = Mat3::<F>::identity().rows();
        let threshold = JACOBI_TOLERANCE * self.max_abs();
        for _ in 0..MAX_JACOBI_SWEEPS {
            if sym.is_diagonal(threshold) {
                return Ok(sorted(sym.diag, columns));
            }
            for plane in PLANES {
                (sym, columns) = jacobi_step(sym, columns, plane);
            }
        }
        if sym.is_diagonal(threshold) {
            return Ok(sorted(sym.diag, columns));
        }
        Err(EngineError::Convergence {
            solver: "Jacobi symmetric eigen-decomposition",
            iterations: MAX_JACOBI_SWEEPS,
            residual: sym.off_diagonal_norm(),
        })
    }
}

/// Sorts eigenpairs ascending and makes the basis right-handed.
fn sorted<F: Frame>(values: [f64; 3], columns: [[f64; 3]; 3]) -> SymmetricEigen<F> {
    let [d0, d1, d2] = values;
    let [c0, c1, c2] = columns.map(Vec3::<F>::from_array);
    let mut pairs = [(d0, c0), (d1, c1), (d2, c2)];
    pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
    let [(v0, e0), (v1, e1), (v2, e2)] = pairs;
    let e2 = if e0.cross(e1).dot(e2) < 0.0 { -e2 } else { e2 };
    SymmetricEigen {
        values: [v0, v1, v2],
        vectors: [e0, e1, e2],
    }
}

#[cfg(test)]
mod tests {
    use crate::frame::Body;
    use crate::mat3::Mat3;

    #[test]
    fn diagonal_matrix_is_already_decomposed_and_sorted() {
        let m: Mat3<Body> = Mat3::from_diagonal([3.0, 1.0, 2.0]);
        let eigen = m.symmetric_eigen().expect("converges");
        let [a, b, c] = eigen.values;
        assert!((a - 1.0).abs() < 1e-15 && (b - 2.0).abs() < 1e-15 && (c - 3.0).abs() < 1e-15);
    }

    /// Largest `‖A·v − λ·v‖` over the eigenpairs, relative to the largest entry of `A`.
    fn residual(m: Mat3<Body>) -> f64 {
        let eigen = m.symmetric_eigen().expect("converges");
        let worst = eigen
            .values
            .iter()
            .zip(eigen.vectors)
            .map(|(&value, vector)| (m * vector - vector * value).norm())
            .fold(0.0, f64::max);
        worst / m.max_abs()
    }

    fn symmetric(diag: [f64; 3], [xy, xz, yz]: [f64; 3]) -> Mat3<Body> {
        let [xx, yy, zz] = diag;
        Mat3::from_rows([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    }

    #[test]
    fn off_diagonal_patterns_that_cancel_in_sums_are_still_decomposed() {
        // A single product of inertia, one entry zero, or signs such that sums of entries and
        // squares cancel: convergence must be judged by the sum of squares and nothing else.
        let patterns = [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [0.0, 0.0, 0.5],
            [0.0, 0.5, 0.5],
            [0.5, 0.0, 0.5],
            [0.5, 0.5, 0.0],
            [-1.0, 1.0, 1.0],
            [1.0, -1.0, 1.0],
            [1.0, 1.0, -1.0],
        ];
        for pattern in patterns {
            let error = residual(symmetric([3.0, 2.0, 5.0], pattern));
            assert!(error < 1e-14, "{pattern:?}: residual {error:e}");
        }
    }

    #[test]
    fn accuracy_is_independent_of_scale() {
        // An inertia tensor of a 30 g vehicle has entries near 1e-5 kg·m²; scale far below that.
        for scale in [1e-8, 1.0, 1e8] {
            let m = symmetric([3.0, 2.0, 5.0], [0.4, -0.3, 0.2]) * scale;
            let error = residual(m);
            assert!(error < 1e-14, "scale {scale:e}: residual {error:e}");
        }
    }

    #[test]
    fn non_finite_input_is_reported() {
        let m: Mat3<Body> = Mat3::from_diagonal([1.0, f64::INFINITY, 2.0]);
        assert!(m.symmetric_eigen().is_err());
    }
}
