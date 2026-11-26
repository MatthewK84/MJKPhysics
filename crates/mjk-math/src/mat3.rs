//! 3×3 matrices that act on vectors of one frame.

use std::fmt;
use std::marker::PhantomData;
use std::ops::{Add, Mul, Neg, Sub};

use crate::frame::Frame;
use crate::vec3::Vec3;

/// Row-major 3×3 entries.
type Rows = [[f64; 3]; 3];

/// A linear operator on vectors resolved in frame `F`, such as an inertia tensor in body axes.
///
/// It maps `Vec3<F>` to `Vec3<F>`. Moving a tensor to another frame goes through
/// [`Rotation::transform`](crate::Rotation::transform), which applies `R·M·Rᵀ`.
///
/// ```compile_fail,E0277
/// use mjk_math::{Body, Mat3, Vec3, World};
/// let inertia: Mat3<Body> = Mat3::from_diagonal([0.02, 0.03, 0.04]);
/// let omega_w: Vec3<World> = Vec3::new(0.0, 0.0, 1.0);
/// let _ = inertia * omega_w; // a body-frame tensor cannot act on a world-frame vector
/// ```
#[derive(Clone, Copy, PartialEq)]
pub struct Mat3<F: Frame> {
    rows: Rows,
    frame: PhantomData<F>,
}

impl<F: Frame> Mat3<F> {
    /// A matrix from its rows.
    #[must_use]
    pub const fn from_rows(rows: Rows) -> Self {
        Self {
            rows,
            frame: PhantomData,
        }
    }

    /// The rows.
    #[must_use]
    pub const fn rows(self) -> Rows {
        self.rows
    }

    /// The identity.
    #[must_use]
    pub const fn identity() -> Self {
        Self::from_diagonal([1.0, 1.0, 1.0])
    }

    /// The zero matrix.
    #[must_use]
    pub const fn zero() -> Self {
        Self::from_rows([[0.0; 3]; 3])
    }

    /// A diagonal matrix.
    #[must_use]
    pub const fn from_diagonal([d0, d1, d2]: [f64; 3]) -> Self {
        Self::from_rows([[d0, 0.0, 0.0], [0.0, d1, 0.0], [0.0, 0.0, d2]])
    }

    /// The diagonal entries.
    #[must_use]
    pub const fn diagonal(self) -> [f64; 3] {
        let [[m00, _, _], [_, m11, _], [_, _, m22]] = self.rows;
        [m00, m11, m22]
    }

    /// The skew-symmetric matrix `[a]×`, with `[a]× · b = a × b`.
    #[must_use]
    pub fn skew(a: Vec3<F>) -> Self {
        Self::from_rows([[0.0, -a.z, a.y], [a.z, 0.0, -a.x], [-a.y, a.x, 0.0]])
    }

    /// The outer product `a·bᵀ`.
    #[must_use]
    pub fn outer(a: Vec3<F>, b: Vec3<F>) -> Self {
        Self::from_rows([
            (a.x * b).to_array(),
            (a.y * b).to_array(),
            (a.z * b).to_array(),
        ])
    }

    /// The transpose.
    #[must_use]
    pub fn transpose(self) -> Self {
        let [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]] = self.rows;
        Self::from_rows([[m00, m10, m20], [m01, m11, m21], [m02, m12, m22]])
    }

    /// The trace.
    #[must_use]
    pub fn trace(self) -> f64 {
        self.diagonal().iter().sum()
    }

    /// The determinant.
    #[must_use]
    pub fn determinant(self) -> f64 {
        let [r0, r1, r2] = self.row_vectors();
        r0.dot(r1.cross(r2))
    }

    /// The inverse, or `None` when the matrix is singular or not finite.
    ///
    /// Computed from the adjugate: the rows of the inverse are the cross products of pairs of
    /// columns, divided by the determinant.
    #[must_use]
    pub fn inverse(self) -> Option<Self> {
        let [c0, c1, c2] = self.transpose().row_vectors();
        let det = c0.dot(c1.cross(c2));
        if !(det.is_finite() && det.abs() > 0.0) {
            return None;
        }
        let inverse = Self::from_rows([
            (c1.cross(c2) / det).to_array(),
            (c2.cross(c0) / det).to_array(),
            (c0.cross(c1) / det).to_array(),
        ]);
        inverse.is_finite().then_some(inverse)
    }

    /// True when every entry is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.rows.iter().flatten().all(|value| value.is_finite())
    }

    /// The largest absolute entry, a scale for relative tolerances.
    #[must_use]
    pub fn max_abs(self) -> f64 {
        self.rows
            .iter()
            .flatten()
            .fold(0.0, |acc: f64, value| acc.max(value.abs()))
    }

    fn row_vectors(self) -> [Vec3<F>; 3] {
        self.rows.map(Vec3::from_array)
    }

    fn map(self, op: impl Fn(f64) -> f64) -> Self {
        Self::from_rows(self.rows.map(|row| row.map(&op)))
    }

    fn zip(self, other: Self, op: impl Fn(f64, f64) -> f64) -> Self {
        let [a0, a1, a2] = self.rows;
        let [b0, b1, b2] = other.rows;
        let zip_row =
            |[x0, x1, x2]: [f64; 3], [y0, y1, y2]: [f64; 3]| [op(x0, y0), op(x1, y1), op(x2, y2)];
        Self::from_rows([zip_row(a0, b0), zip_row(a1, b1), zip_row(a2, b2)])
    }
}

impl<F: Frame> fmt::Debug for Mat3<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Mat3<{}>{:?}", F::NAME, self.rows)
    }
}

impl<F: Frame> Mul<Vec3<F>> for Mat3<F> {
    type Output = Vec3<F>;
    fn mul(self, rhs: Vec3<F>) -> Vec3<F> {
        let [r0, r1, r2] = self.row_vectors();
        Vec3::new(r0.dot(rhs), r1.dot(rhs), r2.dot(rhs))
    }
}

impl<F: Frame> Mul for Mat3<F> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        let [r0, r1, r2] = self.row_vectors();
        let columns = rhs.transpose();
        Self::from_rows([
            (columns * r0).to_array(),
            (columns * r1).to_array(),
            (columns * r2).to_array(),
        ])
    }
}

impl<F: Frame> Mul<f64> for Mat3<F> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        self.map(|value| value * rhs)
    }
}

impl<F: Frame> Add for Mat3<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        self.zip(rhs, |a, b| a + b)
    }
}

impl<F: Frame> Sub for Mat3<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        self.zip(rhs, |a, b| a - b)
    }
}

impl<F: Frame> Neg for Mat3<F> {
    type Output = Self;
    fn neg(self) -> Self {
        self.map(|value| -value)
    }
}

#[cfg(test)]
mod tests {
    use super::Mat3;
    use crate::frame::Body;
    use crate::vec3::Vec3;

    #[test]
    fn skew_matches_cross_product() {
        let a: Vec3<Body> = Vec3::new(1.0, -2.0, 3.0);
        let b: Vec3<Body> = Vec3::new(0.5, 4.0, -1.0);
        assert_eq!(Mat3::skew(a) * b, a.cross(b));
    }

    #[test]
    fn singular_matrix_has_no_inverse() {
        let singular: Mat3<Body> =
            Mat3::from_rows([[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 1.0, 1.0]]);
        assert!(singular.inverse().is_none());
    }

    #[test]
    fn diagonal_inverse() {
        let m: Mat3<Body> = Mat3::from_diagonal([2.0, 4.0, 8.0]);
        let inverse = m.inverse().expect("invertible");
        let product = m * inverse - Mat3::identity();
        assert!(product.max_abs() < 1e-15);
    }
}
