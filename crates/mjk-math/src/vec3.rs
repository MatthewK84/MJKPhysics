//! Three-component vectors tagged with the frame they are resolved in.

use std::fmt;
use std::marker::PhantomData;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

use crate::frame::Frame;

/// A vector whose components are resolved in frame `F`.
///
/// Arithmetic only combines vectors in the same frame, so frame mistakes are compile errors:
///
/// ```
/// use mjk_math::{Body, Vec3};
/// let a: Vec3<Body> = Vec3::new(1.0, 0.0, 0.0);
/// let b: Vec3<Body> = Vec3::new(0.0, 1.0, 0.0);
/// assert_eq!(a.cross(b), Vec3::new(0.0, 0.0, 1.0));
/// ```
///
/// ```compile_fail,E0308
/// use mjk_math::{Body, Vec3, World};
/// let a: Vec3<Body> = Vec3::new(1.0, 0.0, 0.0);
/// let b: Vec3<World> = Vec3::new(0.0, 1.0, 0.0);
/// let _ = a + b; // a body-frame vector cannot be added to a world-frame vector
/// ```
#[derive(Clone, Copy, PartialEq)]
pub struct Vec3<F: Frame> {
    /// First component (North in `World`, Forward in `Body`).
    pub x: f64,
    /// Second component (East in `World`, Right in `Body`).
    pub y: f64,
    /// Third component (Down in both).
    pub z: f64,
    frame: PhantomData<F>,
}

impl<F: Frame> Vec3<F> {
    /// A vector from its components in frame `F`.
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            x,
            y,
            z,
            frame: PhantomData,
        }
    }

    /// The zero vector.
    #[must_use]
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// A vector from `[x, y, z]`.
    #[must_use]
    pub const fn from_array([x, y, z]: [f64; 3]) -> Self {
        Self::new(x, y, z)
    }

    /// The components as `[x, y, z]`.
    #[must_use]
    pub const fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Dot product.
    #[must_use]
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product, right-handed.
    #[must_use]
    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    /// Squared Euclidean norm.
    #[must_use]
    pub fn norm_squared(self) -> f64 {
        self.dot(self)
    }

    /// Euclidean norm.
    #[must_use]
    pub fn norm(self) -> f64 {
        self.norm_squared().sqrt()
    }

    /// The unit vector in this direction, or `None` for a zero or non-finite vector.
    #[must_use]
    pub fn normalized(self) -> Option<Self> {
        let norm = self.norm();
        (norm.is_finite() && norm > 0.0).then(|| self / norm)
    }

    /// True when every component is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl<F: Frame> Default for Vec3<F> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<F: Frame> fmt::Debug for Vec3<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vec3<{}>({}, {}, {})", F::NAME, self.x, self.y, self.z)
    }
}

impl<F: Frame> Add for Vec3<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl<F: Frame> Sub for Vec3<F> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl<F: Frame> Neg for Vec3<F> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl<F: Frame> Mul<f64> for Vec3<F> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl<F: Frame> Mul<Vec3<F>> for f64 {
    type Output = Vec3<F>;
    fn mul(self, rhs: Vec3<F>) -> Vec3<F> {
        rhs * self
    }
}

impl<F: Frame> Div<f64> for Vec3<F> {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl<F: Frame> AddAssign for Vec3<F> {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl<F: Frame> SubAssign for Vec3<F> {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

#[cfg(test)]
mod tests {
    use super::Vec3;
    use crate::frame::{Body, World};

    #[test]
    fn right_handed_basis() {
        let x: Vec3<World> = Vec3::new(1.0, 0.0, 0.0);
        let y: Vec3<World> = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(x.cross(y), Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn normalizing_zero_fails() {
        assert!(Vec3::<Body>::zero().normalized().is_none());
        assert!(Vec3::<Body>::new(f64::NAN, 0.0, 0.0).normalized().is_none());
    }

    #[test]
    fn debug_names_the_frame() {
        let v: Vec3<Body> = Vec3::new(1.0, -2.0, 0.5);
        assert_eq!(format!("{v:?}"), "Vec3<Body>(1, -2, 0.5)");
    }
}
