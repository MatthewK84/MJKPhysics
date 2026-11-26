//! Hamilton quaternions, scalar first (conventions, section 3.1).

use std::ops::{Add, Mul, Neg, Sub};

/// A general quaternion `w + x·i + y·j + z·k` with Hamilton's convention `ij = k`.
///
/// Used for quaternion algebra that leaves the unit sphere, such as attitude derivatives
/// `q̇ = ½·q ⊗ (0, ω)` (CONV-9). Rotations use [`UnitQuat`] and
/// [`Rotation`](crate::Rotation).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    /// Scalar part, `q0`.
    pub w: f64,
    /// First vector component, `q1`.
    pub x: f64,
    /// Second vector component, `q2`.
    pub y: f64,
    /// Third vector component, `q3`.
    pub z: f64,
}

impl Quat {
    /// A quaternion from its components, scalar first.
    #[must_use]
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// The multiplicative identity `(1, 0, 0, 0)`.
    #[must_use]
    pub const fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }

    /// The pure quaternion `(0, v)`.
    #[must_use]
    pub const fn pure([x, y, z]: [f64; 3]) -> Self {
        Self::new(0.0, x, y, z)
    }

    /// The vector part `[x, y, z]`.
    #[must_use]
    pub const fn vector(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// The conjugate `(w, −x, −y, −z)`.
    #[must_use]
    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// The four-dimensional dot product.
    #[must_use]
    pub fn dot(self, other: Self) -> f64 {
        self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Squared norm.
    #[must_use]
    pub fn norm_squared(self) -> f64 {
        self.dot(self)
    }

    /// Norm.
    #[must_use]
    pub fn norm(self) -> f64 {
        self.norm_squared().sqrt()
    }

    /// True when every component is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.w.is_finite() && self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

/// Hamilton product, Eq. (CONV-7).
impl Mul for Quat {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        let (pw, px, py, pz) = (self.w, self.x, self.y, self.z);
        let (qw, qx, qy, qz) = (rhs.w, rhs.x, rhs.y, rhs.z);
        Self::new(
            pw * qw - px * qx - py * qy - pz * qz,
            pw * qx + px * qw + py * qz - pz * qy,
            pw * qy - px * qz + py * qw + pz * qx,
            pw * qz + px * qy - py * qx + pz * qw,
        )
    }
}

impl Mul<f64> for Quat {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.w * rhs, self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Add for Quat {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(
            self.w + rhs.w,
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z,
        )
    }
}

impl Sub for Quat {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(
            self.w - rhs.w,
            self.x - rhs.x,
            self.y - rhs.y,
            self.z - rhs.z,
        )
    }
}

impl Neg for Quat {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.w, -self.x, -self.y, -self.z)
    }
}

/// A unit quaternion: a rotation without frame information.
///
/// The norm is 1 at construction. Products of unit quaternions drift from the unit sphere only
/// by rounding; [`UnitQuat::renormalized`] removes that drift. Frame-typed code uses
/// [`Rotation`](crate::Rotation), which wraps this type.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitQuat(Quat);

impl UnitQuat {
    /// The identity rotation.
    #[must_use]
    pub const fn identity() -> Self {
        Self(Quat::identity())
    }

    /// Normalizes `q`, or returns `None` if it is zero or not finite.
    #[must_use]
    pub fn new(q: Quat) -> Option<Self> {
        let norm = q.norm();
        (norm.is_finite() && norm > 0.0).then(|| Self(q * (1.0 / norm)))
    }

    /// Wraps a quaternion that is unit by construction (up to rounding), such as `Exp(θ)`.
    pub(crate) const fn from_unit(q: Quat) -> Self {
        Self(q)
    }

    /// The underlying quaternion.
    #[must_use]
    pub const fn quat(self) -> Quat {
        self.0
    }

    /// The inverse rotation (the conjugate).
    #[must_use]
    pub fn inverse(self) -> Self {
        Self(self.0.conjugate())
    }

    /// The same rotation with rounding drift in the norm removed.
    #[must_use]
    pub fn renormalized(self) -> Self {
        Self::new(self.0).unwrap_or(self)
    }

    /// Rotates `v`: `q ⊗ (0, v) ⊗ q*`, computed as `v + 2w·(u × v) + 2·u × (u × v)` with
    /// `u` the vector part.
    #[must_use]
    pub fn rotate_array(self, [vx, vy, vz]: [f64; 3]) -> [f64; 3] {
        let Quat { w, x, y, z } = self.0;
        let (tx, ty, tz) = (
            2.0 * (y * vz - z * vy),
            2.0 * (z * vx - x * vz),
            2.0 * (x * vy - y * vx),
        );
        [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ]
    }

    /// The rotation matrix, row-major, Eq. (CONV-8).
    #[must_use]
    pub fn matrix(self) -> [[f64; 3]; 3] {
        let Quat { w, x, y, z } = self.0;
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }
}

/// Composition: `p * q` applies `q` first, then `p`.
impl Mul for UnitQuat {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self(self.0 * rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use super::{Quat, UnitQuat};

    #[test]
    fn basis_products_follow_hamilton() {
        let i = Quat::new(0.0, 1.0, 0.0, 0.0);
        let j = Quat::new(0.0, 0.0, 1.0, 0.0);
        let k = Quat::new(0.0, 0.0, 0.0, 1.0);
        assert_eq!(i * j, k);
        assert_eq!(j * k, i);
        assert_eq!(k * i, j);
        assert_eq!(i * i, Quat::new(-1.0, 0.0, 0.0, 0.0));
    }

    #[test]
    fn zero_quaternion_is_not_a_rotation() {
        assert!(UnitQuat::new(Quat::new(0.0, 0.0, 0.0, 0.0)).is_none());
        assert!(UnitQuat::new(Quat::new(f64::NAN, 0.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn quarter_turn_about_z_takes_x_to_y() {
        let half = std::f64::consts::FRAC_PI_4;
        let q = UnitQuat::new(Quat::new(half.cos(), 0.0, 0.0, half.sin())).expect("unit");
        let [x, y, z] = q.rotate_array([1.0, 0.0, 0.0]);
        assert!(x.abs() < 1e-15 && (y - 1.0).abs() < 1e-15 && z.abs() < 1e-15);
    }
}
