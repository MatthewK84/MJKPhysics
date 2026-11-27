//! Rotations typed by the frames they connect.

use std::fmt;
use std::marker::PhantomData;
use std::ops::Mul;

use crate::frame::Frame;
use crate::mat3::Mat3;
use crate::quat::UnitQuat;
use crate::vec3::Vec3;

/// The rotation that maps coordinates in frame `From` to frame `To`: the frame-typed
/// $`\mathbf{q}_{\text{To},\text{From}}`$ of the conventions page.
///
/// The type parameters read like the notation: `q_wb: Rotation<World, Body>` is
/// $`\mathbf{q}_{WB}`$, and `q_wb.rotate(v_b)` is $`\mathbf{x}^W = \mathbf{R}_{WB}\,\mathbf{x}^B`$,
/// Eq. (CONV-1). Composition type-checks the way the subscripts cancel:
///
/// ```
/// use mjk_math::{Body, Rotation, Stability, Vec3, World};
/// let q_wb: Rotation<World, Body> = Rotation::identity();
/// let q_bs: Rotation<Body, Stability> = Rotation::identity();
/// let q_ws: Rotation<World, Stability> = q_wb * q_bs;
/// let v_w: Vec3<World> = q_ws.rotate(Vec3::new(1.0, 0.0, 0.0));
/// assert_eq!(v_w, Vec3::new(1.0, 0.0, 0.0));
/// ```
///
/// Rotating a vector from the wrong frame does not compile:
///
/// ```compile_fail,E0308
/// use mjk_math::{Body, Rotation, Vec3, World};
/// let q_wb: Rotation<World, Body> = Rotation::identity();
/// let v_w: Vec3<World> = Vec3::new(1.0, 0.0, 0.0);
/// let _ = q_wb.rotate(v_w); // expects a Vec3<Body>
/// ```
///
/// Neither does composing rotations whose inner frames differ:
///
/// ```compile_fail,E0308
/// use mjk_math::{Body, Rotation, Stability, World};
/// let q_wb: Rotation<World, Body> = Rotation::identity();
/// let q_ws: Rotation<World, Stability> = Rotation::identity();
/// let _ = q_wb * q_ws; // Body ≠ World: the subscripts don't cancel
/// ```
#[derive(Clone, Copy, PartialEq)]
pub struct Rotation<To: Frame, From: Frame> {
    q: UnitQuat,
    frames: PhantomData<(To, From)>,
}

impl<To: Frame, From: Frame> Rotation<To, From> {
    /// The identity rotation between two frames whose axes coincide.
    #[must_use]
    pub const fn identity() -> Self {
        Self::from_unit_quat(UnitQuat::identity())
    }

    /// The rotation represented by `q`. The caller asserts which frames it connects.
    #[must_use]
    pub const fn from_unit_quat(q: UnitQuat) -> Self {
        Self {
            q,
            frames: PhantomData,
        }
    }

    /// The underlying unit quaternion.
    #[must_use]
    pub const fn unit_quat(self) -> UnitQuat {
        self.q
    }

    /// Maps a vector from `From` coordinates to `To` coordinates.
    #[must_use]
    pub fn rotate(self, v: Vec3<From>) -> Vec3<To> {
        Vec3::from_array(self.q.rotate_array(v.to_array()))
    }

    /// The inverse rotation, from `To` back to `From`.
    #[must_use]
    pub fn inverse(self) -> Rotation<From, To> {
        Rotation::from_unit_quat(self.q.inverse())
    }

    /// The rotation matrix, row-major. Its columns are the `From` axes expressed in `To`,
    /// Eq. (CONV-1).
    #[must_use]
    pub fn matrix(self) -> [[f64; 3]; 3] {
        self.q.matrix()
    }

    /// Moves a tensor from `From` to `To` coordinates: `R·M·Rᵀ`.
    #[must_use]
    pub fn transform(self, m: Mat3<From>) -> Mat3<To> {
        let inverse = self.inverse();
        let basis: [Vec3<To>; 3] = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let columns = basis.map(|axis| self.rotate(m * inverse.rotate(axis)).to_array());
        Mat3::from_rows(columns).transpose()
    }

    /// The same rotation with rounding drift in the quaternion norm removed.
    #[must_use]
    pub fn renormalized(self) -> Self {
        Self::from_unit_quat(self.q.renormalized())
    }
}

impl<To: Frame, From: Frame> fmt::Debug for Rotation<To, From> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let q = self.q.quat();
        write!(
            f,
            "Rotation<{}←{}>({}, {}, {}, {})",
            To::NAME,
            From::NAME,
            q.w,
            q.x,
            q.y,
            q.z
        )
    }
}

/// Composition, `q_ab * q_bc = q_ac`: apply the right operand first.
impl<To: Frame, Mid: Frame, From: Frame> Mul<Rotation<Mid, From>> for Rotation<To, Mid> {
    type Output = Rotation<To, From>;
    fn mul(self, rhs: Rotation<Mid, From>) -> Rotation<To, From> {
        Rotation::from_unit_quat(self.q * rhs.q)
    }
}

#[cfg(test)]
mod tests {
    use super::Rotation;
    use crate::frame::{Body, World};
    use crate::quat::{Quat, UnitQuat};
    use crate::vec3::Vec3;

    #[test]
    fn debug_shows_direction() {
        let q: Rotation<World, Body> = Rotation::identity();
        assert_eq!(format!("{q:?}"), "Rotation<World←Body>(1, 0, 0, 0)");
    }

    #[test]
    fn yaw_of_ninety_degrees_points_the_nose_east() {
        let half = std::f64::consts::FRAC_PI_4;
        let yaw =
            UnitQuat::new(Quat::new(libm::cos(half), 0.0, 0.0, libm::sin(half))).expect("unit");
        let q_wb: Rotation<World, Body> = Rotation::from_unit_quat(yaw);
        let nose_w = q_wb.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(nose_w.x.abs() < 1e-15 && (nose_w.y - 1.0).abs() < 1e-15);
    }
}
