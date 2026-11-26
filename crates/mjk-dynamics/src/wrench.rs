//! Forces and moments acting on a rigid body.

use std::ops::{Add, AddAssign};

use mjk_math::{Frame, Rotation, Vec3};

/// A force and a moment about the center of mass, both resolved in frame `F`.
///
/// Models report their loads as wrenches, which add component by component. Gravity and
/// aerodynamics are both wrenches; so is the reaction torque of a rotor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wrench<F: Frame> {
    /// Force.
    pub force_n: Vec3<F>,
    /// Moment about the center of mass.
    pub moment_n_m: Vec3<F>,
}

impl<F: Frame> Wrench<F> {
    /// A wrench from its force and its moment about the center of mass.
    #[must_use]
    pub const fn new(force_n: Vec3<F>, moment_n_m: Vec3<F>) -> Self {
        Self {
            force_n,
            moment_n_m,
        }
    }

    /// No force and no moment.
    #[must_use]
    pub const fn zero() -> Self {
        Self::new(Vec3::zero(), Vec3::zero())
    }

    /// A force applied at a point, given from the center of mass: the force, and the moment
    /// `r × F` it exerts about the center of mass, Eq. (RB-3).
    #[must_use]
    pub fn from_force_at(force_n: Vec3<F>, point_from_cg_m: Vec3<F>) -> Self {
        Self::new(force_n, point_from_cg_m.cross(force_n))
    }

    /// The same wrench resolved in frame `To`. Both vectors rotate; the moment stays about the
    /// center of mass.
    #[must_use]
    pub fn resolved_in<To: Frame>(self, rotation: Rotation<To, F>) -> Wrench<To> {
        Wrench::new(
            rotation.rotate(self.force_n),
            rotation.rotate(self.moment_n_m),
        )
    }

    /// True when every component is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.force_n.is_finite() && self.moment_n_m.is_finite()
    }
}

impl<F: Frame> Default for Wrench<F> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<F: Frame> Add for Wrench<F> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.force_n + rhs.force_n, self.moment_n_m + rhs.moment_n_m)
    }
}

impl<F: Frame> AddAssign for Wrench<F> {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

#[cfg(test)]
mod tests {
    use mjk_math::{Body, Rotation, UnitQuat, Vec3, World};

    use super::Wrench;

    fn v(x: f64, y: f64, z: f64) -> Vec3<Body> {
        Vec3::new(x, y, z)
    }

    #[test]
    fn a_force_off_the_center_of_mass_exerts_a_moment() {
        // 2 N along +x at 0.5 m along +y: the moment is r × F = (0, 0, −1) N·m.
        let wrench = Wrench::from_force_at(v(2.0, 0.0, 0.0), v(0.0, 0.5, 0.0));
        assert_eq!(wrench.force_n, v(2.0, 0.0, 0.0));
        assert_eq!(wrench.moment_n_m, v(0.0, 0.0, -1.0));
        let through_cg = Wrench::from_force_at(v(1.0, 2.0, 3.0), Vec3::zero());
        assert_eq!(through_cg.moment_n_m, Vec3::zero());
    }

    #[test]
    fn wrenches_add_component_by_component() {
        let a = Wrench::new(v(1.0, 2.0, 3.0), v(-1.0, 0.5, 0.25));
        let b = Wrench::new(v(0.5, -2.0, 1.0), v(2.0, 0.5, -0.25));
        let expected = Wrench::new(v(1.5, 0.0, 4.0), v(1.0, 1.0, 0.0));
        assert_eq!(a + b, expected);
        let mut sum = Wrench::default();
        sum += a;
        sum += b;
        assert_eq!(sum, expected);
        assert_eq!(
            Wrench::<Body>::zero(),
            Wrench::new(Vec3::zero(), Vec3::zero())
        );
    }

    #[test]
    fn resolving_in_another_frame_rotates_both_vectors() {
        // 90° about z: body x maps to world y, body y to world −x.
        let q_wb = Rotation::<World, Body>::from_unit_quat(UnitQuat::exp([
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_2,
        ]));
        let world = Wrench::new(v(1.0, 0.0, 0.0), v(0.0, 2.0, 3.0)).resolved_in(q_wb);
        assert!((world.force_n - Vec3::new(0.0, 1.0, 0.0)).norm() < 1e-15);
        assert!((world.moment_n_m - Vec3::new(-2.0, 0.0, 3.0)).norm() < 1e-15);
    }

    #[test]
    fn finiteness_covers_force_and_moment() {
        assert!(Wrench::new(v(1.0, 2.0, 3.0), v(4.0, 5.0, 6.0)).is_finite());
        assert!(!Wrench::new(v(f64::NAN, 0.0, 0.0), Vec3::zero()).is_finite());
        assert!(!Wrench::new(Vec3::zero(), v(0.0, 0.0, f64::INFINITY)).is_finite());
    }
}
