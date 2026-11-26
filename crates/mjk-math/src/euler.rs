//! Euler angles, 3-2-1 sequence, for input and output only (conventions, section 3.3).

use libm::{atan2, cos, sin};

use crate::frame::Frame;
use crate::quat::{Quat, UnitQuat};
use crate::rotation::Rotation;

/// When `|cos θ|` (pitch) is below this, the rotation is treated as gimbal-locked: only
/// `ψ ∓ φ` is defined, so roll is set to zero (CONV-12).
///
/// The threshold sits just above rounding noise, so it catches pitch exactly at ±90° in
/// floating point. Short of that, the well-conditioned roll formula in
/// [`Rotation::to_euler`] already rebuilds the rotation to rounding precision, so a wider
/// threshold would only discard information: with `1e-10` the round-trip error was `1e-10`.
pub const GIMBAL_LOCK_COS: f64 = 1e-15;

/// Yaw `ψ`, pitch `θ` and roll `φ` of the 3-2-1 sequence, `R = R_z(ψ)·R_y(θ)·R_x(φ)`
/// (CONV-11).
///
/// For human-facing input and output only: scenario files, logs, the HUD. Euler angles are
/// never state and their rates are never integrated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EulerAngles {
    /// Yaw `ψ` in `(−π, π]`, positive nose right.
    pub yaw_rad: f64,
    /// Pitch `θ` in `[−π/2, π/2]`, positive nose up.
    pub pitch_rad: f64,
    /// Roll `φ` in `(−π, π]`, positive right wing down.
    pub roll_rad: f64,
}

/// The unit quaternion of a rotation by `angle` about a coordinate axis.
fn axis_rotation(angle: f64, axis: [f64; 3]) -> Quat {
    let half = 0.5 * angle;
    let s = sin(half);
    let [x, y, z] = axis;
    Quat::new(cos(half), s * x, s * y, s * z)
}

impl<To: Frame, From: Frame> Rotation<To, From> {
    /// The rotation `R_z(ψ)·R_y(θ)·R_x(φ)`, Eq. (CONV-11).
    #[must_use]
    pub fn from_euler(angles: EulerAngles) -> Self {
        let yaw = axis_rotation(angles.yaw_rad, [0.0, 0.0, 1.0]);
        let pitch = axis_rotation(angles.pitch_rad, [0.0, 1.0, 0.0]);
        let roll = axis_rotation(angles.roll_rad, [1.0, 0.0, 0.0]);
        Self::from_unit_quat(UnitQuat::from_unit(yaw * pitch * roll))
    }

    /// Yaw, pitch and roll of this rotation, Eq. (CONV-12), computed as in Eq. (SO3-11).
    ///
    /// Yaw is `atan2(R₂₁, R₁₁)` and pitch is `atan2(−R₃₁, √(R₁₁² + R₂₁²))`, which equals
    /// `−asin R₃₁` but stays accurate near ±90°. Roll is computed from the yaw just found,
    /// `atan2(sin ψ·R₁₃ − cos ψ·R₂₃, cos ψ·R₂₂ − sin ψ·R₁₂)`, rather than as
    /// `atan2(R₃₂, R₃₃)`: near gimbal lock those two entries shrink with `cos θ`, while these
    /// do not, so roll absorbs any yaw error and the rebuilt rotation stays exact.
    ///
    /// At gimbal lock ([`GIMBAL_LOCK_COS`]) roll is zero and yaw is `atan2(−R₁₂, R₂₂)`, for
    /// either sign of pitch.
    #[must_use]
    pub fn to_euler(self) -> EulerAngles {
        let [[r11, r12, r13], [r21, r22, r23], [r31, _, _]] = self.matrix();
        let cos_pitch = (r11 * r11 + r21 * r21).sqrt();
        let pitch_rad = atan2(-r31, cos_pitch);
        if cos_pitch < GIMBAL_LOCK_COS {
            return EulerAngles {
                yaw_rad: atan2(-r12, r22),
                pitch_rad,
                roll_rad: 0.0,
            };
        }
        let yaw_rad = atan2(r21, r11);
        let (sin_yaw, cos_yaw) = (sin(yaw_rad), cos(yaw_rad));
        let roll_rad = atan2(sin_yaw * r13 - cos_yaw * r23, cos_yaw * r22 - sin_yaw * r12);
        EulerAngles {
            yaw_rad,
            pitch_rad,
            roll_rad,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use super::EulerAngles;
    use crate::frame::{Body, World};
    use crate::rotation::Rotation;
    use crate::vec3::Vec3;

    fn attitude(yaw: f64, pitch: f64, roll: f64) -> Rotation<World, Body> {
        Rotation::from_euler(EulerAngles {
            yaw_rad: yaw,
            pitch_rad: pitch,
            roll_rad: roll,
        })
    }

    #[test]
    fn signs_match_the_conventions() {
        let nose = Vec3::<Body>::new(1.0, 0.0, 0.0);
        let right_wing = Vec3::<Body>::new(0.0, 1.0, 0.0);
        assert!(
            attitude(0.2, 0.0, 0.0).rotate(nose).y > 0.0,
            "positive yaw: nose right"
        );
        assert!(
            attitude(0.0, 0.2, 0.0).rotate(nose).z < 0.0,
            "positive pitch: nose up"
        );
        assert!(
            attitude(0.0, 0.0, 0.2).rotate(right_wing).z > 0.0,
            "positive roll: right wing down"
        );
    }

    #[test]
    fn gimbal_lock_keeps_only_the_defined_combination() {
        for (pitch, combined) in [(FRAC_PI_2, 0.9 - 0.3), (-FRAC_PI_2, 0.9 + 0.3)] {
            let original = attitude(0.9, pitch, 0.3);
            let angles = original.to_euler();
            assert!(angles.roll_rad.abs() < 1e-15);
            assert!(
                (angles.yaw_rad - combined).abs() < 1e-9,
                "yaw {}",
                angles.yaw_rad
            );
            assert!(original.angle_to(Rotation::from_euler(angles)) < 1e-9);
        }
    }
}
