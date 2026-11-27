//! The rigid body as a [`System`] that integrators can advance, Eq. (INT-4), and as a
//! [`LieSystem`] with an exponential chart for its attitude, Eq. (INT-6).

use std::ops::{Add, Mul};

use mjk_math::so3::right_jacobian_inverse;
use mjk_math::{Body, Quat, Rotation, UnitQuat, Vec3, World};

use crate::integrator::{Coordinates, LieSystem, System};
use crate::mass::MassProperties;
use crate::rigid_body::{RigidBodyState, StateDerivative, derivative};

/// Rigid-body coordinates as integrators see them: the [`RigidBodyState`] with the attitude as
/// four raw quaternion components.
///
/// Intermediate Runge–Kutta stages leave the unit sphere slightly; normalizing them would cost
/// RK4 two orders of accuracy, so the attitude is a plain [`Quat`] here and is normalized only
/// by [`Coordinates::project`] at the end of each step.
///
/// The same type holds the rate of change: each field is then the time derivative of the
/// same field, which is why the names carry no units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidBodyCoordinates {
    /// Position of the center of mass, world axes; as a rate, the velocity.
    pub position: Vec3<World>,
    /// Velocity of the center of mass, world axes; as a rate, the acceleration.
    pub velocity: Vec3<World>,
    /// Attitude quaternion, body to world; as a rate, q̇.
    pub attitude: Quat,
    /// Angular velocity, body axes; as a rate, the angular acceleration.
    pub angular_velocity: Vec3<Body>,
}

impl RigidBodyCoordinates {
    /// The state these coordinates describe, with the attitude normalized. `None` when the
    /// quaternion is zero or not finite.
    #[must_use]
    pub fn to_state(&self) -> Option<RigidBodyState> {
        let q_wb = UnitQuat::new(self.attitude)?;
        Some(RigidBodyState {
            position_m: self.position,
            velocity_m_s: self.velocity,
            q_wb: Rotation::from_unit_quat(q_wb),
            omega_rad_s: self.angular_velocity,
        })
    }
}

impl From<RigidBodyState> for RigidBodyCoordinates {
    fn from(state: RigidBodyState) -> Self {
        Self {
            position: state.position_m,
            velocity: state.velocity_m_s,
            attitude: state.q_wb.unit_quat().quat(),
            angular_velocity: state.omega_rad_s,
        }
    }
}

impl Add for RigidBodyCoordinates {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            position: self.position + rhs.position,
            velocity: self.velocity + rhs.velocity,
            attitude: self.attitude + rhs.attitude,
            angular_velocity: self.angular_velocity + rhs.angular_velocity,
        }
    }
}

impl Mul<f64> for RigidBodyCoordinates {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            position: self.position * rhs,
            velocity: self.velocity * rhs,
            attitude: self.attitude * rhs,
            angular_velocity: self.angular_velocity * rhs,
        }
    }
}

impl Coordinates for RigidBodyCoordinates {
    fn positions(self) -> Self {
        Self {
            velocity: Vec3::zero(),
            angular_velocity: Vec3::zero(),
            ..self
        }
    }

    fn velocities(self) -> Self {
        Self {
            position: Vec3::zero(),
            attitude: Quat::new(0.0, 0.0, 0.0, 0.0),
            ..self
        }
    }

    /// Renormalizes the attitude quaternion, Eq. (INT-4). A zero or non-finite quaternion is left
    /// as it is, so the failure shows up in the state rather than being hidden.
    fn project(self) -> Self {
        let attitude = UnitQuat::new(self.attitude).map_or(self.attitude, UnitQuat::quat);
        Self { attitude, ..self }
    }
}

/// The total force (world axes, gravity included) and moment about the center of mass (body
/// axes) acting on a rigid body.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Loads {
    /// Total force, world axes.
    pub force_w: Vec3<World>,
    /// Total moment about the center of mass, body axes.
    pub moment_b: Vec3<Body>,
}

/// Everything that pushes on the body: gravity, aerodynamics, propulsion.
///
/// A load model must be a pure function of time and state during a step: integrators evaluate it
/// at intermediate stages. Inputs that change, such as controller commands, are updated between
/// steps and held during one.
pub trait LoadModel {
    /// The loads at `time_s` on a body in `state` with mass properties `props`.
    fn loads(&self, time_s: f64, state: &RigidBodyState, props: &MassProperties) -> Loads;
}

/// A rigid body and the loads acting on it, as a [`System`].
#[derive(Clone, Copy, Debug)]
pub struct RigidBodySystem<L: LoadModel> {
    /// Mass properties of the body.
    pub props: MassProperties,
    /// The loads acting on it.
    pub loads: L,
}

impl<L: LoadModel> RigidBodySystem<L> {
    /// The Newton–Euler rates at coordinates `y`. The physics sees the normalized attitude: a
    /// non-unit stage quaternion would scale every rotated vector by |q|². `None` when the
    /// attitude is zero or not finite.
    fn rates_at(&self, time_s: f64, y: &RigidBodyCoordinates) -> Option<StateDerivative> {
        let state = y.to_state()?;
        let loads = self.loads.loads(time_s, &state, &self.props);
        Some(derivative(
            &state,
            &self.props,
            loads.force_w,
            loads.moment_b,
        ))
    }
}

impl<L: LoadModel> System for RigidBodySystem<L> {
    type Coordinates = RigidBodyCoordinates;

    fn rate(&self, time_s: f64, y: &RigidBodyCoordinates) -> RigidBodyCoordinates {
        let Some(rates) = self.rates_at(time_s, y) else {
            return *y * f64::NAN;
        };
        // Eq. (INT-4): q̇ = ½ q ⊗ (0, ω), with the raw stage quaternion.
        let omega = Quat::pure(rates.omega_rad_s.to_array());
        RigidBodyCoordinates {
            position: rates.velocity_m_s,
            velocity: rates.acceleration_m_s2,
            attitude: y.attitude * omega * 0.5,
            angular_velocity: rates.angular_acceleration_rad_s2,
        }
    }
}

/// Tangent coordinates of the rigid body's chart around a base state, Eq. (INT-6): changes of
/// position, velocity and angular velocity, and a rotation vector θ applied on the right of the
/// base attitude, in body axes (CONV-10). The same type holds their rate of change.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RigidBodyTangent {
    /// Change of position, world axes; as a rate, the velocity.
    pub position: Vec3<World>,
    /// Change of velocity, world axes; as a rate, the acceleration.
    pub velocity: Vec3<World>,
    /// Rotation vector θ from the base attitude, body axes; as a rate, θ̇.
    pub rotation: Vec3<Body>,
    /// Change of angular velocity, body axes; as a rate, the angular acceleration.
    pub angular_velocity: Vec3<Body>,
}

impl Add for RigidBodyTangent {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            position: self.position + rhs.position,
            velocity: self.velocity + rhs.velocity,
            rotation: self.rotation + rhs.rotation,
            angular_velocity: self.angular_velocity + rhs.angular_velocity,
        }
    }
}

impl Mul<f64> for RigidBodyTangent {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            position: self.position * rhs,
            velocity: self.velocity * rhs,
            rotation: self.rotation * rhs,
            angular_velocity: self.angular_velocity * rhs,
        }
    }
}

impl Coordinates for RigidBodyTangent {
    fn positions(self) -> Self {
        Self {
            velocity: Vec3::zero(),
            angular_velocity: Vec3::zero(),
            ..self
        }
    }

    fn velocities(self) -> Self {
        Self {
            position: Vec3::zero(),
            rotation: Vec3::zero(),
            ..self
        }
    }

    /// Tangent vectors live in a vector space with no constraint to restore.
    fn project(self) -> Self {
        self
    }
}

impl<L: LoadModel> LieSystem for RigidBodySystem<L> {
    type Tangent = RigidBodyTangent;

    fn retract(&self, base: &RigidBodyCoordinates, u: &RigidBodyTangent) -> RigidBodyCoordinates {
        // Eq. (INT-6): q = q₀ ⊗ Exp(θ), increments on the right (CONV-10).
        let turn = UnitQuat::exp(u.rotation.to_array()).quat();
        RigidBodyCoordinates {
            position: base.position + u.position,
            velocity: base.velocity + u.velocity,
            attitude: base.attitude * turn,
            angular_velocity: base.angular_velocity + u.angular_velocity,
        }
    }

    fn tangent_rate(
        &self,
        time_s: f64,
        base: &RigidBodyCoordinates,
        u: &RigidBodyTangent,
    ) -> RigidBodyTangent {
        let Some(rates) = self.rates_at(time_s, &self.retract(base, u)) else {
            return *u * f64::NAN;
        };
        RigidBodyTangent {
            position: rates.velocity_m_s,
            velocity: rates.acceleration_m_s2,
            // Eq. (INT-6): θ̇ = J_r⁻¹(θ)·ω, Eq. (SO3-5).
            rotation: right_jacobian_inverse(u.rotation) * rates.omega_rad_s,
            angular_velocity: rates.angular_acceleration_rad_s2,
        }
    }
}
