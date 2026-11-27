//! The rigid-body state and the Newton–Euler equations of motion, Eqs. (RB-1) and (RB-2).

use mjk_math::{Body, Rotation, StateHash, StateHasher, Vec3, World};

use crate::mass::MassProperties;

/// The state of a rigid body: where its center of mass is, how it moves, how it is oriented
/// and how it turns.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidBodyState {
    /// Position of the center of mass, world axes (NED).
    pub position_m: Vec3<World>,
    /// Velocity of the center of mass relative to the world, world axes.
    pub velocity_m_s: Vec3<World>,
    /// Attitude: the rotation from body axes to world axes, Eq. (CONV-1).
    pub q_wb: Rotation<World, Body>,
    /// Angular velocity of the body relative to the world, body axes: (p, q, r).
    pub omega_rad_s: Vec3<Body>,
}

impl StateHash for RigidBodyState {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.position_m.hash_state(hasher);
        self.velocity_m_s.hash_state(hasher);
        self.q_wb.hash_state(hasher);
        self.omega_rad_s.hash_state(hasher);
    }
}

impl RigidBodyState {
    /// A body at rest at `position_m` with attitude `q_wb`.
    #[must_use]
    pub const fn at_rest(position_m: Vec3<World>, q_wb: Rotation<World, Body>) -> Self {
        Self {
            position_m,
            velocity_m_s: Vec3::zero(),
            q_wb,
            omega_rad_s: Vec3::zero(),
        }
    }

    /// True when every component is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.position_m.is_finite()
            && self.velocity_m_s.is_finite()
            && self.q_wb.unit_quat().quat().is_finite()
            && self.omega_rad_s.is_finite()
    }

    /// Kinetic energy, translational plus rotational, Eq. (RB-8).
    #[must_use]
    pub fn kinetic_energy_j(&self, props: &MassProperties) -> f64 {
        let translational = 0.5 * props.mass_kg() * self.velocity_m_s.norm_squared();
        let h_b = props.inertia_kg_m2() * self.omega_rad_s;
        translational + 0.5 * self.omega_rad_s.dot(h_b)
    }

    /// Angular momentum about the center of mass, world axes, Eq. (RB-8).
    #[must_use]
    pub fn angular_momentum_w(&self, props: &MassProperties) -> Vec3<World> {
        self.q_wb.rotate(props.inertia_kg_m2() * self.omega_rad_s)
    }
}

/// The time derivative of a [`RigidBodyState`].
///
/// The attitude rate is carried as the body angular velocity, the tangent of the attitude on
/// SO(3): integrators either form q̇ = ½ q ⊗ (0, ω), Eq. (CONV-9), or step on the group with
/// the exponential map, Eq. (CONV-10).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateDerivative {
    /// Rate of change of position: the velocity, world axes.
    pub velocity_m_s: Vec3<World>,
    /// Rate of change of velocity, world axes.
    pub acceleration_m_s2: Vec3<World>,
    /// Attitude rate, as the body angular velocity.
    pub omega_rad_s: Vec3<Body>,
    /// Rate of change of the body angular velocity, body axes.
    pub angular_acceleration_rad_s2: Vec3<Body>,
}

/// The Newton–Euler equations: the state derivative under a total force, world axes, and a
/// total moment about the center of mass, body axes.
///
/// The force includes gravity; the caller sums every load first. Nothing is allocated and no
/// matrix is inverted: the inverse inertia was computed when the mass properties were built.
#[must_use]
pub fn derivative(
    state: &RigidBodyState,
    props: &MassProperties,
    force_w: Vec3<World>,
    moment_b: Vec3<Body>,
) -> StateDerivative {
    debug_assert!(
        state.is_finite(),
        "derivative of a non-finite state: {state:?}"
    );
    debug_assert!(
        force_w.is_finite() && moment_b.is_finite(),
        "non-finite loads: force {force_w:?}, moment {moment_b:?}"
    );
    let omega = state.omega_rad_s;
    // Eq. (RB-2): J·ω̇ = M − ω × (J·ω)
    let h_b = props.inertia_kg_m2() * omega;
    let angular_acceleration = props.inertia_inverse() * (moment_b - omega.cross(h_b));
    StateDerivative {
        velocity_m_s: state.velocity_m_s,
        // Eq. (RB-1): m·v̇ = F
        acceleration_m_s2: force_w / props.mass_kg(),
        omega_rad_s: omega,
        angular_acceleration_rad_s2: angular_acceleration,
    }
}
