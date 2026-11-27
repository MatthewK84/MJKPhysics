//! The bodies, loads and simulation loop the analytic checks share.

use core::f64::consts::TAU;

use mjk_dynamics::mass::MassProperties;
use mjk_dynamics::{
    Clock, Integrator, IntegratorKind, LoadModel, Loads, Model, RateError, RigidBodyCoordinates,
    RigidBodyState, RigidBodySystem, World as Simulation,
};
use mjk_env::Gravity;
use mjk_math::{Body, Rotation, UnitQuat, Vec3, World};

/// Standard gravity, nothing else.
#[derive(Clone, Copy, Debug)]
pub struct Weight;

impl LoadModel for Weight {
    fn loads(&self, _: f64, _: &RigidBodyState, props: &MassProperties) -> Loads {
        Loads {
            force_w: Gravity::Standard.acceleration_w(0.0) * props.mass_kg(),
            moment_b: Vec3::zero(),
        }
    }
}

/// Standard gravity and linear drag, `−c·v`.
#[derive(Clone, Copy, Debug)]
pub struct WeightAndDrag {
    /// The drag coefficient `c`, N·s/m.
    pub drag_n_s_m: f64,
}

impl LoadModel for WeightAndDrag {
    fn loads(&self, _: f64, state: &RigidBodyState, props: &MassProperties) -> Loads {
        let weight_w = Gravity::Standard.acceleration_w(0.0) * props.mass_kg();
        Loads {
            force_w: weight_w - state.velocity_m_s * self.drag_n_s_m,
            moment_b: Vec3::zero(),
        }
    }
}

/// No force and no moment.
#[derive(Clone, Copy, Debug)]
pub struct TorqueFree;

impl LoadModel for TorqueFree {
    fn loads(&self, _: f64, _: &RigidBodyState, _: &MassProperties) -> Loads {
        Loads::default()
    }
}

/// A body with no subsystems: only its loads act on it.
#[derive(Clone, Copy, Debug)]
pub struct Free<L: LoadModel> {
    system: RigidBodySystem<L>,
}

impl<L: LoadModel> Model for Free<L> {
    type System = RigidBodySystem<L>;

    fn system(&self) -> &Self::System {
        &self.system
    }

    fn update(&mut self, _: &Clock, _: &RigidBodyCoordinates) {}
}

/// A simulation of one free body, stepping with `method` at `physics_hz`.
pub type FreeSimulation<L> = Simulation<Free<L>, IntegratorKind>;

/// The free body in its simulation loop, at tick zero.
///
/// # Errors
///
/// [`RateError::Zero`] for a physics rate of zero.
pub fn simulation<L: LoadModel>(
    loads: L,
    props: MassProperties,
    start: RigidBodyCoordinates,
    physics_hz: u32,
    method: IntegratorKind,
) -> Result<FreeSimulation<L>, RateError> {
    let model = Free {
        system: RigidBodySystem { props, loads },
    };
    Simulation::new(model, method, start, physics_hz)
}

/// Advances `simulation` by `steps` physics steps.
pub fn run<M: Model, I: Integrator<M::System>>(simulation: &mut Simulation<M, I>, steps: u32) {
    for _ in 0..steps {
        simulation.step();
    }
}

/// A body at the origin with identity attitude, velocity `velocity` and body rates `omega`.
#[must_use]
pub fn start(velocity: Vec3<World>, omega: Vec3<Body>) -> RigidBodyCoordinates {
    let mut state = RigidBodyState::at_rest(Vec3::zero(), Rotation::identity());
    state.velocity_m_s = velocity;
    state.omega_rad_s = omega;
    RigidBodyCoordinates::from(state)
}

/// The angle of the rotation between two attitudes, rad.
#[must_use]
pub fn angle_between(from: UnitQuat, to: UnitQuat) -> f64 {
    let rotation = (from.inverse() * to).log();
    rotation.iter().map(|c| c * c).sum::<f64>().sqrt()
}

/// `angle` wrapped into `(−π, π]`.
fn wrap(angle_rad: f64) -> f64 {
    angle_rad - TAU * libm::round(angle_rad / TAU)
}

/// An angle followed through many turns: the sum of its wrapped increments.
#[derive(Clone, Copy, Debug)]
pub struct Unwrapped {
    last_rad: f64,
    total_rad: f64,
}

impl Unwrapped {
    /// Starts at `angle_rad`.
    #[must_use]
    pub const fn new(angle_rad: f64) -> Self {
        Self {
            last_rad: angle_rad,
            total_rad: 0.0,
        }
    }

    /// Moves to `angle_rad`, taking the shorter way round from the last angle.
    pub fn advance(&mut self, angle_rad: f64) {
        self.total_rad += wrap(angle_rad - self.last_rad);
        self.last_rad = angle_rad;
    }

    /// The angle turned since the start, rad.
    #[must_use]
    pub const fn total_rad(&self) -> f64 {
        self.total_rad
    }
}
