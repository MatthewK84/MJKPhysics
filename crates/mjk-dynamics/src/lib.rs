//! Rigid-body dynamics, numerical integrators and the fixed-step world loop.
//!
//! Depends only on [`mjk_math`]. Holds:
//!
//! - the rigid-body state, the Newton–Euler equations and mass properties
//!   ([#10](https://github.com/MatthewK84/MJKPhysics/issues/10));
//! - the fixed-step integrators, including the Lie-group method
//!   ([#11](https://github.com/MatthewK84/MJKPhysics/issues/11),
//!   [#12](https://github.com/MatthewK84/MJKPhysics/issues/12));
//! - the world loop and its multi-rate scheduler
//!   ([#13](https://github.com/MatthewK84/MJKPhysics/issues/13)).
//!
//! Will hold determinism and replay ([#14](https://github.com/MatthewK84/MJKPhysics/issues/14)),
//! and trim and linearization for fixed-wing analysis
//! ([#47](https://github.com/MatthewK84/MJKPhysics/issues/47),
//! [#48](https://github.com/MatthewK84/MJKPhysics/issues/48)).

mod clock;
mod integrator;
pub mod mass;
mod rigid_body;
mod rigid_body_system;
mod world;
mod wrench;

pub use clock::{Clock, DEFAULT_PHYSICS_HZ, Rate, RateError};
pub use integrator::{
    Coordinates, ExplicitEuler, Integrator, IntegratorKind, LieSystem, Rk4, Rkmk4,
    SemiImplicitEuler, System,
};
pub use rigid_body::{RigidBodyState, StateDerivative, derivative};
pub use rigid_body_system::{
    LoadModel, Loads, RigidBodyCoordinates, RigidBodySystem, RigidBodyTangent,
};
pub use world::{Advance, Model, ModelCoordinates, World};
pub use wrench::Wrench;
