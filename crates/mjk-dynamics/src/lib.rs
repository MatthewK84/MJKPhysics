//! Rigid-body dynamics, numerical integrators and the fixed-step world loop.
//!
//! Depends only on [`mjk_math`]. Holds the rigid-body state, the Newton–Euler equations and
//! mass properties ([#10](https://github.com/MatthewK84/MJKPhysics/issues/10)), and the
//! fixed-step integrators ([#11](https://github.com/MatthewK84/MJKPhysics/issues/11)). Will
//! hold the Lie-group integrator ([#12](https://github.com/MatthewK84/MJKPhysics/issues/12)),
//! the multi-rate scheduler
//! ([#13](https://github.com/MatthewK84/MJKPhysics/issues/13)), determinism and replay
//! ([#14](https://github.com/MatthewK84/MJKPhysics/issues/14)), and trim and linearization for
//! fixed-wing analysis ([#47](https://github.com/MatthewK84/MJKPhysics/issues/47),
//! [#48](https://github.com/MatthewK84/MJKPhysics/issues/48)).

mod integrator;
pub mod mass;
mod rigid_body;
mod rigid_body_system;
mod wrench;

pub use integrator::{Coordinates, ExplicitEuler, Integrator, Rk4, SemiImplicitEuler, System};
pub use rigid_body::{RigidBodyState, StateDerivative, derivative};
pub use rigid_body_system::{LoadModel, Loads, RigidBodyCoordinates, RigidBodySystem};
pub use wrench::Wrench;
