//! Rigid-body dynamics, numerical integrators and the fixed-step world loop.
//!
//! Depends only on [`mjk_math`]. Holds the rigid-body state, the Newton–Euler equations and
//! mass properties ([#10](https://github.com/MatthewK84/MJKPhysics/issues/10)). Will hold the
//! integrator family including a Lie-group method
//! ([#11](https://github.com/MatthewK84/MJKPhysics/issues/11),
//! [#12](https://github.com/MatthewK84/MJKPhysics/issues/12)), the multi-rate scheduler
//! ([#13](https://github.com/MatthewK84/MJKPhysics/issues/13)), determinism and replay
//! ([#14](https://github.com/MatthewK84/MJKPhysics/issues/14)), and trim and linearization for
//! fixed-wing analysis ([#47](https://github.com/MatthewK84/MJKPhysics/issues/47),
//! [#48](https://github.com/MatthewK84/MJKPhysics/issues/48)).

pub mod mass;
mod rigid_body;
mod wrench;

pub use rigid_body::{RigidBodyState, StateDerivative, derivative};
pub use wrench::Wrench;
