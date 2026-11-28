//! World composition, scheduling, airframes and scenarios.
//!
//! The top engine crate: it may depend on every other engine crate, and nothing in the engine may
//! depend on it. It holds configuration loading and the airframe format ([`config`],
//! [#18](https://github.com/MatthewK84/MJKPhysics/issues/18)), the columnar log writer
//! ([`log`]), recorded runs and their replay ([`run_log`]), and the validation studies
//! ([`validation`]). It will hold fixed-wing airframe definitions
//! ([#43](https://github.com/MatthewK84/MJKPhysics/issues/43)) and scenario files
//! ([#28](https://github.com/MatthewK84/MJKPhysics/issues/28)).

pub mod config;
pub mod log;
pub mod run_log;
pub mod validation;
