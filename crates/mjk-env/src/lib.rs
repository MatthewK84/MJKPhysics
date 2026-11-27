//! Atmosphere, gravity, wind and turbulence.
//!
//! Depends only on [`mjk_math`]. Holds:
//!
//! - the U.S. Standard Atmosphere 1976 from −5 km to 20 km, with a temperature offset, and
//!   standard or WGS-84 normal gravity ([#15](https://github.com/MatthewK84/MJKPhysics/issues/15));
//! - [`Environment`], which places them under the world origin at a field elevation.
//!
//! Will hold the steady wind, Dryden and von Kármán turbulence and discrete gust models
//! ([#53](https://github.com/MatthewK84/MJKPhysics/issues/53)).
//!
//! Transcendental functions come from `libm`, so results are identical on every platform.

pub mod atmosphere;
mod environment;
mod error;
pub mod gravity;
pub mod reference;

pub use atmosphere::{Air, Atmosphere};
pub use environment::Environment;
pub use error::EnvironmentError;
pub use gravity::{G0_M_S2, Gravity, NormalGravity};
