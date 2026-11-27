//! The environment of a run: where the field is, its atmosphere and its gravity (#15).

use mjk_math::{Vec3, World};

use crate::atmosphere::{Air, Atmosphere};
use crate::error::EnvironmentError;
use crate::gravity::Gravity;

/// The atmosphere and gravity around the world origin.
///
/// The world origin sits at the field elevation above mean sea level, and altitude is the field
/// elevation minus the Down coordinate, Eq. (ATM-1). Gravity uses the same altitude: the
/// separation between mean sea level and the WGS-84 ellipsoid, at most about 100 m, changes
/// gravity by about 3 × 10⁻⁵ of its value and is neglected.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Environment {
    field_elevation_m: f64,
    atmosphere: Atmosphere,
    gravity: Gravity,
}

impl Environment {
    /// An environment with its origin at `field_elevation_m` above mean sea level.
    ///
    /// # Errors
    ///
    /// [`EnvironmentError::InvalidFieldElevation`] for a non-finite elevation.
    pub fn new(
        field_elevation_m: f64,
        atmosphere: Atmosphere,
        gravity: Gravity,
    ) -> Result<Self, EnvironmentError> {
        if !field_elevation_m.is_finite() {
            return Err(EnvironmentError::InvalidFieldElevation { field_elevation_m });
        }
        Ok(Self {
            field_elevation_m,
            atmosphere,
            gravity,
        })
    }

    /// The field elevation, m.
    #[must_use]
    pub const fn field_elevation_m(&self) -> f64 {
        self.field_elevation_m
    }

    /// The atmosphere.
    #[must_use]
    pub const fn atmosphere(&self) -> &Atmosphere {
        &self.atmosphere
    }

    /// The gravity model.
    #[must_use]
    pub const fn gravity(&self) -> &Gravity {
        &self.gravity
    }

    /// Geometric altitude above mean sea level of a world position, Eq. (ATM-1).
    #[must_use]
    pub fn altitude_m(&self, position: Vec3<World>) -> f64 {
        self.field_elevation_m - position.z
    }

    /// The air at a world position.
    ///
    /// # Errors
    ///
    /// [`EnvironmentError::AltitudeOutOfRange`] where the altitude is outside the atmosphere
    /// model's range.
    pub fn air(&self, position: Vec3<World>) -> Result<Air, EnvironmentError> {
        self.atmosphere.at(self.altitude_m(position))
    }

    /// The gravitational acceleration at a world position, world axes.
    #[must_use]
    pub fn gravity_w(&self, position: Vec3<World>) -> Vec3<World> {
        self.gravity.acceleration_w(self.altitude_m(position))
    }
}
