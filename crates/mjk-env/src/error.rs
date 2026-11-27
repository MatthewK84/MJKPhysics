//! Why an environment cannot be built or evaluated.

use std::fmt;

/// Why an environment cannot be built or evaluated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EnvironmentError {
    /// The temperature offset is not finite, or cools the coldest layer to absolute zero.
    InvalidTemperatureOffset {
        /// The offset, K.
        temperature_offset_k: f64,
    },
    /// The altitude lies outside the atmosphere model's range, or is not finite.
    AltitudeOutOfRange {
        /// The geometric altitude above mean sea level, m.
        altitude_m: f64,
    },
    /// The latitude lies outside ±90°, or is not finite.
    LatitudeOutOfRange {
        /// The latitude, rad.
        latitude_rad: f64,
    },
    /// The field elevation is not finite.
    InvalidFieldElevation {
        /// The elevation, m.
        field_elevation_m: f64,
    },
}

impl fmt::Display for EnvironmentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTemperatureOffset {
                temperature_offset_k,
            } => write!(
                f,
                "a temperature offset of {temperature_offset_k} K is not usable: it must be \
                 finite and leave the coldest layer above absolute zero"
            ),
            Self::AltitudeOutOfRange { altitude_m } => write!(
                f,
                "{altitude_m} m is outside the standard atmosphere's range, -5000 m to 20000 m"
            ),
            Self::LatitudeOutOfRange { latitude_rad } => write!(
                f,
                "a latitude of {latitude_rad} rad is outside -pi/2 to pi/2"
            ),
            Self::InvalidFieldElevation { field_elevation_m } => {
                write!(
                    f,
                    "a field elevation of {field_elevation_m} m is not finite"
                )
            }
        }
    }
}
