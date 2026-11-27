//! Gravity: the constant standard value, or WGS-84 normal gravity (#15).
//!
//! Normal gravity is the gravity of the WGS-84 reference ellipsoid, earth rotation included. On
//! the ellipsoid it follows Somigliana's closed form, Eq. (ATM-9); above it, the second-order
//! expansion in height of NIMA TR8350.2, Eq. (ATM-10), which stays within 1.5 × 10⁻⁶ m/s² of the
//! exact field from −5 km to 20 km.

use mjk_math::{Vec3, World};

use crate::error::EnvironmentError;

/// Standard gravity, m/s².
pub const G0_M_S2: f64 = 9.806_65;

/// WGS-84 constants. The four defining parameters (a, 1/f, GM, ω) are those of NIMA TR8350.2,
/// Table 3.1, as published in `GeographicLib`'s `Constants.hpp`. The derived constants are computed
/// from them with the closed formulas of the level ellipsoid (Heiskanen and Moritz, 1967) at 40
/// significant digits and rounded to double precision; a test recomputes them.
pub mod wgs84 {
    /// Semi-major axis, m.
    pub const SEMI_MAJOR_AXIS_M: f64 = 6_378_137.0;
    /// Inverse flattening.
    pub const INVERSE_FLATTENING: f64 = 298.257_223_563;
    /// Flattening.
    pub const FLATTENING: f64 = 1.0 / INVERSE_FLATTENING;
    /// Earth's gravitational constant, m³/s².
    pub const GM_M3_S2: f64 = 3.986_004_418e14;
    /// Earth's angular velocity, rad/s.
    pub const ANGULAR_VELOCITY_RAD_S: f64 = 7.292_115e-5;
    /// First eccentricity squared, (a² − b²)/a².
    pub const ECCENTRICITY_SQUARED: f64 = 6.694_379_990_141_317e-3;
    /// Normal gravity at the equator, m/s².
    pub const EQUATORIAL_GRAVITY_M_S2: f64 = 9.780_325_335_903_892;
    /// Normal gravity at the poles, m/s².
    pub const POLAR_GRAVITY_M_S2: f64 = 9.832_184_937_863_4;
    /// Somigliana's constant k = b·γp / (a·γe) − 1.
    pub const SOMIGLIANA_K: f64 = 1.931_852_652_458_274e-3;
    /// m = ω²a²b / GM.
    pub const GRAVITY_RATIO_M: f64 = 3.449_786_506_840_845e-3;
}

/// Normal gravity at one latitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalGravity {
    latitude_rad: f64,
}

impl NormalGravity {
    /// Normal gravity at geodetic latitude `latitude_rad`.
    ///
    /// # Errors
    ///
    /// [`EnvironmentError::LatitudeOutOfRange`] for a latitude outside ±π/2 or not finite.
    pub fn new(latitude_rad: f64) -> Result<Self, EnvironmentError> {
        let pole = core::f64::consts::FRAC_PI_2;
        if !(-pole..=pole).contains(&latitude_rad) {
            return Err(EnvironmentError::LatitudeOutOfRange { latitude_rad });
        }
        Ok(Self { latitude_rad })
    }

    /// The latitude, rad.
    #[must_use]
    pub const fn latitude_rad(&self) -> f64 {
        self.latitude_rad
    }

    /// Normal gravity on the ellipsoid, Eq. (ATM-9).
    #[must_use]
    pub fn on_ellipsoid_m_s2(&self) -> f64 {
        let sin2 = sin_squared(self.latitude_rad);
        wgs84::EQUATORIAL_GRAVITY_M_S2 * (1.0 + wgs84::SOMIGLIANA_K * sin2)
            / (1.0 - wgs84::ECCENTRICITY_SQUARED * sin2).sqrt()
    }

    /// Normal gravity at `altitude_m` above the ellipsoid, Eq. (ATM-10).
    #[must_use]
    pub fn at_m_s2(&self, altitude_m: f64) -> f64 {
        let sin2 = sin_squared(self.latitude_rad);
        let a = wgs84::SEMI_MAJOR_AXIS_M;
        let f = wgs84::FLATTENING;
        let linear = 2.0 / a * (1.0 + f + wgs84::GRAVITY_RATIO_M - 2.0 * f * sin2) * altitude_m;
        let quadratic = 3.0 * altitude_m * altitude_m / (a * a);
        self.on_ellipsoid_m_s2() * (1.0 - linear + quadratic)
    }
}

fn sin_squared(angle_rad: f64) -> f64 {
    let sine = libm::sin(angle_rad);
    sine * sine
}

/// The gravity model of a run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Gravity {
    /// [`G0_M_S2`] everywhere.
    Standard,
    /// WGS-84 normal gravity at the run's latitude, varying with altitude.
    Wgs84(NormalGravity),
}

impl Gravity {
    /// The magnitude of gravity at `altitude_m`, m/s².
    #[must_use]
    pub fn magnitude_m_s2(&self, altitude_m: f64) -> f64 {
        match self {
            Self::Standard => G0_M_S2,
            Self::Wgs84(normal) => normal.at_m_s2(altitude_m),
        }
    }

    /// The gravitational acceleration at `altitude_m`, world axes: along +z, which points down.
    #[must_use]
    pub fn acceleration_w(&self, altitude_m: f64) -> Vec3<World> {
        Vec3::new(0.0, 0.0, self.magnitude_m_s2(altitude_m))
    }
}
