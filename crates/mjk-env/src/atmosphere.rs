//! The U.S. Standard Atmosphere, 1976, from −5 km to 20 km (#15).
//!
//! Below 20 km the standard has two layers in geopotential altitude: a troposphere cooling at
//! 6.5 K/km up to 11 km, then an isothermal layer at 216.65 K. Temperature and pressure follow in
//! closed form, Eqs. (ATM-3) to (ATM-5); density, the speed of sound and the viscosity follow from
//! them, Eqs. (ATM-6) to (ATM-8). The constants are the standard's own, so the results reproduce
//! its tables.
//!
//! A temperature offset shifts the whole temperature profile, as on a hot or cold day. Sea-level
//! pressure stays at 101 325 Pa and pressure follows from hydrostatic balance with the shifted
//! temperature, so pressure, density and temperature stay consistent with `dP = −ρ g dh`.

use crate::error::EnvironmentError;
use crate::gravity::G0_M_S2;

/// The universal gas constant as the 1976 standard defines it, J/(mol·K). The standard's tables
/// were computed with this value, not the later CODATA value 8.314 462 618.
pub const GAS_CONSTANT_J_MOL_K: f64 = 8.314_32;

/// The mean molar mass of air below 80 km, kg/mol.
pub const MOLAR_MASS_KG_MOL: f64 = 0.028_964_4;

/// The earth radius the standard uses to convert geometric to geopotential altitude, m.
pub const EARTH_RADIUS_M: f64 = 6_356_766.0;

/// The ratio of specific heats of air.
pub const HEAT_CAPACITY_RATIO: f64 = 1.4;

/// Sutherland's constant β for air, kg/(m·s·K^½).
pub const SUTHERLAND_BETA: f64 = 1.458e-6;

/// Sutherland's temperature S for air, K.
pub const SUTHERLAND_TEMPERATURE_K: f64 = 110.4;

/// Sea-level temperature, K.
pub const SEA_LEVEL_TEMPERATURE_K: f64 = 288.15;

/// Sea-level pressure, Pa.
pub const SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;

/// The troposphere's temperature gradient, K/m of geopotential altitude.
pub const LAPSE_RATE_K_M: f64 = -0.0065;

/// The geopotential altitude of the tropopause, where the isothermal layer begins, m.
pub const TROPOPAUSE_GEOPOTENTIAL_M: f64 = 11_000.0;

/// The lowest geometric altitude the model covers, m: the bottom of the standard's tables.
pub const MIN_ALTITUDE_M: f64 = -5_000.0;

/// The highest geometric altitude the model covers, m. It lies inside the isothermal layer,
/// which ends at 20 km geopotential altitude (20 063 m geometric).
pub const MAX_ALTITUDE_M: f64 = 20_000.0;

/// The state of the air at one altitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Air {
    /// Temperature, K.
    pub temperature_k: f64,
    /// Pressure, Pa.
    pub pressure_pa: f64,
    /// Density, kg/m³.
    pub density_kg_m3: f64,
    /// Speed of sound, m/s.
    pub speed_of_sound_m_s: f64,
    /// Dynamic viscosity, Pa·s.
    pub dynamic_viscosity_pa_s: f64,
}

/// The standard atmosphere, optionally shifted by a temperature offset.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Atmosphere {
    temperature_offset_k: f64,
    tropopause_pressure_pa: f64,
}

impl Atmosphere {
    /// The 1976 standard atmosphere itself.
    #[must_use]
    pub fn standard() -> Self {
        Self::with_offset(0.0)
    }

    /// The standard atmosphere with every temperature shifted by `temperature_offset_k`, as for
    /// "ISA + 15".
    ///
    /// # Errors
    ///
    /// [`EnvironmentError::InvalidTemperatureOffset`] for a non-finite offset, or one that cools
    /// the isothermal layer to absolute zero or below.
    pub fn new(temperature_offset_k: f64) -> Result<Self, EnvironmentError> {
        let coldest_k = tropopause_temperature_k(temperature_offset_k);
        if !temperature_offset_k.is_finite() || coldest_k <= 0.0 {
            return Err(EnvironmentError::InvalidTemperatureOffset {
                temperature_offset_k,
            });
        }
        Ok(Self::with_offset(temperature_offset_k))
    }

    fn with_offset(temperature_offset_k: f64) -> Self {
        let sea_level_k = SEA_LEVEL_TEMPERATURE_K + temperature_offset_k;
        let tropopause_k = tropopause_temperature_k(temperature_offset_k);
        Self {
            temperature_offset_k,
            tropopause_pressure_pa: gradient_pressure_pa(sea_level_k, tropopause_k),
        }
    }

    /// The temperature offset, K.
    #[must_use]
    pub const fn temperature_offset_k(&self) -> f64 {
        self.temperature_offset_k
    }

    /// The air at a geometric altitude above mean sea level.
    ///
    /// # Errors
    ///
    /// [`EnvironmentError::AltitudeOutOfRange`] outside [`MIN_ALTITUDE_M`] to [`MAX_ALTITUDE_M`],
    /// or for a non-finite altitude. The model never extrapolates.
    pub fn at(&self, altitude_m: f64) -> Result<Air, EnvironmentError> {
        if !(MIN_ALTITUDE_M..=MAX_ALTITUDE_M).contains(&altitude_m) {
            return Err(EnvironmentError::AltitudeOutOfRange { altitude_m });
        }
        let (temperature_k, pressure_pa) =
            self.temperature_and_pressure(geopotential_altitude_m(altitude_m));
        Ok(air(temperature_k, pressure_pa))
    }

    /// Temperature and pressure at a geopotential altitude, Eqs. (ATM-3) to (ATM-5).
    fn temperature_and_pressure(&self, geopotential_m: f64) -> (f64, f64) {
        if geopotential_m <= TROPOPAUSE_GEOPOTENTIAL_M {
            let sea_level_k = SEA_LEVEL_TEMPERATURE_K + self.temperature_offset_k;
            let temperature_k = sea_level_k + LAPSE_RATE_K_M * geopotential_m;
            return (
                temperature_k,
                gradient_pressure_pa(sea_level_k, temperature_k),
            );
        }
        let temperature_k = tropopause_temperature_k(self.temperature_offset_k);
        let height_m = geopotential_m - TROPOPAUSE_GEOPOTENTIAL_M;
        let exponent =
            -G0_M_S2 * MOLAR_MASS_KG_MOL * height_m / (GAS_CONSTANT_J_MOL_K * temperature_k);
        (
            temperature_k,
            self.tropopause_pressure_pa * libm::exp(exponent),
        )
    }
}

/// Geopotential altitude from geometric altitude, Eq. (ATM-2): the height in a uniform gravity
/// field of `g₀` with the same potential energy.
#[must_use]
pub fn geopotential_altitude_m(altitude_m: f64) -> f64 {
    EARTH_RADIUS_M * altitude_m / (EARTH_RADIUS_M + altitude_m)
}

/// The temperature of the isothermal layer, K.
fn tropopause_temperature_k(temperature_offset_k: f64) -> f64 {
    SEA_LEVEL_TEMPERATURE_K + temperature_offset_k + LAPSE_RATE_K_M * TROPOPAUSE_GEOPOTENTIAL_M
}

/// Pressure in the troposphere where the temperature is `temperature_k`, Eq. (ATM-4).
fn gradient_pressure_pa(sea_level_k: f64, temperature_k: f64) -> f64 {
    let exponent = -G0_M_S2 * MOLAR_MASS_KG_MOL / (GAS_CONSTANT_J_MOL_K * LAPSE_RATE_K_M);
    SEA_LEVEL_PRESSURE_PA * libm::pow(temperature_k / sea_level_k, exponent)
}

/// Density, speed of sound and viscosity from temperature and pressure, Eqs. (ATM-6) to (ATM-8).
fn air(temperature_k: f64, pressure_pa: f64) -> Air {
    Air {
        temperature_k,
        pressure_pa,
        density_kg_m3: pressure_pa * MOLAR_MASS_KG_MOL / (GAS_CONSTANT_J_MOL_K * temperature_k),
        speed_of_sound_m_s: (HEAT_CAPACITY_RATIO * GAS_CONSTANT_J_MOL_K * temperature_k
            / MOLAR_MASS_KG_MOL)
            .sqrt(),
        dynamic_viscosity_pa_s: SUTHERLAND_BETA * temperature_k * temperature_k.sqrt()
            / (temperature_k + SUTHERLAND_TEMPERATURE_K),
    }
}
