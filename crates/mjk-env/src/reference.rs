//! Values printed in the tables of *U.S. Standard Atmosphere, 1976* (NOAA, NASA, USAF), for
//! validating the atmosphere model (#15).
//!
//! Temperature, pressure and density come from Table I (geometric altitude, metric units),
//! speed of sound and dynamic viscosity from Table III, as printed: five significant figures,
//! pressure converted from millibars to pascals. Page numbers are the document's own; the NASA
//! scan (NTRS 19770009539) shows document page `n` as PDF page `n + 15`.

/// One row of the published tables, at a geometric altitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TableRow {
    /// Geometric altitude, m.
    pub altitude_m: f64,
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
    /// Pages of Table I and Table III holding the row.
    pub pages: (u32, u32),
}

/// The rows at 0, 1, 2, 5, 11 and 20 km.
pub const USSA_1976: [TableRow; 6] = [
    TableRow {
        altitude_m: 0.0,
        temperature_k: 288.150,
        pressure_pa: 101_325.0,
        density_kg_m3: 1.2250,
        speed_of_sound_m_s: 340.29,
        dynamic_viscosity_pa_s: 1.7894e-5,
        pages: (53, 101),
    },
    TableRow {
        altitude_m: 1000.0,
        temperature_k: 281.651,
        pressure_pa: 89_876.0,
        density_kg_m3: 1.1117,
        speed_of_sound_m_s: 336.43,
        dynamic_viscosity_pa_s: 1.7579e-5,
        pages: (53, 101),
    },
    TableRow {
        altitude_m: 2000.0,
        temperature_k: 275.154,
        pressure_pa: 79_501.0,
        density_kg_m3: 1.0066,
        speed_of_sound_m_s: 332.53,
        dynamic_viscosity_pa_s: 1.7260e-5,
        pages: (53, 101),
    },
    TableRow {
        altitude_m: 5000.0,
        temperature_k: 255.676,
        pressure_pa: 54_048.0,
        density_kg_m3: 0.736_43,
        speed_of_sound_m_s: 320.55,
        dynamic_viscosity_pa_s: 1.6282e-5,
        pages: (55, 103),
    },
    TableRow {
        altitude_m: 11_000.0,
        temperature_k: 216.774,
        pressure_pa: 22_699.0,
        density_kg_m3: 0.364_80,
        speed_of_sound_m_s: 295.15,
        dynamic_viscosity_pa_s: 1.4223e-5,
        pages: (59, 107),
    },
    TableRow {
        altitude_m: 20_000.0,
        temperature_k: 216.650,
        pressure_pa: 5_529.3,
        density_kg_m3: 0.088_910,
        speed_of_sound_m_s: 295.07,
        dynamic_viscosity_pa_s: 1.4216e-5,
        pages: (61, 109),
    },
];
