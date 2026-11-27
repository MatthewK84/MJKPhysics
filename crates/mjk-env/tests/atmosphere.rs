//! The U.S. Standard Atmosphere 1976 (#15): the published tables, the layer boundary, the
//! temperature offset and the model's range.

use mjk_env::atmosphere::{
    EARTH_RADIUS_M, MAX_ALTITUDE_M, MIN_ALTITUDE_M, SEA_LEVEL_PRESSURE_PA,
    TROPOPAUSE_GEOPOTENTIAL_M, geopotential_altitude_m,
};
use mjk_env::reference::{TableRow, USSA_1976};
use mjk_env::{Air, Atmosphere, EnvironmentError, G0_M_S2};

/// `model` agrees with the printed `table` value to four significant figures: within half a unit
/// of its fourth figure. Comparing rounded strings would fail on double rounding, as for 320.5455
/// against a printed 320.55.
fn agrees_to_four_figures(model: f64, table: f64) -> bool {
    let exponent = libm::floor(libm::log10(table.abs())) - 3.0;
    (model - table).abs() <= 0.5 * libm::pow(10.0, exponent)
}

/// The model's values beside the table's, named.
fn pairs(air: &Air, row: &TableRow) -> [(&'static str, f64, f64); 5] {
    [
        ("temperature", air.temperature_k, row.temperature_k),
        ("pressure", air.pressure_pa, row.pressure_pa),
        ("density", air.density_kg_m3, row.density_kg_m3),
        (
            "speed of sound",
            air.speed_of_sound_m_s,
            row.speed_of_sound_m_s,
        ),
        (
            "viscosity",
            air.dynamic_viscosity_pa_s,
            row.dynamic_viscosity_pa_s,
        ),
    ]
}

fn standard_at(altitude_m: f64) -> Option<Air> {
    Atmosphere::standard().at(altitude_m).ok()
}

// ---------------------------------------------------------------- the published tables

#[test]
fn matches_the_1976_tables_to_four_significant_figures() {
    for row in USSA_1976 {
        let air = standard_at(row.altitude_m).expect("in range");
        for (name, model, table) in pairs(&air, &row) {
            assert!(
                agrees_to_four_figures(model, table),
                "{name} at {} m: model {model}, table {table}",
                row.altitude_m
            );
            // Closer still: the tables print five figures and the model reproduces them.
            assert!(
                (model - table).abs() / table < 5e-5,
                "{name} at {} m",
                row.altitude_m
            );
        }
    }
}

#[test]
fn the_comparison_rejects_a_value_off_in_the_fourth_figure() {
    assert!(agrees_to_four_figures(320.5455, 320.55));
    assert!(agrees_to_four_figures(1.2250, 1.2250));
    assert!(!agrees_to_four_figures(1.2256, 1.2250));
    assert!(!agrees_to_four_figures(1.7900e-5, 1.7894e-5));
}

#[test]
fn geopotential_altitude_matches_the_tables() {
    // Table I prints H beside Z, rounded to the meter.
    for (z, h) in [
        (1000.0, 1000.0),
        (2000.0, 1999.0),
        (5000.0, 4996.0),
        (11_000.0, 10_981.0),
        (20_000.0, 19_937.0),
    ] {
        assert!((geopotential_altitude_m(z) - h).abs() <= 0.5, "{z} m");
    }
    assert!(geopotential_altitude_m(0.0).abs() < 1e-12);
    assert!((geopotential_altitude_m(-EARTH_RADIUS_M / 2.0) + EARTH_RADIUS_M).abs() < 1e-6);
}

// ---------------------------------------------------------------- layers

#[test]
fn temperature_and_pressure_are_continuous_at_the_tropopause() {
    // The geometric altitude of 11 km geopotential.
    let z =
        EARTH_RADIUS_M * TROPOPAUSE_GEOPOTENTIAL_M / (EARTH_RADIUS_M - TROPOPAUSE_GEOPOTENTIAL_M);
    for offset_k in [0.0, 15.0, -20.0] {
        let atmosphere = Atmosphere::new(offset_k).expect("valid offset");
        let below = atmosphere.at(z - 1e-6).expect("in range");
        let above = atmosphere.at(z + 1e-6).expect("in range");
        assert!(
            (below.temperature_k - (216.65 + offset_k)).abs() < 1e-8,
            "{offset_k} K"
        );
        assert!(
            (above.temperature_k - (216.65 + offset_k)).abs() < 1e-12,
            "{offset_k} K"
        );
        let jump = (below.pressure_pa - above.pressure_pa).abs() / above.pressure_pa;
        assert!(jump < 1e-9, "{offset_k} K: pressure jumps by {jump}");
    }
    let below = standard_at(z - 1e-6).expect("in range");
    // Cooling stops at the tropopause: the troposphere's temperature falls with altitude, the
    // layer above holds it constant.
    assert!(standard_at(z - 100.0).expect("in range").temperature_k > below.temperature_k);
    assert!((standard_at(z + 100.0).expect("in range").temperature_k - 216.65).abs() < 1e-12);
}

#[test]
fn the_air_is_in_hydrostatic_balance_with_and_without_an_offset() {
    // dP/dz = −ρ g(z), with g(z) = g₀ (r₀ / (r₀ + z))², the gravity geopotential altitude assumes.
    for offset_k in [0.0, 15.0, -20.0] {
        let atmosphere = Atmosphere::new(offset_k).expect("valid offset");
        for z in [-4000.0, 0.0, 3000.0, 10_000.0, 12_000.0, 19_000.0] {
            let step = 1.0;
            let up = atmosphere.at(z + step).expect("in range").pressure_pa;
            let down = atmosphere.at(z - step).expect("in range").pressure_pa;
            let slope = (up - down) / (2.0 * step);
            let ratio = EARTH_RADIUS_M / (EARTH_RADIUS_M + z);
            let expected =
                -atmosphere.at(z).expect("in range").density_kg_m3 * G0_M_S2 * ratio * ratio;
            assert!(
                (slope - expected).abs() / expected.abs() < 1e-6,
                "{offset_k} K, {z} m"
            );
        }
    }
}

// ---------------------------------------------------------------- temperature offset

#[test]
fn an_offset_shifts_temperature_and_keeps_sea_level_pressure() {
    let standard = Atmosphere::standard();
    let hot = Atmosphere::new(15.0).expect("valid");
    assert_eq!(Atmosphere::new(0.0), Ok(standard));
    assert!((hot.temperature_offset_k() - 15.0).abs() < 1e-15);
    for z in [-5000.0, 0.0, 2000.0, 11_000.0, 20_000.0] {
        let warm = hot.at(z).expect("in range");
        let base = standard.at(z).expect("in range");
        assert!(
            (warm.temperature_k - base.temperature_k - 15.0).abs() < 1e-9,
            "{z} m"
        );
        // A warm column holds more air aloft: pressure is higher at every altitude above sea level.
        if z > 0.0 {
            assert!(warm.pressure_pa > base.pressure_pa, "{z} m");
        }
    }
    // Low down the warmth wins and the air is thinner; above about 9 km the extra pressure wins.
    for (z, thinner) in [
        (2000.0, true),
        (5000.0, true),
        (11_000.0, false),
        (20_000.0, false),
    ] {
        let warm = hot.at(z).expect("in range").density_kg_m3;
        let base = standard.at(z).expect("in range").density_kg_m3;
        assert_eq!(warm < base, thinner, "{z} m");
    }
    let sea_level = hot.at(0.0).expect("in range");
    assert!((sea_level.pressure_pa - SEA_LEVEL_PRESSURE_PA).abs() < 1e-9);
}

#[test]
fn an_offset_that_is_not_physical_is_rejected() {
    for offset_k in [f64::NAN, f64::INFINITY, -216.65, -300.0] {
        assert_eq!(
            Atmosphere::new(offset_k)
                .map_err(|e| matches!(e, EnvironmentError::InvalidTemperatureOffset { .. })),
            Err(true),
            "{offset_k}"
        );
    }
    assert!(Atmosphere::new(-216.6).is_ok());
}

// ---------------------------------------------------------------- range

#[test]
fn the_model_covers_exactly_its_range() {
    assert!(Atmosphere::standard().at(MIN_ALTITUDE_M).is_ok());
    assert!(Atmosphere::standard().at(MAX_ALTITUDE_M).is_ok());
    for z in [
        MIN_ALTITUDE_M - 0.001,
        MAX_ALTITUDE_M + 0.001,
        f64::NAN,
        f64::INFINITY,
    ] {
        assert_eq!(
            Atmosphere::standard()
                .at(z)
                .map_err(|e| matches!(e, EnvironmentError::AltitudeOutOfRange { .. })),
            Err(true),
            "{z}"
        );
    }
}
