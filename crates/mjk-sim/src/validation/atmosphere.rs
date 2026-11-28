//! The atmosphere study of [#15](https://github.com/MatthewK84/MJKPhysics/issues/15): the model
//! beside the printed tables of *U.S. Standard Atmosphere, 1976* at 0, 1, 2, 5, 11 and 20 km.
//!
//! Each altitude has two rows: the model, then the 1976 table as printed. The last column is the
//! largest gap at that altitude, in units of the table value's fourth significant figure:
//! four-figure agreement means at most 0.5.

use mjk_env::reference::{TableRow, USSA_1976};
use mjk_env::{Air, Atmosphere, EnvironmentError};

/// One altitude of the study.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StudyRow {
    /// The model's air.
    pub model: Air,
    /// The published row.
    pub table: TableRow,
    /// The largest gap between model and table, in units of the table's fourth figure.
    pub worst_gap: f64,
}

/// The model at every published altitude.
///
/// # Errors
///
/// Only if a published altitude fell outside the model's range, which a test rules out.
pub fn atmosphere_study() -> Result<Vec<StudyRow>, EnvironmentError> {
    let atmosphere = Atmosphere::standard();
    let mut rows = Vec::with_capacity(USSA_1976.len());
    for table in USSA_1976 {
        let model = atmosphere.at(table.altitude_m)?;
        let worst_gap = pairs(&model, &table)
            .iter()
            .map(|(m, t)| fourth_figure_gap(*m, *t))
            .fold(0.0, f64::max);
        rows.push(StudyRow {
            model,
            table,
            worst_gap,
        });
    }
    Ok(rows)
}

/// The model's values beside the table's: temperature, pressure, density, speed of sound,
/// viscosity.
fn pairs(model: &Air, table: &TableRow) -> [(f64, f64); 5] {
    [
        (model.temperature_k, table.temperature_k),
        (model.pressure_pa, table.pressure_pa),
        (model.density_kg_m3, table.density_kg_m3),
        (model.speed_of_sound_m_s, table.speed_of_sound_m_s),
        (model.dynamic_viscosity_pa_s, table.dynamic_viscosity_pa_s),
    ]
}

/// `|model − table|` in units of the table value's fourth significant figure.
#[must_use]
pub fn fourth_figure_gap(model: f64, table: f64) -> f64 {
    (model - table).abs() / fourth_figure_unit(table)
}

/// One unit of the fourth significant figure of `value`.
fn fourth_figure_unit(value: f64) -> f64 {
    libm::pow(10.0, libm::floor(libm::log10(value.abs())) - 3.0)
}

/// `value` with `digits` significant figures, in plain decimal notation.
fn significant(value: f64, digits: i32) -> String {
    let exponent = libm::floor(libm::log10(value.abs()));
    let decimals = usize::try_from(i64::from(digits) - 1 - float_to_i64(exponent)).unwrap_or(0);
    format!("{value:.decimals$}")
}

/// A small whole number held in an `f64`, as an `i64`.
fn float_to_i64(value: f64) -> i64 {
    let mut whole = 0_i64;
    let mut remaining = value;
    // bound: |value| is a decimal exponent, below 400.
    for _ in 0..400 {
        if remaining >= 1.0 {
            whole += 1;
            remaining -= 1.0;
        } else if remaining <= -1.0 {
            whole -= 1;
            remaining += 1.0;
        } else {
            break;
        }
    }
    whole
}

/// Header of the study table.
const HEADER: &str = "| Z (m) | Source | T (K) | P (Pa) | ρ (kg/m³) | a (m/s) | μ (µPa·s) \
                      | Gap |\n|---:|---|---:|---:|---:|---:|---:|---:|\n";

/// The largest gap that still agrees to four significant figures, in units of the fourth.
pub const FOUR_FIGURES: f64 = 0.5;

/// True if the model agrees with the table to four significant figures at every altitude.
#[must_use]
pub fn passed(rows: &[StudyRow]) -> bool {
    rows.iter().all(|row| row.worst_gap <= FOUR_FIGURES)
}

/// The study as a Markdown table, as `mjk validate atmosphere` prints it and the book includes it.
#[must_use]
pub fn markdown(rows: &[StudyRow]) -> String {
    let lines: String = rows.iter().map(table_line).collect();
    format!("{HEADER}{lines}")
}

/// Scale factors to the table's units: viscosity in µPa·s.
const SCALES: [f64; 5] = [1.0, 1.0, 1.0, 1.0, 1e6];

/// Table I prints temperature to six figures; everything else is printed to five.
const PRINTED_DIGITS: [i32; 5] = [6, 5, 5, 5, 5];

fn table_line(row: &StudyRow) -> String {
    let values = pairs(&row.model, &row.table);
    let model: Vec<String> = values
        .iter()
        .zip(SCALES)
        .map(|((model, _), scale)| significant(model * scale, 6))
        .collect();
    let table: Vec<String> = values
        .iter()
        .zip(SCALES)
        .zip(PRINTED_DIGITS)
        .map(|(((_, table), scale), digits)| significant(table * scale, digits))
        .collect();
    format!(
        "| {:.0} | Model | {} | {:.2} |\n| | 1976 | {} | |\n",
        row.table.altitude_m,
        model.join(" | "),
        row.worst_gap,
        table.join(" | ")
    )
}
