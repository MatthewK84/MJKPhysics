//! Units ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)): reading the unit of a
//! parameter and converting its value to SI.
//!
//! A unit is a product of symbols, each with an optional integer power, divided by others:
//! `kg m^2`, `N m s/rad`, `rpm/V`, `mAh`, `1/s`. Symbols are separated by spaces, `·` or `*`;
//! powers are written `^2`, `^-1`, `²` or `³`; `1` is a dimensionless value. Only the symbols in
//! [`SYMBOLS`] are known, each with its definition in SI. There is no general prefix rule, so
//! `min` is a minute and never a milli-inch.
//!
//! Every quantity has a dimension: powers of length, mass, time, current and angle. Angle is its
//! own dimension, although SI counts it as dimensionless, so that `rpm` cannot be mistaken for
//! `Hz`.

use core::f64::consts::PI;
use std::fmt;

/// Powers of length, mass, time, current and angle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Dimension(pub [i8; 5]);

impl Dimension {
    /// A pure number.
    pub const NONE: Self = Self([0; 5]);

    const fn times(self, other: Self) -> Self {
        let [length, mass, time, current, angle] = self.0;
        let [by_length, by_mass, by_time, by_current, by_angle] = other.0;
        Self([
            length + by_length,
            mass + by_mass,
            time + by_time,
            current + by_current,
            angle + by_angle,
        ])
    }

    const fn power(self, exponent: i8) -> Self {
        let [length, mass, time, current, angle] = self.0;
        Self([
            length * exponent,
            mass * exponent,
            time * exponent,
            current * exponent,
            angle * exponent,
        ])
    }
}

impl fmt::Display for Dimension {
    /// The dimension in SI base units, such as `kg·m²·s⁻²`, or `1`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let names = ["m", "kg", "s", "A", "rad"];
        let parts: Vec<String> = [1, 0, 2, 3, 4]
            .into_iter()
            .filter_map(|axis| {
                let power = *self.0.get(axis)?;
                let name = names.get(axis)?;
                (power != 0).then(|| format!("{name}{}", superscript(power)))
            })
            .collect();
        if parts.is_empty() {
            return f.write_str("1");
        }
        f.write_str(&parts.join("·"))
    }
}

/// `power` as superscript digits, or nothing for 1.
fn superscript(power: i8) -> String {
    if power == 1 {
        return String::new();
    }
    power
        .to_string()
        .chars()
        .map(|c| match c {
            '-' => '⁻',
            '0' => '⁰',
            '1' => '¹',
            '2' => '²',
            '3' => '³',
            '4' => '⁴',
            '5' => '⁵',
            '6' => '⁶',
            '7' => '⁷',
            '8' => '⁸',
            _ => '⁹',
        })
        .collect()
}

const LENGTH: Dimension = Dimension([1, 0, 0, 0, 0]);
const MASS: Dimension = Dimension([0, 1, 0, 0, 0]);
const TIME: Dimension = Dimension([0, 0, 1, 0, 0]);
const CURRENT: Dimension = Dimension([0, 0, 0, 1, 0]);
const ANGLE: Dimension = Dimension([0, 0, 0, 0, 1]);
const FREQUENCY: Dimension = Dimension([0, 0, -1, 0, 0]);
const FORCE: Dimension = Dimension([1, 1, -2, 0, 0]);
const ENERGY: Dimension = Dimension([2, 1, -2, 0, 0]);
const POWER: Dimension = Dimension([2, 1, -3, 0, 0]);
const VOLTAGE: Dimension = Dimension([2, 1, -3, -1, 0]);
const RESISTANCE: Dimension = Dimension([2, 1, -3, -2, 0]);
const CHARGE: Dimension = Dimension([0, 0, 1, 1, 0]);
const ANGULAR_RATE: Dimension = Dimension([0, 0, -1, 0, 1]);

/// A unit symbol and its value in SI.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Symbol {
    /// As written in a file.
    pub name: &'static str,
    /// One of it, in SI units.
    pub factor: f64,
    /// Its dimension.
    pub dimension: Dimension,
}

const fn symbol(name: &'static str, factor: f64, dimension: Dimension) -> Symbol {
    Symbol {
        name,
        factor,
        dimension,
    }
}

/// Standard gravity, m/s², which defines the gram-force and its relatives.
const G0: f64 = 9.806_65;

/// Every known symbol. Each factor is the symbol's definition; the international inch and pound
/// and the standard gravity behind the gram-force are exact by definition.
pub const SYMBOLS: [Symbol; 37] = [
    symbol("1", 1.0, Dimension::NONE),
    symbol("m", 1.0, LENGTH),
    symbol("cm", 1e-2, LENGTH),
    symbol("mm", 1e-3, LENGTH),
    symbol("km", 1e3, LENGTH),
    symbol("in", 0.0254, LENGTH),
    symbol("ft", 0.3048, LENGTH),
    symbol("kg", 1.0, MASS),
    symbol("g", 1e-3, MASS),
    symbol("lb", 0.453_592_37, MASS),
    symbol("oz", 0.028_349_523_125, MASS),
    symbol("s", 1.0, TIME),
    symbol("ms", 1e-3, TIME),
    symbol("us", 1e-6, TIME),
    symbol("µs", 1e-6, TIME),
    symbol("min", 60.0, TIME),
    symbol("h", 3600.0, TIME),
    symbol("A", 1.0, CURRENT),
    symbol("mA", 1e-3, CURRENT),
    symbol("rad", 1.0, ANGLE),
    symbol("deg", PI / 180.0, ANGLE),
    symbol("rev", 2.0 * PI, ANGLE),
    symbol("rpm", 2.0 * PI / 60.0, ANGULAR_RATE),
    symbol("Hz", 1.0, FREQUENCY),
    symbol("N", 1.0, FORCE),
    symbol("gf", G0 * 1e-3, FORCE),
    symbol("kgf", G0, FORCE),
    symbol("lbf", G0 * 0.453_592_37, FORCE),
    symbol("J", 1.0, ENERGY),
    symbol("W", 1.0, POWER),
    symbol("V", 1.0, VOLTAGE),
    symbol("mV", 1e-3, VOLTAGE),
    symbol("ohm", 1.0, RESISTANCE),
    symbol("mohm", 1e-3, RESISTANCE),
    symbol("Ω", 1.0, RESISTANCE),
    symbol("Ah", 3600.0, CHARGE),
    symbol("mAh", 3.6, CHARGE),
];

/// A parsed unit: its size in SI units, and its dimension.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Unit {
    /// One of the unit, in SI units.
    pub factor: f64,
    /// The dimension.
    pub dimension: Dimension,
}

impl Unit {
    const ONE: Self = Self {
        factor: 1.0,
        dimension: Dimension::NONE,
    };

    const fn times(self, other: Self) -> Self {
        Self {
            factor: self.factor * other.factor,
            dimension: self.dimension.times(other.dimension),
        }
    }

    fn power(self, exponent: i8) -> Self {
        let magnitude = (0..exponent.unsigned_abs()).fold(1.0, |product, _| product * self.factor);
        let factor = if exponent.is_negative() {
            1.0 / magnitude
        } else {
            magnitude
        };
        Self {
            factor,
            dimension: self.dimension.power(exponent),
        }
    }
}

/// Why a unit could not be read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UnitError {
    /// A symbol that is not in [`SYMBOLS`].
    Unknown(String),
    /// A power that is not a nonzero integer.
    Power(String),
    /// Nothing between separators.
    Empty,
}

impl fmt::Display for UnitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unknown(symbol) => write!(f, "unknown unit `{symbol}`"),
            Self::Power(text) => write!(
                f,
                "`{text}`: a power is a nonzero integer, as in m^2 or s^-1"
            ),
            Self::Empty => f.write_str("an empty unit; write 1 for a pure number"),
        }
    }
}

/// Reads a unit.
///
/// # Errors
///
/// A [`UnitError`] for an unknown symbol, a malformed power or an empty unit.
pub fn parse_unit(text: &str) -> Result<Unit, UnitError> {
    let mut parts = text.split('/');
    let numerator = product(parts.next().unwrap_or_default())?;
    parts.try_fold(numerator, |unit, part| {
        Ok(unit.times(product(part)?.power(-1)))
    })
}

fn product(text: &str) -> Result<Unit, UnitError> {
    let factors: Vec<&str> = text
        .split([' ', '·', '*'])
        .filter(|factor| !factor.is_empty())
        .collect();
    if factors.is_empty() {
        return Err(UnitError::Empty);
    }
    factors
        .into_iter()
        .try_fold(Unit::ONE, |unit, factor| Ok(unit.times(powered(factor)?)))
}

fn powered(text: &str) -> Result<Unit, UnitError> {
    let (name, power) = split_power(text)?;
    let found = SYMBOLS
        .iter()
        .find(|symbol| symbol.name == name)
        .ok_or_else(|| UnitError::Unknown(name.to_owned()))?;
    let unit = Unit {
        factor: found.factor,
        dimension: found.dimension,
    };
    Ok(unit.power(power))
}

fn split_power(text: &str) -> Result<(&str, i8), UnitError> {
    if let Some(name) = text.strip_suffix('²') {
        return Ok((name, 2));
    }
    if let Some(name) = text.strip_suffix('³') {
        return Ok((name, 3));
    }
    let Some((name, power)) = text.split_once('^') else {
        return Ok((text, 1));
    };
    power
        .parse::<i8>()
        .ok()
        .filter(|power| *power != 0 && power.unsigned_abs() <= 9)
        .map(|power| (name, power))
        .ok_or_else(|| UnitError::Power(text.to_owned()))
}

/// A physical quantity a parameter must have: its dimension, and how to name it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Quantity {
    /// Its name in messages, such as `"a mass"`.
    pub name: &'static str,
    /// Its dimension.
    pub dimension: Dimension,
    /// Its SI unit as the book writes it, such as `"kg·m²"`.
    pub si: &'static str,
    /// Units a file might use, for messages.
    pub examples: &'static str,
}

const fn quantity(
    name: &'static str,
    dimension: Dimension,
    si: &'static str,
    examples: &'static str,
) -> Quantity {
    Quantity {
        name,
        dimension,
        si,
        examples,
    }
}

/// The quantities airframe parameters have.
pub mod quantities {
    use super::{
        ANGLE, ANGULAR_RATE, CHARGE, CURRENT, Dimension, FORCE, FREQUENCY, LENGTH, MASS, Quantity,
        RESISTANCE, TIME, VOLTAGE, quantity,
    };

    /// Mass, kg.
    pub const MASS_KG: Quantity = quantity("a mass", MASS, "kg", "kg or g");
    /// Length, m.
    pub const LENGTH_M: Quantity = quantity("a length", LENGTH, "m", "m, mm or in");
    /// Area, m².
    pub const AREA_M2: Quantity =
        quantity("an area", Dimension([2, 0, 0, 0, 0]), "m²", "m^2 or cm^2");
    /// Moment of inertia, kg·m².
    pub const INERTIA_KG_M2: Quantity = quantity(
        "a moment of inertia",
        Dimension([2, 1, 0, 0, 0]),
        "kg·m²",
        "kg m^2 or g cm^2",
    );
    /// Force, N.
    pub const FORCE_N: Quantity = quantity("a force", FORCE, "N", "N, gf or kgf");
    /// Time, s.
    pub const TIME_S: Quantity = quantity("a time", TIME, "s", "s or ms");
    /// Rate, 1/s.
    pub const RATE_PER_S: Quantity = quantity("a rate", FREQUENCY, "1/s", "1/s");
    /// Angle, rad.
    pub const ANGLE_RAD: Quantity = quantity("an angle", ANGLE, "rad", "deg or rad");
    /// Angular rate, rad/s.
    pub const ANGULAR_RATE_RAD_S: Quantity =
        quantity("an angular rate", ANGULAR_RATE, "rad/s", "rpm or rad/s");
    /// Current, A.
    pub const CURRENT_A: Quantity = quantity("a current", CURRENT, "A", "A or mA");
    /// Resistance, Ω.
    pub const RESISTANCE_OHM: Quantity = quantity("a resistance", RESISTANCE, "Ω", "ohm or mohm");
    /// Charge, A·s.
    pub const CHARGE_C: Quantity = quantity("a charge", CHARGE, "A·s", "mAh or Ah");
    /// Motor speed constant, rad/(s·V).
    pub const SPEED_CONSTANT: Quantity = quantity(
        "a speed constant",
        ANGULAR_RATE.times(VOLTAGE.power(-1)),
        "rad/(s·V)",
        "rpm/V",
    );
    /// A pure number.
    pub const PURE: Quantity = quantity("a pure number", Dimension::NONE, "1", "1");
}
