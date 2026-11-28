//! Parameters ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)): every number in an
//! airframe file is a table `{ value, unit, source, uncertainty }`.
//!
//! The unit is required: a bare number, or a table without `unit`, is an error naming the field.
//! The source is the key of an entry in the file's `[sources]` table; without one the number
//! loads with a warning. The uncertainty, if given, is in the same unit as the value.
//!
//! A [`Resolver`] reads each parameter once: it checks the unit's dimension against the quantity
//! the field needs, converts the value to SI, checks the source, and records the parameter for
//! the airframe's generated page.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::marker::PhantomData;

use mjk_core::{EngineError, FieldPath};
use serde::de::value::MapAccessDeserializer;
use serde::de::{self, DeserializeOwned, MapAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize};

use super::sources::Source;
use super::units::{Quantity, parse_unit};

/// What a file says when a number has no unit.
pub const MISSING_UNIT: &str = "missing `unit`: every number in an airframe file carries one, \
                                as in { value = 1.2, unit = \"kg\", source = \"maker\" }";

/// A number, vector or matrix with its unit, source and uncertainty.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Parameter<T> {
    /// The value, in `unit`.
    pub value: T,
    /// The unit, such as `"g"` or `"rpm/V"`.
    pub unit: String,
    /// The key of the entry in `[sources]` this value comes from.
    pub source: Option<String>,
    /// The uncertainty, in `unit`.
    pub uncertainty: Option<T>,
}

/// The table as written, before the unit is required.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Table<T> {
    value: T,
    unit: Option<String>,
    source: Option<String>,
    uncertainty: Option<T>,
}

impl<'de, T: DeserializeOwned> Deserialize<'de> for Parameter<T> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(ParameterVisitor(PhantomData))
    }
}

/// Reads a parameter table, and turns anything else into the missing-unit error.
struct ParameterVisitor<T>(PhantomData<T>);

impl<'de, T: DeserializeOwned> Visitor<'de> for ParameterVisitor<T> {
    type Value = Parameter<T>;

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a table { value, unit, source, uncertainty }")
    }

    fn visit_map<A: MapAccess<'de>>(self, map: A) -> Result<Self::Value, A::Error> {
        let table = Table::<T>::deserialize(MapAccessDeserializer::new(map))?;
        let unit = table.unit.ok_or_else(|| de::Error::custom(MISSING_UNIT))?;
        Ok(Parameter {
            value: table.value,
            unit,
            source: table.source,
            uncertainty: table.uncertainty,
        })
    }

    fn visit_f64<E: de::Error>(self, _: f64) -> Result<Self::Value, E> {
        Err(E::custom(MISSING_UNIT))
    }

    fn visit_i64<E: de::Error>(self, _: i64) -> Result<Self::Value, E> {
        Err(E::custom(MISSING_UNIT))
    }

    fn visit_u64<E: de::Error>(self, _: u64) -> Result<Self::Value, E> {
        Err(E::custom(MISSING_UNIT))
    }

    fn visit_seq<A: de::SeqAccess<'de>>(self, _: A) -> Result<Self::Value, A::Error> {
        Err(de::Error::custom(MISSING_UNIT))
    }
}

/// A value a parameter can hold: a number, a vector or a matrix.
pub trait Value: Copy {
    /// The entries, row by row.
    fn entries(&self) -> Vec<f64>;
    /// The value with every entry mapped by `f`.
    #[must_use]
    fn map(self, f: impl Fn(f64) -> f64) -> Self;
    /// The path of entry `index`, counted row by row, below `path`.
    fn entry(path: &FieldPath, index: usize) -> FieldPath;
    /// The value with each entry written in `format`.
    fn shown(&self, format: Format) -> String;
}

impl Value for f64 {
    fn entries(&self) -> Vec<f64> {
        vec![*self]
    }

    fn map(self, f: impl Fn(f64) -> f64) -> Self {
        f(self)
    }

    fn entry(path: &FieldPath, _: usize) -> FieldPath {
        path.clone()
    }

    fn shown(&self, format: Format) -> String {
        format.number(*self)
    }
}

impl Value for [f64; 3] {
    fn entries(&self) -> Vec<f64> {
        self.to_vec()
    }

    fn map(self, f: impl Fn(f64) -> f64) -> Self {
        self.map(f)
    }

    fn entry(path: &FieldPath, index: usize) -> FieldPath {
        path.index(index)
    }

    fn shown(&self, format: Format) -> String {
        let parts: Vec<String> = self.iter().map(|x| format.number(*x)).collect();
        format!("[{}]", parts.join(", "))
    }
}

impl Value for [[f64; 3]; 3] {
    fn entries(&self) -> Vec<f64> {
        self.iter().flatten().copied().collect()
    }

    fn map(self, f: impl Fn(f64) -> f64) -> Self {
        self.map(|row| row.map(&f))
    }

    fn entry(path: &FieldPath, index: usize) -> FieldPath {
        path.index(index / 3).index(index % 3)
    }

    fn shown(&self, format: Format) -> String {
        let rows: Vec<String> = self.iter().map(|row| row.shown(format)).collect();
        format!("[{}]", rows.join(", "))
    }
}

/// How a number is written on an airframe's page.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// As written: [`plain`].
    Plain,
    /// To six significant figures: [`significant`].
    Significant,
}

impl Format {
    /// `value` in this format.
    #[must_use]
    pub fn number(self, value: f64) -> String {
        match self {
            Self::Plain => plain(value),
            Self::Significant => significant(value),
        }
    }
}

/// A number as written: the shortest form that reads back to the same value.
#[must_use]
pub fn plain(value: f64) -> String {
    format!("{value}")
}

/// A number to six significant figures, without trailing zeros, with an exponent below 10⁻⁴ and
/// from 10⁶ on. Zero and non-finite values have no exponent and come through unchanged.
#[must_use]
pub fn significant(value: f64) -> String {
    let scientific = format!("{value:.5e}");
    let (mantissa, exponent) = scientific.split_once('e').unwrap_or((&scientific, "0"));
    let power: i32 = exponent.parse().unwrap_or(0);
    if (-4..6).contains(&power) {
        let decimals = usize::try_from(5 - power).unwrap_or(0);
        return trim_zeros(&format!("{value:.decimals$}"));
    }
    format!("{}e{power}", trim_zeros(mantissa))
}

/// A value and its SI unit, or the value alone if it is a pure number.
fn with_unit(value: &str, unit: &str) -> String {
    if unit == "1" {
        return value.to_owned();
    }
    format!("{value} {unit}")
}

fn trim_zeros(text: &str) -> String {
    if !text.contains('.') {
        return text.to_owned();
    }
    text.trim_end_matches('0').trim_end_matches('.').to_owned()
}

/// One parameter as the airframe's page lists it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParameterRecord {
    /// The parameter's field.
    pub path: FieldPath,
    /// The value as written.
    pub value: String,
    /// The unit as written.
    pub unit: String,
    /// The value in SI units, with its unit.
    pub si: String,
    /// The uncertainty as written, in `unit`.
    pub uncertainty: Option<String>,
    /// The key of its source.
    pub source: Option<String>,
}

/// Something worth fixing in a file that still loads, such as a number without a source.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConfigWarning {
    /// The field.
    pub path: FieldPath,
    /// What is worth fixing.
    pub message: String,
}

impl fmt::Display for ConfigWarning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.path, self.message)
    }
}

/// Reads parameters into SI values, recording each and collecting warnings.
pub struct Resolver<'a> {
    document: &'a str,
    sources: &'a BTreeMap<String, Source>,
    cited: BTreeSet<String>,
    records: Vec<ParameterRecord>,
    warnings: Vec<ConfigWarning>,
}

impl<'a> Resolver<'a> {
    /// A resolver for `document`, whose `[sources]` table is `sources`.
    #[must_use]
    pub fn new(document: &'a str, sources: &'a BTreeMap<String, Source>) -> Self {
        Self {
            document,
            sources,
            cited: BTreeSet::new(),
            records: Vec::new(),
            warnings: Vec::new(),
        }
    }

    /// An [`EngineError::Config`] for `path`.
    #[must_use]
    pub fn error(&self, path: FieldPath, reason: impl Into<String>) -> EngineError {
        EngineError::config(self.document, path, reason)
    }

    /// Records a warning.
    pub fn warn(&mut self, path: FieldPath, message: impl Into<String>) {
        self.warnings.push(ConfigWarning {
            path,
            message: message.into(),
        });
    }

    /// The parameter at `path` in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] if the unit is unknown or not of `quantity`, an entry or the
    /// uncertainty is not finite, the uncertainty is negative, or the source is not in
    /// `[sources]`.
    pub fn si<T: Value>(
        &mut self,
        path: &FieldPath,
        parameter: &Parameter<T>,
        quantity: Quantity,
    ) -> Result<T, EngineError> {
        self.convert(path, parameter, Some(quantity))
    }

    /// The parameter at `path` in SI units, whatever its dimension: for values such as
    /// controller gains, whose units vary.
    ///
    /// # Errors
    ///
    /// As [`Resolver::si`], except that any well-formed unit is accepted.
    pub fn any(
        &mut self,
        path: &FieldPath,
        parameter: &Parameter<f64>,
    ) -> Result<f64, EngineError> {
        self.convert(path, parameter, None)
    }

    fn convert<T: Value>(
        &mut self,
        path: &FieldPath,
        parameter: &Parameter<T>,
        quantity: Option<Quantity>,
    ) -> Result<T, EngineError> {
        let (factor, si_unit) = self.factor(path, &parameter.unit, quantity)?;
        self.check_entries(&path.key("value"), &parameter.value, false)?;
        if let Some(uncertainty) = &parameter.uncertainty {
            self.check_entries(&path.key("uncertainty"), uncertainty, true)?;
        }
        self.cite(path, parameter.source.as_deref())?;
        let si = parameter.value.map(|x| x * factor);
        self.records.push(ParameterRecord {
            path: path.clone(),
            value: parameter.value.shown(Format::Plain),
            unit: parameter.unit.clone(),
            si: with_unit(&si.shown(Format::Significant), &si_unit),
            uncertainty: parameter.uncertainty.map(|u| u.shown(Format::Plain)),
            source: parameter.source.clone(),
        });
        Ok(si)
    }

    /// The SI factor of `unit`, which must be of `quantity` if one is given, and the SI unit.
    fn factor(
        &self,
        path: &FieldPath,
        unit: &str,
        quantity: Option<Quantity>,
    ) -> Result<(f64, String), EngineError> {
        let unit_path = path.key("unit");
        let parsed =
            parse_unit(unit).map_err(|error| self.error(unit_path.clone(), error.to_string()))?;
        let Some(quantity) = quantity else {
            return Ok((parsed.factor, parsed.dimension.to_string()));
        };
        if parsed.dimension != quantity.dimension {
            let reason = format!(
                "`{unit}` is not {}; use a unit such as {}",
                quantity.name, quantity.examples
            );
            return Err(self.error(unit_path, reason));
        }
        Ok((parsed.factor, quantity.si.to_owned()))
    }

    /// Every entry must be finite, and an uncertainty must not be negative.
    fn check_entries<T: Value>(
        &self,
        path: &FieldPath,
        value: &T,
        uncertainty: bool,
    ) -> Result<(), EngineError> {
        let bad = value
            .entries()
            .iter()
            .position(|x| !x.is_finite() || (uncertainty && *x < 0.0));
        bad.map_or(Ok(()), |index| {
            let found = value.entries().get(index).copied().unwrap_or(f64::NAN);
            let reason = if uncertainty {
                format!("an uncertainty must be finite and not negative; found {found}")
            } else {
                format!("must be finite; found {found}")
            };
            Err(self.error(T::entry(path, index), reason))
        })
    }

    /// Notes the source as cited, or warns that there is none.
    fn cite(&mut self, path: &FieldPath, source: Option<&str>) -> Result<(), EngineError> {
        let Some(key) = source else {
            self.warn(path.clone(), "no source");
            return Ok(());
        };
        if !self.sources.contains_key(key) {
            let reason = format!("no entry `{key}` in [sources]");
            return Err(self.error(path.key("source"), reason));
        }
        self.cited.insert(key.to_owned());
        Ok(())
    }

    /// A whole number of things, such as blades or cells: unit `1` and at least one.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] for another unit, a count below one, or an unknown source.
    pub fn count(
        &mut self,
        path: &FieldPath,
        parameter: &Parameter<u32>,
    ) -> Result<u32, EngineError> {
        if parameter.unit != "1" {
            let reason = format!("a count has unit `1`; found `{}`", parameter.unit);
            return Err(self.error(path.key("unit"), reason));
        }
        if parameter.value == 0 {
            return Err(self.error(path.key("value"), "must be at least 1"));
        }
        self.cite(path, parameter.source.as_deref())?;
        self.records.push(ParameterRecord {
            path: path.clone(),
            value: parameter.value.to_string(),
            unit: parameter.unit.clone(),
            si: parameter.value.to_string(),
            uncertainty: parameter.uncertainty.map(|u| u.to_string()),
            source: parameter.source.clone(),
        });
        Ok(parameter.value)
    }

    /// The parameters read, in order, and the warnings, including one for each source that no
    /// parameter cites.
    #[must_use]
    pub fn finish(mut self) -> (Vec<ParameterRecord>, Vec<ConfigWarning>) {
        let uncited: Vec<String> = self
            .sources
            .keys()
            .filter(|key| !self.cited.contains(*key))
            .cloned()
            .collect();
        for key in uncited {
            let path = FieldPath::root().key("sources").key(&key);
            self.warn(path, "no parameter cites this source");
        }
        (self.records, self.warnings)
    }
}
