//! The columnar log writer ([#17](https://github.com/MatthewK84/MJKPhysics/issues/17)): named
//! channels of `f64` samples, stored column by column.
//!
//! A [`ColumnLog`] is sized when it is made and never grows, so logging a row allocates nothing
//! inside a step, and a full log is an error, never a silent drop. Engine crates do no I/O: a
//! front end writes the log out after the run, or between steps, as JSON through [`ChannelLog`]
//! or as CSV with [`write_csv`], so writing never slows a step.

use std::fmt;

use serde::de::{self, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// A logged quantity: its name and unit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Channel {
    /// Name, such as `"north"`.
    pub name: &'static str,
    /// Unit, such as `"m"`; `"1"` for a dimensionless quantity.
    pub unit: &'static str,
}

/// Why a row could not be logged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogError {
    /// The log already holds as many rows as it was sized for.
    Full {
        /// The number of rows it holds.
        capacity: usize,
    },
    /// The row does not have one value per channel.
    Width {
        /// The number of channels.
        expected: usize,
        /// The number of values in the row.
        found: usize,
    },
}

impl fmt::Display for LogError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Full { capacity } => write!(
                f,
                "the log is full at {capacity} rows; size it for the whole run"
            ),
            Self::Width { expected, found } => write!(
                f,
                "a row has {found} values, but the log has {expected} channels"
            ),
        }
    }
}

/// Samples of fixed channels, stored column by column in storage sized when the log is made.
#[derive(Clone, Debug, PartialEq)]
pub struct ColumnLog {
    channels: &'static [Channel],
    columns: Vec<Vec<f64>>,
    rows: usize,
    capacity: usize,
}

impl ColumnLog {
    /// An empty log of `channels`, with room for `capacity` rows.
    #[must_use]
    pub fn new(channels: &'static [Channel], capacity: usize) -> Self {
        Self {
            channels,
            columns: channels
                .iter()
                .map(|_| Vec::with_capacity(capacity))
                .collect(),
            rows: 0,
            capacity,
        }
    }

    /// The channels, in column order.
    #[must_use]
    pub const fn channels(&self) -> &'static [Channel] {
        self.channels
    }

    /// The number of rows logged.
    #[must_use]
    pub const fn rows(&self) -> usize {
        self.rows
    }

    /// The samples of channel `index`, or `None` past the last channel.
    #[must_use]
    pub fn column(&self, index: usize) -> Option<&[f64]> {
        self.columns.get(index).map(Vec::as_slice)
    }

    /// Logs one row: a value for each channel, in channel order. Allocates nothing.
    ///
    /// # Errors
    ///
    /// [`LogError::Width`] if `row` does not have one value per channel, and [`LogError::Full`]
    /// if the log already holds `capacity` rows. Either way nothing is logged.
    pub fn push(&mut self, row: &[f64]) -> Result<(), LogError> {
        if row.len() != self.channels.len() {
            return Err(LogError::Width {
                expected: self.channels.len(),
                found: row.len(),
            });
        }
        if self.rows == self.capacity {
            return Err(LogError::Full {
                capacity: self.capacity,
            });
        }
        // Within capacity, `push` never reallocates.
        for (column, value) in self.columns.iter_mut().zip(row) {
            column.push(*value);
        }
        self.rows += 1;
        Ok(())
    }

    /// The log as one [`ChannelLog`] per channel, for JSON output.
    #[must_use]
    pub fn channel_logs(&self) -> Vec<ChannelLog> {
        self.channels
            .iter()
            .zip(&self.columns)
            .map(|(channel, column)| ChannelLog {
                name: channel.name.to_owned(),
                unit: channel.unit.to_owned(),
                values: column.iter().copied().map(Sample).collect(),
            })
            .collect()
    }
}

/// One channel of a log as written to, and read from, a file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChannelLog {
    /// Name, such as `"north"`.
    pub name: String,
    /// Unit, such as `"m"`.
    pub unit: String,
    /// The samples, one per row.
    pub values: Vec<Sample>,
}

/// One sample: a JSON number when finite, otherwise the string `"NaN"`, `"inf"` or `"-inf"`,
/// since JSON has no non-finite numbers.
///
/// Two samples are equal when their bits are, or when both are NaN.
#[derive(Clone, Copy, Debug)]
pub struct Sample(pub f64);

impl PartialEq for Sample {
    fn eq(&self, other: &Self) -> bool {
        self.0.to_bits() == other.0.to_bits() || (self.0.is_nan() && other.0.is_nan())
    }
}

impl Serialize for Sample {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let value = self.0;
        if value.is_finite() {
            return serializer.serialize_f64(value);
        }
        let text = if value.is_nan() {
            "NaN"
        } else if value.is_sign_negative() {
            "-inf"
        } else {
            "inf"
        };
        serializer.serialize_str(text)
    }
}

impl<'de> Deserialize<'de> for Sample {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(SampleVisitor)
    }
}

/// Reads a sample from a number, or from one of the three non-finite strings.
struct SampleVisitor;

impl Visitor<'_> for SampleVisitor {
    type Value = Sample;

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a number, or \"NaN\", \"inf\" or \"-inf\"")
    }

    fn visit_f64<E: de::Error>(self, value: f64) -> Result<Sample, E> {
        Ok(Sample(value))
    }

    fn visit_i64<E: de::Error>(self, value: i64) -> Result<Sample, E> {
        exact_integer(i32::try_from(value).ok())
    }

    fn visit_u64<E: de::Error>(self, value: u64) -> Result<Sample, E> {
        exact_integer(i32::try_from(value).ok())
    }

    fn visit_str<E: de::Error>(self, text: &str) -> Result<Sample, E> {
        match text {
            "NaN" => Ok(Sample(f64::NAN)),
            "inf" => Ok(Sample(f64::INFINITY)),
            "-inf" => Ok(Sample(f64::NEG_INFINITY)),
            _ => Err(E::invalid_value(de::Unexpected::Str(text), &self)),
        }
    }
}

/// An integer sample, which must fit in an `i32` to convert exactly.
fn exact_integer<E: de::Error>(value: Option<i32>) -> Result<Sample, E> {
    value.map(|v| Sample(f64::from(v))).ok_or_else(|| {
        E::custom("an integer sample beyond ±2147483647; write it with a decimal point")
    })
}

/// Writes channels as CSV: a header of `name (unit)` fields, then one line per row.
///
/// Numbers use the shortest form that reads back to the same `f64`, with an exponent for very
/// small and very large magnitudes; non-finite values are `NaN`, `inf` and `-inf`. A channel
/// shorter than the others leaves its fields empty.
///
/// # Errors
///
/// Only if `out` fails.
pub fn write_csv<W: fmt::Write>(channels: &[ChannelLog], out: &mut W) -> fmt::Result {
    let header: Vec<String> = channels
        .iter()
        .map(|channel| format!("{} ({})", channel.name, channel.unit))
        .collect();
    writeln!(out, "{}", header.join(","))?;
    let rows = channels
        .iter()
        .map(|channel| channel.values.len())
        .max()
        .unwrap_or(0);
    for row in 0..rows {
        let fields: Vec<String> = channels
            .iter()
            .map(|channel| {
                channel
                    .values
                    .get(row)
                    .map_or_else(String::new, |sample| format!("{:?}", sample.0))
            })
            .collect();
        writeln!(out, "{}", fields.join(","))?;
    }
    Ok(())
}
