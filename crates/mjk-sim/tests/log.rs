//! The columnar log writer (#17): rows go in column by column within the capacity, a full log
//! or a wrong-width row is an error that changes nothing, CSV reads back exactly, and samples
//! survive serialization, non-finite ones included.

use mjk_sim::log::{Channel, ChannelLog, ColumnLog, LogError, Sample, write_csv};
use serde::de::IntoDeserializer;
use serde::de::value::U64Deserializer;
use serde::de::value::{Error as ValueError, F64Deserializer, I64Deserializer, StrDeserializer};
use serde::{Deserialize, Serialize};

const CHANNELS: [Channel; 2] = [
    Channel {
        name: "time",
        unit: "s",
    },
    Channel {
        name: "height",
        unit: "m",
    },
];

#[test]
fn rows_are_stored_column_by_column() {
    let mut log = ColumnLog::new(&CHANNELS, 3);
    assert_eq!(log.channels(), &CHANNELS);
    assert_eq!(log.rows(), 0);
    assert_eq!(log.push(&[0.0, 10.0]), Ok(()));
    assert_eq!(log.push(&[0.5, 9.0]), Ok(()));
    assert_eq!(log.rows(), 2);
    assert_eq!(log.column(0), Some(&[0.0, 0.5][..]));
    assert_eq!(log.column(1), Some(&[10.0, 9.0][..]));
    assert_eq!(log.column(2), None);
}

#[test]
fn a_full_log_or_a_wrong_row_is_an_error_and_changes_nothing() {
    let mut log = ColumnLog::new(&CHANNELS, 1);
    assert_eq!(
        log.push(&[1.0]),
        Err(LogError::Width {
            expected: 2,
            found: 1
        })
    );
    assert_eq!(
        log.push(&[1.0, 2.0, 3.0]),
        Err(LogError::Width {
            expected: 2,
            found: 3
        })
    );
    assert_eq!(log.rows(), 0);
    assert_eq!(log.push(&[1.0, 2.0]), Ok(()));
    assert_eq!(log.push(&[3.0, 4.0]), Err(LogError::Full { capacity: 1 }));
    assert_eq!(log.rows(), 1);
    assert_eq!(log.column(1), Some(&[2.0][..]));
}

#[test]
fn errors_explain_themselves() {
    assert_eq!(
        LogError::Full { capacity: 1001 }.to_string(),
        "the log is full at 1001 rows; size it for the whole run"
    );
    assert_eq!(
        LogError::Width {
            expected: 14,
            found: 13
        }
        .to_string(),
        "a row has 13 values, but the log has 14 channels"
    );
}

#[test]
fn channel_logs_carry_names_units_and_values() {
    let mut log = ColumnLog::new(&CHANNELS, 2);
    log.push(&[0.0, f64::NAN]).expect("room");
    let channels = log.channel_logs();
    assert_eq!(channels.len(), 2);
    assert_eq!(channels[0].name, "time");
    assert_eq!(channels[1].unit, "m");
    assert_eq!(channels[1].values, vec![Sample(f64::NAN)]);
}

fn channel(name: &str, unit: &str, values: &[f64]) -> ChannelLog {
    ChannelLog {
        name: name.to_owned(),
        unit: unit.to_owned(),
        values: values.iter().copied().map(Sample).collect(),
    }
}

#[test]
fn csv_has_a_header_with_units_and_numbers_that_read_back_exactly() {
    let channels = [
        channel("time", "s", &[0.0, 0.01, 1e-300]),
        channel("rate", "rad/s", &[-0.3, f64::NAN, 1.5e17]),
        channel("thrust", "N", &[f64::INFINITY, f64::NEG_INFINITY, 19.6]),
    ];
    let mut text = String::new();
    write_csv(&channels, &mut text).expect("a String never fails");
    let expected = "time (s),rate (rad/s),thrust (N)\n\
                    0.0,-0.3,inf\n\
                    0.01,NaN,-inf\n\
                    1e-300,1.5e17,19.6\n";
    assert_eq!(text, expected);
    for line in text.lines().skip(1) {
        for field in line.split(',') {
            let value: f64 = field.parse().expect("Rust reads its own output");
            assert_eq!(format!("{value:?}"), field);
        }
    }
}

#[test]
fn a_short_channel_leaves_its_fields_empty() {
    let channels = [channel("a", "1", &[1.0, 2.0]), channel("b", "1", &[3.0])];
    let mut text = String::new();
    write_csv(&channels, &mut text).expect("a String never fails");
    assert_eq!(text, "a (1),b (1)\n1.0,3.0\n2.0,\n");
    let mut empty = String::new();
    write_csv(&[], &mut empty).expect("a String never fails");
    assert_eq!(empty, "\n");
}

#[test]
fn samples_are_equal_by_bits_or_when_both_are_nan() {
    assert_eq!(Sample(1.0), Sample(1.0));
    assert_ne!(Sample(0.0), Sample(-0.0));
    assert_eq!(Sample(f64::NAN), Sample(-f64::NAN));
    assert_ne!(Sample(f64::NAN), Sample(0.0));
    assert_ne!(Sample(0.0), Sample(f64::NAN));
}

#[derive(Serialize, Deserialize)]
struct Values {
    values: Vec<Sample>,
}

#[test]
fn finite_samples_serialize_as_numbers_and_the_rest_as_strings() {
    let values = Values {
        values: [0.25, f64::NAN, f64::INFINITY, f64::NEG_INFINITY]
            .map(Sample)
            .to_vec(),
    };
    let text = toml::to_string(&values).expect("serializes");
    assert_eq!(text, "values = [0.25, \"NaN\", \"inf\", \"-inf\"]\n");
    let back: Values = toml::from_str(&text).expect("reads back");
    assert_eq!(back.values, values.values);
}

fn from_f64(value: f64) -> Result<Sample, ValueError> {
    let deserializer: F64Deserializer<ValueError> = value.into_deserializer();
    Sample::deserialize(deserializer)
}

fn from_i64(value: i64) -> Result<Sample, ValueError> {
    let deserializer: I64Deserializer<ValueError> = value.into_deserializer();
    Sample::deserialize(deserializer)
}

fn from_u64(value: u64) -> Result<Sample, ValueError> {
    let deserializer: U64Deserializer<ValueError> = value.into_deserializer();
    Sample::deserialize(deserializer)
}

fn from_str(text: &str) -> Result<Sample, ValueError> {
    let deserializer: StrDeserializer<'_, ValueError> = text.into_deserializer();
    Sample::deserialize(deserializer)
}

#[test]
fn samples_read_numbers_integers_and_the_three_non_finite_names() {
    assert_eq!(from_f64(-2.5), Ok(Sample(-2.5)));
    assert_eq!(from_i64(-7), Ok(Sample(-7.0)));
    assert_eq!(from_i64(2_147_483_647), Ok(Sample(2_147_483_647.0)));
    assert_eq!(from_u64(42), Ok(Sample(42.0)));
    assert_eq!(from_str("NaN"), Ok(Sample(f64::NAN)));
    assert_eq!(from_str("inf"), Ok(Sample(f64::INFINITY)));
    assert_eq!(from_str("-inf"), Ok(Sample(f64::NEG_INFINITY)));
}

#[test]
fn samples_reject_other_strings_and_integers_that_would_round() {
    let other = from_str("nan").expect_err("only `NaN`");
    assert!(
        other
            .to_string()
            .contains("a number, or \"NaN\", \"inf\" or \"-inf\""),
        "{other}"
    );
    for error in [
        from_i64(2_147_483_648).expect_err("beyond i32"),
        from_i64(-2_147_483_649).expect_err("beyond i32"),
        from_u64(1 << 53).expect_err("beyond i32"),
    ] {
        assert!(
            error.to_string().contains("write it with a decimal point"),
            "{error}"
        );
    }
    assert_eq!(from_i64(-2_147_483_648), Ok(Sample(-2_147_483_648.0)));
}
