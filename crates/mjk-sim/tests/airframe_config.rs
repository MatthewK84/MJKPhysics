//! Acceptance tests for configuration loading (#7, #18): each malformed sample file must produce
//! an error that names the exact field at fault, and a number without a source loads with a
//! warning.

use mjk_core::{EngineError, FieldPath};
use mjk_math::Mat3;
use mjk_sim::config::{ConfigWarning, load_airframe};

/// A malformed fixture, the field its error must name, and a fragment of the reason.
struct Case {
    file: &'static str,
    text: &'static str,
    path: &'static str,
    reason: &'static str,
}

macro_rules! fixture {
    ($name:literal) => {
        (
            $name,
            include_str!(concat!("fixtures/airframe/", $name, ".toml")),
        )
    };
}

/// A fixture's name and text, the field its error must name, and a fragment of the reason.
type Row = ((&'static str, &'static str), &'static str, &'static str);

/// Malformed documents: syntax, header and table structure.
fn document_rows() -> Vec<Row> {
    vec![
        (fixture!("syntax_error"), "(document)", "line 4, column"),
        (fixture!("missing_version"), "schema_version", "missing"),
        (
            fixture!("newer_version"),
            "schema_version",
            "schema version 2, but this build reads version 1; update MJKPhysics",
        ),
        (
            fixture!("older_version"),
            "schema_version",
            "schema version 0 is not supported (oldest supported: 1)",
        ),
        (fixture!("wrong_kind"), "kind", "found \"scenario\""),
        (fixture!("unknown_field"), "mass.mas", "unknown field"),
        (
            fixture!("missing_field"),
            "mass",
            "missing `inertia`: give mass, cg and inertia, or [[mass.components]]",
        ),
    ]
}

/// Well-formed documents whose values are wrong or physically impossible.
fn value_rows() -> Vec<Row> {
    vec![
        (fixture!("wrong_type"), "mass.mass.value", "invalid type"),
        (
            fixture!("nested_wrong_type"),
            "mass.inertia.value[1][0]",
            "invalid type",
        ),
        (
            fixture!("short_row"),
            "mass.inertia.value[2]",
            "invalid length 2",
        ),
        (
            fixture!("negative_mass"),
            "mass.mass.value",
            "positive, finite mass",
        ),
        (fixture!("zero_mass"), "mass.mass.value", "found 0"),
        (
            fixture!("nonfinite_cg"),
            "mass.cg.value[1]",
            "must be finite; found NaN",
        ),
        (
            fixture!("singular_inertia"),
            "mass.inertia.value",
            "cannot be inverted in double precision",
        ),
        (
            fixture!("asymmetric_inertia"),
            "mass.inertia.value[0][1]",
            "not symmetric",
        ),
        (
            fixture!("triangle_violation"),
            "mass.inertia.value",
            "triangle inequality",
        ),
    ]
}

/// Documents that break the airframe format's rules (#18): units, sources and plausibility.
fn format_rows() -> Vec<Row> {
    vec![
        (fixture!("missing_unit"), "mass.mass", "missing `unit`"),
        (fixture!("bare_number"), "mass.mass", "missing `unit`"),
        (
            fixture!("wrong_unit"),
            "mass.mass.unit",
            "`m` is not a mass",
        ),
        (
            fixture!("unknown_unit"),
            "mass.mass.unit",
            "unknown unit `stone`",
        ),
        (
            fixture!("unknown_source"),
            "mass.mass.source",
            "no entry `manual` in [sources]",
        ),
        (
            fixture!("low_thrust"),
            "rotors",
            "thrust-to-weight is 0.666667: the rotors' 9.80665 N of maximum thrust cannot lift \
             14.71 N",
        ),
        (
            fixture!("overlapping_rotors"),
            "rotors[1]",
            "its disk overlaps rotors[0]'s: their hubs are 0.21 m apart, less than the 0.254 m \
             their radii add up to",
        ),
    ]
}

fn cases() -> Vec<Case> {
    document_rows()
        .into_iter()
        .chain(value_rows())
        .chain(format_rows())
        .map(|((file, text), path, reason)| Case {
            file,
            text,
            path,
            reason,
        })
        .collect()
}

#[test]
fn valid_sample_loads() {
    let text = include_str!("fixtures/airframe/valid.toml");
    let loaded = load_airframe("valid.toml", text).expect("valid sample loads");
    let airframe = &loaded.airframe;
    assert_eq!(airframe.name, "Example quadrotor");
    assert!((airframe.mass.mass_kg() - 1.5).abs() < 1e-15);
    assert!((airframe.mass.cg_m().z - 0.01).abs() < 1e-15);
    let inverse = airframe.mass.inertia_kg_m2() * airframe.mass.inertia_inverse();
    assert!((inverse - Mat3::identity()).max_abs() < 1e-12);
    let propeller = &airframe.propellers["p10"];
    assert!((propeller.diameter_m - 0.254).abs() < 1e-15);
    assert!((propeller.max_thrust_n - 2.0 * 9.806_65).abs() < 1e-12);
    assert_eq!(airframe.rotors.len(), 1);
    assert_eq!(loaded.parameters.len(), 8);
    assert_eq!(loaded.warnings, vec![]);
}

#[test]
fn a_number_without_a_source_loads_with_a_warning() {
    let text = include_str!("fixtures/airframe/missing_source.toml");
    let loaded = load_airframe("missing_source.toml", text).expect("loads");
    assert_eq!(
        loaded.warnings,
        vec![ConfigWarning {
            path: FieldPath::root().key("mass").key("mass"),
            message: "no source".to_owned(),
        }]
    );
    assert_eq!(loaded.warnings[0].to_string(), "mass.mass: no source");
}

#[test]
fn every_malformed_sample_names_the_exact_field() {
    for case in cases() {
        let error = load_airframe(case.file, case.text).expect_err(case.file);
        let EngineError::Config {
            document,
            path,
            reason,
        } = &error
        else {
            panic!("{}: expected a configuration error, got {error}", case.file);
        };
        assert_eq!(document, case.file);
        assert_eq!(path.to_string(), case.path, "{}: {error}", case.file);
        assert!(
            reason.contains(case.reason),
            "{}: reason {reason:?} lacks {:?}",
            case.file,
            case.reason
        );
    }
}

#[test]
fn messages_read_as_document_field_reason() {
    let text = include_str!("fixtures/airframe/negative_mass.toml");
    let error = load_airframe("airframes/example.toml", text).expect_err("negative mass");
    assert_eq!(
        error.to_string(),
        "airframes/example.toml: mass.mass.value: must be a positive, finite mass in \
         kilograms; found -1.5"
    );
}
