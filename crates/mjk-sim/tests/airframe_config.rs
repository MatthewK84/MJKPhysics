//! Acceptance tests for configuration loading (#7): each malformed sample file must produce an
//! error that names the exact field at fault.

use mjk_core::EngineError;
use mjk_sim::config::load_airframe;

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
        (fixture!("syntax_error"), "(document)", "line 3, column"),
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
        (fixture!("unknown_field"), "mass.mas_kg", "unknown field"),
        (fixture!("missing_field"), "mass", "missing field `mass_kg`"),
    ]
}

/// Well-formed documents whose values are wrong or physically impossible.
fn value_rows() -> Vec<Row> {
    vec![
        (fixture!("wrong_type"), "mass.mass_kg", "invalid type"),
        (
            fixture!("nested_wrong_type"),
            "mass.inertia_kg_m2[1][0]",
            "invalid type",
        ),
        (
            fixture!("short_row"),
            "mass.inertia_kg_m2[2]",
            "invalid length 2",
        ),
        (
            fixture!("negative_mass"),
            "mass.mass_kg",
            "positive, finite mass",
        ),
        (fixture!("zero_mass"), "mass.mass_kg", "found 0"),
        (
            fixture!("asymmetric_inertia"),
            "mass.inertia_kg_m2[0][1]",
            "not symmetric",
        ),
        (
            fixture!("triangle_violation"),
            "mass.inertia_kg_m2",
            "triangle inequality",
        ),
    ]
}

fn cases() -> Vec<Case> {
    document_rows()
        .into_iter()
        .chain(value_rows())
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
    let airframe = load_airframe("valid.toml", text).expect("valid sample loads");
    assert_eq!(airframe.name, "Example quadrotor");
    assert!((airframe.mass.cg_m[2] - 0.01).abs() < 1e-12);
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
        "airframes/example.toml: mass.mass_kg: must be a positive, finite mass in kilograms; \
         found -1.5"
    );
}
