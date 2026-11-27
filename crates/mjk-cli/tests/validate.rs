//! `mjk validate` (#12, #14): each study's output is identical on every run and is what the book
//! publishes.

use std::io;
use std::process::{Command, Output};

const INTEGRATORS: &str = include_str!("../../../docs/src/validation/integrator-study.md");
const DETERMINISM: &str = include_str!("../../../docs/src/validation/determinism.md");
const ATMOSPHERE: &str = include_str!("../../../docs/src/validation/atmosphere.md");
const ANALYTIC: &str = include_str!("../../../docs/src/validation/analytic.md");

fn mjk(args: &[&str]) -> io::Result<Output> {
    Command::new(env!("CARGO_BIN_EXE_mjk")).args(args).output()
}

#[test]
fn each_study_is_byte_identical_and_matches_the_book() {
    let studies = [
        ("integrators", INTEGRATORS),
        ("determinism", DETERMINISM),
        ("atmosphere", ATMOSPHERE),
        ("analytic", ANALYTIC),
    ];
    for (study, published) in studies {
        let first = mjk(&["validate", study]).expect("the mjk binary runs");
        let second = mjk(&["validate", study]).expect("the mjk binary runs");
        assert!(first.status.success(), "{study}");
        assert_eq!(first.stdout, second.stdout, "{study}");
        assert_eq!(String::from_utf8_lossy(&first.stdout), published, "{study}");
    }
}

#[test]
fn validate_without_a_study_names_the_choices() {
    let output = mjk(&["validate"]).expect("the mjk binary runs");
    assert_eq!(output.status.code(), Some(2));
    let text = String::from_utf8_lossy(&output.stderr);
    assert!(
        text.contains("name a study: integrators, determinism, atmosphere or analytic"),
        "{text}"
    );
}

#[test]
fn timing_reports_every_method() {
    let output = mjk(&["validate", "integrators", "--timing"]).expect("the mjk binary runs");
    assert!(output.status.success());
    let text = String::from_utf8_lossy(&output.stdout);
    for name in ["RK4, not renormalized", "RK4, renormalized", "RKMK4"] {
        assert!(text.contains(name), "{text}");
    }
    assert!(text.contains(" ns"), "{text}");
}

#[test]
fn version_help_and_unknown_arguments() {
    let version = mjk(&["--version"]).expect("the mjk binary runs");
    assert!(String::from_utf8_lossy(&version.stdout).starts_with("mjk "));
    let help = mjk(&[]).expect("the mjk binary runs");
    assert!(String::from_utf8_lossy(&help.stdout).contains("validate"));
    let unknown = mjk(&["fly"]).expect("the mjk binary runs");
    assert_eq!(unknown.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&unknown.stderr).contains("unknown arguments `fly`"));
}
