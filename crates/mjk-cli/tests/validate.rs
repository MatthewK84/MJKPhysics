//! `mjk validate` (#12, #14, #17): each study's output is identical on every run and is what the
//! book publishes, whether printed one study at a time or written as a set of tables.

use std::path::{Path, PathBuf};
use std::process::{self, Command, Output};
use std::{env, fs, io};

const INTEGRATORS: &str = include_str!("../../../docs/src/validation/integrator-study.md");
const DETERMINISM: &str = include_str!("../../../docs/src/validation/determinism.md");
const ATMOSPHERE: &str = include_str!("../../../docs/src/validation/atmosphere.md");
const ANALYTIC: &str = include_str!("../../../docs/src/validation/analytic.md");
const MOTOR: &str = include_str!("../../../docs/src/validation/motor.md");

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
        ("motor", MOTOR),
    ];
    for (study, published) in studies {
        let first = mjk(&["validate", study]).expect("the mjk binary runs");
        let second = mjk(&["validate", study]).expect("the mjk binary runs");
        assert!(first.status.success(), "{study}");
        assert_eq!(first.stdout, second.stdout, "{study}");
        assert_eq!(String::from_utf8_lossy(&first.stdout), published, "{study}");
    }
}

/// A path for a fresh directory for one test; nothing is there yet.
fn scratch(name: &str) -> io::Result<PathBuf> {
    let dir = env::temp_dir().join(format!("mjk-validate-{}-{name}", process::id()));
    if dir.exists() {
        fs::remove_dir_all(&dir)?;
    }
    Ok(dir)
}

fn read(path: &Path) -> String {
    fs::read_to_string(path).unwrap_or_default()
}

#[test]
fn validate_writes_every_table_byte_identically_and_as_published() {
    let first = scratch("first").expect("a scratch directory");
    let second = scratch("second").expect("a scratch directory");
    let published = [
        ("integrator-study.md", INTEGRATORS),
        ("determinism.md", DETERMINISM),
        ("atmosphere.md", ATMOSPHERE),
        ("analytic.md", ANALYTIC),
        ("motor.md", MOTOR),
    ];
    for dir in [&first, &second] {
        let output = mjk(&["validate", "--out", &dir.to_string_lossy()]).expect("runs");
        assert!(output.status.success(), "{output:?}");
        let listing = String::from_utf8_lossy(&output.stdout);
        assert_eq!(listing.lines().count(), 5, "{listing}");
        assert!(
            listing.lines().all(|line| line.starts_with("pass  ")),
            "{listing}"
        );
    }
    for (file, table) in published {
        assert_eq!(read(&first.join(file)), table, "{file}");
        assert_eq!(
            fs::read(first.join(file)).ok(),
            fs::read(second.join(file)).ok()
        );
    }
    fs::remove_dir_all(first).expect("cleaned up");
    fs::remove_dir_all(second).expect("cleaned up");
}

#[test]
fn validate_names_a_directory_it_cannot_create() {
    let dir = scratch("blocked").expect("a scratch directory");
    fs::create_dir_all(&dir).expect("the scratch directory");
    let file = dir.join("a-file");
    fs::write(&file, "not a directory").expect("a scratch file");
    let blocked = file.join("tables");
    let output = mjk(&["validate", "--out", &blocked.to_string_lossy()]).expect("runs");
    assert_eq!(output.status.code(), Some(5));
    let text = String::from_utf8_lossy(&output.stderr);
    assert!(text.starts_with("error: cannot create "), "{text}");
    fs::remove_dir_all(dir).expect("cleaned up");
}

#[test]
fn unknown_studies_and_options_are_usage_errors() {
    let cases = [
        (
            &["validate", "fly"][..],
            "unknown study `fly`: the studies are integrators",
        ),
        (&["validate", "--out"][..], "--out takes a directory"),
        (
            &["validate", "analytic", "--timing"][..],
            "unknown arguments to validate: `analytic --timing`",
        ),
    ];
    for (args, message) in cases {
        let output = mjk(args).expect("runs");
        assert_eq!(output.status.code(), Some(2), "{args:?}");
        let text = String::from_utf8_lossy(&output.stderr);
        assert!(text.contains(message), "{text}");
        assert!(text.contains("usage: mjk"), "{text}");
    }
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
