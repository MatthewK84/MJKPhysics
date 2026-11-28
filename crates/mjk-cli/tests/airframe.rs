//! `mjk airframe` (#18): the page of an airframe file is the book's, through a file or standard
//! input; warnings go to standard error; a bad file exits with code 3 naming the field.

use std::io::{self, Write};
use std::process::{Command, Output, Stdio};

const EXAMPLE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../airframes/example-quadrotor.toml"
);
const EXAMPLE_TEXT: &str = include_str!("../../../airframes/example-quadrotor.toml");
const PAGE: &str = include_str!("../../../docs/src/airframes/example-quadrotor.md");
const MISSING_SOURCE: &str =
    include_str!("../../mjk-sim/tests/fixtures/airframe/missing_source.toml");
const WRONG_UNIT: &str = include_str!("../../mjk-sim/tests/fixtures/airframe/wrong_unit.toml");

fn mjk(args: &[&str]) -> io::Result<Output> {
    Command::new(env!("CARGO_BIN_EXE_mjk")).args(args).output()
}

/// Runs `mjk` with `input` on standard input.
fn mjk_reading(args: &[&str], input: &str) -> io::Result<Output> {
    let mut child = Command::new(env!("CARGO_BIN_EXE_mjk"))
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
    if let Some(mut stdin) = child.stdin.take() {
        stdin.write_all(input.as_bytes())?;
    }
    child.wait_with_output()
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

#[test]
fn the_page_is_the_books_from_a_file_or_standard_input() {
    let from_file = mjk(&["airframe", EXAMPLE]).expect("runs");
    assert!(from_file.status.success(), "{from_file:?}");
    assert_eq!(text(&from_file.stdout), PAGE);
    assert_eq!(text(&from_file.stderr), "", "the example has no warnings");
    let from_stdin = mjk_reading(&["airframe", "-"], EXAMPLE_TEXT).expect("runs");
    assert_eq!(from_stdin.stdout, from_file.stdout);
}

#[test]
fn warnings_go_to_standard_error_and_the_file_still_loads() {
    let output = mjk_reading(&["airframe", "-"], MISSING_SOURCE).expect("runs");
    assert!(output.status.success(), "{output:?}");
    assert_eq!(
        text(&output.stderr),
        "warning: standard input: mass.mass: no source\n"
    );
    assert!(text(&output.stdout).starts_with("# Example quadrotor\n"));
}

#[test]
fn a_bad_file_exits_with_code_3_naming_the_field() {
    let output = mjk_reading(&["airframe", "-"], WRONG_UNIT).expect("runs");
    assert_eq!(output.status.code(), Some(3));
    assert_eq!(
        text(&output.stderr),
        "error: standard input: mass.mass.unit: `m` is not a mass; use a unit such as kg or g\n"
    );
    assert_eq!(text(&output.stdout), "");
}

#[test]
fn a_missing_file_or_a_wrong_command_line_has_its_exit_code() {
    let missing = mjk(&["airframe", "no-such-airframe.toml"]).expect("runs");
    assert_eq!(missing.status.code(), Some(5));
    assert!(text(&missing.stderr).starts_with("error: cannot read no-such-airframe.toml: "));
    for args in [&["airframe"][..], &["airframe", "a.toml", "b.toml"][..]] {
        let output = mjk(args).expect("runs");
        assert_eq!(output.status.code(), Some(2), "{args:?}");
        assert!(text(&output.stderr).starts_with("error: name one airframe file\n"));
    }
}
