//! `mjk run` and `mjk replay` (#17): a recorded run replays to the bit, through files or the
//! standard streams; a changed record diverges at the tick it affects; and every failure exits
//! with its documented code and a message naming what is wrong.

use std::io::{self, Write};
use std::path::PathBuf;
use std::process::{self, Command, Output, Stdio};
use std::{env, fs};

use mjk_math::Xoshiro256StarStar;
use serde_json::Value;

fn mjk(args: &[&str]) -> io::Result<Output> {
    Command::new(env!("CARGO_BIN_EXE_mjk")).args(args).output()
}

/// Runs `mjk` with `input` on standard input.
fn mjk_reading(args: &[&str], input: &[u8]) -> io::Result<Output> {
    let mut child = Command::new(env!("CARGO_BIN_EXE_mjk"))
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
    if let Some(mut stdin) = child.stdin.take() {
        stdin.write_all(input)?;
    }
    child.wait_with_output()
}

/// An empty scratch directory for one test.
fn scratch(test: &str) -> io::Result<PathBuf> {
    let dir = env::temp_dir().join(format!("mjk-run-{}-{test}", process::id()));
    if dir.exists() {
        fs::remove_dir_all(&dir)?;
    }
    fs::create_dir_all(&dir)?;
    Ok(dir)
}

/// The reference run's log, as `mjk run reference --log -` writes it.
fn reference_log() -> Vec<u8> {
    mjk(&["run", "reference", "--log", "-"])
        .map(|output| output.stdout)
        .unwrap_or_default()
}

/// The reference log with `change` applied to its JSON.
fn changed_log(change: impl FnOnce(&mut Value)) -> Vec<u8> {
    let mut log: Value = serde_json::from_slice(&reference_log()).unwrap_or(Value::Null);
    change(&mut log);
    serde_json::to_vec(&log).unwrap_or_default()
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

#[test]
fn a_run_written_to_files_replays_to_the_bit() {
    let dir = scratch("files").expect("a scratch directory");
    let (log, csv) = (dir.join("run.json"), dir.join("run.csv"));
    let run = mjk(&[
        "run",
        "reference",
        "--log",
        &log.to_string_lossy(),
        "--csv",
        &csv.to_string_lossy(),
    ])
    .expect("runs");
    assert!(run.status.success(), "{run:?}");
    assert_eq!(
        text(&run.stdout),
        "reference run, seed 14: 10000 ticks at 1000 Hz, 20 inputs, 11 checkpoints, final \
         fingerprint ba6762fb8571ca0f\n"
    );
    let replay = mjk(&["replay", &log.to_string_lossy()]).expect("runs");
    assert!(replay.status.success(), "{replay:?}");
    assert_eq!(
        text(&replay.stdout),
        "replay matches its record: 10000 ticks, 11 checkpoints and 14014 samples, bit for bit\n"
    );
    let table = fs::read_to_string(&csv).unwrap_or_default();
    assert!(
        table.starts_with("time (s),north (m),east (m),down (m),velocity_north (m/s),"),
        "{table}"
    );
    assert_eq!(table.lines().count(), 1002);
    assert_eq!(fs::read(&log).ok(), Some(reference_log()));
    fs::remove_dir_all(dir).expect("cleaned up");
}

#[test]
fn logs_go_to_standard_output_and_replay_reads_standard_input() {
    let first = mjk(&["run", "reference", "--log", "-"]).expect("runs");
    let second = mjk(&["run", "reference", "--log", "-"]).expect("runs");
    assert!(first.status.success());
    assert_eq!(first.stdout, second.stdout, "byte-identical across runs");
    assert!(
        text(&first.stderr).starts_with("reference run, seed 14:"),
        "the summary moves aside"
    );
    let log: Value = serde_json::from_slice(&first.stdout).expect("one JSON document");
    assert_eq!(log["format_version"], 1);
    assert_eq!(log["checkpoints"][10]["hash"], "ba6762fb8571ca0f");
    let replay = mjk_reading(&["replay", "-"], &first.stdout).expect("runs");
    assert!(replay.status.success(), "{replay:?}");
    let csv = mjk(&["run", "reference", "--csv", "-"]).expect("runs");
    assert!(text(&csv.stdout).starts_with("time (s),"));
}

#[test]
fn the_seed_changes_the_run() {
    let output = mjk(&["run", "reference", "--seed", "15"]).expect("runs");
    let summary = text(&output.stdout);
    assert!(summary.starts_with("reference run, seed 15:"), "{summary}");
    assert!(!summary.contains("ba6762fb8571ca0f"), "{summary}");
}

#[test]
fn a_changed_record_diverges_with_exit_code_1() {
    let log = changed_log(|log| {
        let thrust = log["inputs"][1]["thrust_n"].as_f64().unwrap_or_default();
        log["inputs"][1]["thrust_n"] = Value::from(thrust * (1.0 + 1e-9));
    });
    let replay = mjk_reading(&["replay", "-"], &log).expect("runs");
    assert_eq!(replay.status.code(), Some(1));
    let report = text(&replay.stdout);
    assert!(
        report.starts_with("replay diverged at tick 510: north was "),
        "{report}"
    );
}

#[test]
fn a_malformed_or_unsupported_record_exits_with_code_3_naming_the_field() {
    let unknown = changed_log(|log| log["inputs"][2]["tikc"] = Value::from(1));
    let version = changed_log(|log| log["format_version"] = Value::from(2));
    let mut trailing = reference_log();
    trailing.extend_from_slice(b"{}");
    let cases = [
        (
            unknown,
            "error: standard input: inputs[2].tikc: unknown field `tikc`",
        ),
        (
            version,
            "error: standard input: format_version: this build replays only 1; found 2",
        ),
        (
            trailing,
            "error: standard input: (document): trailing characters",
        ),
        (b"[1, 2".to_vec(), "error: standard input: "),
    ];
    for (input, message) in cases {
        let replay = mjk_reading(&["replay", "-"], &input).expect("runs");
        assert_eq!(replay.status.code(), Some(3), "{message}");
        let report = text(&replay.stderr);
        assert!(report.starts_with(message), "{report}");
    }
}

#[test]
fn files_that_cannot_be_read_or_written_exit_with_code_5() {
    let dir = scratch("io").expect("a scratch directory");
    let missing = dir.join("missing.json");
    let replay = mjk(&["replay", &missing.to_string_lossy()]).expect("runs");
    assert_eq!(replay.status.code(), Some(5));
    assert!(text(&replay.stderr).starts_with("error: cannot read "));
    let blocked = dir.join("a-file");
    fs::write(&blocked, "").expect("a scratch file");
    let target = blocked.join("run.json");
    let run = mjk(&["run", "reference", "--log", &target.to_string_lossy()]).expect("runs");
    assert_eq!(run.status.code(), Some(5));
    assert!(text(&run.stderr).starts_with("error: cannot write "));
    fs::remove_dir_all(dir).expect("cleaned up");
}

#[test]
fn command_line_mistakes_exit_with_code_2() {
    let cases: [(&[&str], &str); 8] = [
        (&["run"], "name a scenario: `reference`"),
        (
            &["run", "hover.toml"],
            "cannot run `hover.toml`: scenario files arrive with #28",
        ),
        (
            &["run", "reference", "--seed", "-1"],
            "--seed takes a whole number from 0 to 18446744073709551615; found `-1`",
        ),
        (
            &["run", "reference", "--speed", "2"],
            "unknown option to run: `--speed 2`",
        ),
        (
            &["run", "reference", "--log"],
            "unknown option to run: `--log`",
        ),
        (
            &["run", "reference", "--log", "-", "--csv", "-"],
            "only one of --log and --csv can write to standard output",
        ),
        (&["replay"], "name one run log to replay"),
        (
            &["replay", "a.json", "b.json"],
            "name one run log to replay",
        ),
    ];
    for (args, message) in cases {
        let output = mjk(args).expect("runs");
        assert_eq!(output.status.code(), Some(2), "{args:?}");
        let report = text(&output.stderr);
        assert!(report.starts_with(&format!("error: {message}")), "{report}");
        assert!(report.contains("usage: mjk"), "{report}");
    }
}

#[test]
fn json_numbers_read_back_to_the_same_bits() {
    // Replay compares to the bit, so every finite double must survive JSON exactly: serde_json's
    // default float parser is not exact, which is why the CLI enables `float_roundtrip`.
    let mut random = Xoshiro256StarStar::named(17, "json");
    let edges = [
        0.0,
        -0.0,
        f64::MIN_POSITIVE,
        f64::MIN_POSITIVE / 3.0,
        5e-324,
        f64::MAX,
        f64::MIN,
        f64::EPSILON,
        0.1,
        1.0 / 3.0,
    ];
    let samples = (0..100_000)
        .map(|_| f64::from_bits(random.next_u64()))
        .filter(|value| value.is_finite())
        .chain(edges);
    for value in samples {
        let text = serde_json::to_string(&value).unwrap_or_default();
        let back: f64 = serde_json::from_str(&text).unwrap_or(f64::NAN);
        assert_eq!(back.to_bits(), value.to_bits(), "{text}");
    }
}
