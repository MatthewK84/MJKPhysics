//! `mjk random` (#14): named streams interleaved as little-endian words, whole rounds only, and a
//! quiet end when the reader stops early.

use std::io::{self, Read};
use std::process::{Command, Output, Stdio};

use mjk_math::Xoshiro256StarStar;

fn mjk(args: &[&str]) -> io::Result<Output> {
    Command::new(env!("CARGO_BIN_EXE_mjk")).args(args).output()
}

/// The bytes `mjk random` should write: `rounds` rounds of one word from each stream.
fn expected(seed: u64, names: &[&str], rounds: usize) -> Vec<u8> {
    let mut streams: Vec<Xoshiro256StarStar> = names
        .iter()
        .map(|name| Xoshiro256StarStar::named(seed, name))
        .collect();
    let mut bytes = Vec::new();
    for _ in 0..rounds {
        for stream in &mut streams {
            bytes.extend_from_slice(&stream.next_u64().to_le_bytes());
        }
    }
    bytes
}

#[test]
fn streams_are_interleaved_as_little_endian_words() {
    let output = mjk(&["random", "42", "gyro", "accelerometer", "--bytes", "48"]).expect("runs");
    assert!(output.status.success());
    assert_eq!(output.stdout, expected(42, &["gyro", "accelerometer"], 3));
    // The first word is the gyro stream's first draw.
    assert_eq!(
        output.stdout.get(..8),
        Some(31_614_918_199_316_448_u64.to_le_bytes().as_slice())
    );
}

#[test]
fn output_spanning_several_blocks_continues_each_stream() {
    // 2051 rounds: two full blocks of 1024 rounds and part of a third.
    let bytes = (2051 * 3 * 8).to_string();
    let names = ["gyro", "accelerometer", "pilot"];
    let output = mjk(&[
        "random",
        "7",
        "gyro",
        "accelerometer",
        "pilot",
        "--bytes",
        &bytes,
    ])
    .expect("runs");
    assert!(output.status.success());
    assert_eq!(output.stdout, expected(7, &names, 2051));
}

#[test]
fn only_whole_rounds_are_written() {
    let output = mjk(&["random", "1", "a", "b", "--bytes", "47"]).expect("runs");
    assert_eq!(output.stdout.len(), 32);
    let empty = mjk(&["random", "1", "a", "b", "--bytes", "15"]).expect("runs");
    assert!(empty.status.success() && empty.stdout.is_empty());
}

#[test]
fn malformed_arguments_are_rejected_with_the_usage() {
    let cases: [&[&str]; 6] = [
        &["random"],
        &["random", "42", "gyro"],
        &["random", "42", "--bytes", "8"],
        &["random", "x", "gyro", "--bytes", "8"],
        &["random", "42", "gyro", "--bytes", "-1"],
        &["random", "42", "gyro", "--size", "8"],
    ];
    for args in cases {
        let output = mjk(args).expect("runs");
        assert_eq!(output.status.code(), Some(2), "{args:?}");
        let text = String::from_utf8_lossy(&output.stderr);
        assert!(
            text.contains("usage: mjk random <seed> <name>... --bytes <n>"),
            "{text}"
        );
    }
}

#[test]
fn a_reader_that_stops_early_ends_the_output_quietly() {
    let mut child = Command::new(env!("CARGO_BIN_EXE_mjk"))
        .args(["random", "3", "gyro", "--bytes", "1000000000000"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("starts");
    let mut first = vec![0_u8; 65_536];
    child
        .stdout
        .take()
        .expect("piped")
        .read_exact(&mut first)
        .expect("reads");
    let output = child.wait_with_output().expect("exits");
    assert!(output.status.success());
    assert!(output.stderr.is_empty());
    assert_eq!(first.get(..64), expected(3, &["gyro"], 8).get(..64));
}
