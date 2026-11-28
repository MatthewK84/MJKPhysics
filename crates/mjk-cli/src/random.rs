//! `mjk random`: named random streams, interleaved, for statistical test suites such as `PractRand`
//! ([#14](https://github.com/MatthewK84/MJKPhysics/issues/14)).
//!
//! `mjk random <seed> <name>... --bytes <n>` writes rounds of one 64-bit word from each named
//! stream, in the order named, as little-endian bytes, until the next whole round would exceed
//! `n` bytes. A suite that tests the interleaved sequence sees any correlation between the
//! streams. If the reader stops early, as `PractRand` does at its length limit, the output ends
//! quietly.

use std::io::{self, Write};

use mjk_math::Xoshiro256StarStar;

use crate::exit::{Exit, Failure};

const USAGE: &str = "usage: mjk random <seed> <name>... --bytes <n>";

/// Rounds generated per write.
const ROUNDS_PER_BLOCK: u64 = 1024;

/// Bytes in one 64-bit word.
const WORD_BYTES: usize = 8;

/// The seed, the stream names and the number of whole rounds that fit in the byte limit.
fn parse<'a>(args: &[&'a str]) -> Option<(u64, Vec<&'a str>, u64)> {
    let [seed, names @ .., flag, bytes] = args else {
        return None;
    };
    if *flag != "--bytes" || names.is_empty() {
        return None;
    }
    let round_bytes = u64::try_from(names.len() * WORD_BYTES).ok()?;
    let rounds = bytes.parse::<u64>().ok()? / round_bytes;
    Some((seed.parse().ok()?, names.to_vec(), rounds))
}

/// Writes `rounds` rounds of one word from each stream, in order, as little-endian bytes.
fn write_streams<W: Write>(seed: u64, names: &[&str], rounds: u64, out: &mut W) -> io::Result<()> {
    let mut streams: Vec<Xoshiro256StarStar> = names
        .iter()
        .map(|name| Xoshiro256StarStar::named(seed, name))
        .collect();
    let block_rounds = usize::try_from(ROUNDS_PER_BLOCK).unwrap_or(1);
    let mut buffer: Vec<u8> = Vec::with_capacity(block_rounds * WORD_BYTES * streams.len());
    let mut remaining = rounds;
    for _ in 0..rounds.div_ceil(ROUNDS_PER_BLOCK) {
        let block = remaining.min(ROUNDS_PER_BLOCK);
        buffer.clear();
        for _ in 0..block {
            for stream in &mut streams {
                buffer.extend_from_slice(&stream.next_u64().to_le_bytes());
            }
        }
        out.write_all(&buffer)?;
        remaining -= block;
    }
    out.flush()
}

/// Runs `mjk random` with the arguments after `random`.
///
/// # Errors
///
/// A usage failure for malformed arguments, and an I/O failure if standard output fails for any
/// reason but the reader closing it.
pub fn run(args: &[String]) -> Result<Exit, Failure> {
    let words: Vec<&str> = args.iter().map(String::as_str).collect();
    let Some((seed, names, rounds)) = parse(&words) else {
        return Err(Failure::usage(format!(
            "unknown arguments `random {}`\n{USAGE}",
            words.join(" ")
        )));
    };
    match write_streams(seed, &names, rounds, &mut io::stdout().lock()) {
        Ok(()) => Ok(Exit::Success),
        // The reader has all it wanted.
        Err(error) if error.kind() == io::ErrorKind::BrokenPipe => Ok(Exit::Success),
        Err(error) => Err(Failure::new(
            Exit::Io,
            format!("cannot write standard output: {error}"),
        )),
    }
}
