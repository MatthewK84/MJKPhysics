//! The `mjk` command-line tool.
//!
//! A front end, not part of the engine. `mjk validate <study>` prints a validation table that the
//! book includes: `integrators` ([#12](https://github.com/MatthewK84/MJKPhysics/issues/12)) or
//! `determinism` ([#14](https://github.com/MatthewK84/MJKPhysics/issues/14)). `run` and `replay`, and
//! the rest of the validation suite, arrive with
//! [#16](https://github.com/MatthewK84/MJKPhysics/issues/16) and
//! [#17](https://github.com/MatthewK84/MJKPhysics/issues/17).

use std::hint::black_box;
use std::process::ExitCode;
use std::time::Instant;

use mjk_sim::validation::determinism::{self, determinism_study};
use mjk_sim::validation::integrators::{self, Method, integrator_study};

const USAGE: &str =
    "usage: mjk [--version | --help | validate (integrators [--timing] | determinism)]";

/// Step size and step count for `validate --timing`.
const TIMING_RUN: (f64, u32) = (0.01, 10_000);

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let words: Vec<&str> = args.iter().map(String::as_str).collect();
    match words.as_slice() {
        [] | ["--help" | "-h"] => {
            println!("{USAGE}");
            ExitCode::SUCCESS
        }
        ["--version" | "-V"] => {
            println!("mjk {}", env!("CARGO_PKG_VERSION"));
            ExitCode::SUCCESS
        }
        ["validate", "integrators"] => validate_integrators(),
        ["validate", "integrators", "--timing"] => timing(),
        ["validate", "determinism"] => validate_determinism(),
        ["validate"] => {
            eprintln!("error: name a study: integrators or determinism\n{USAGE}");
            ExitCode::from(2)
        }
        _ => {
            eprintln!("error: unknown arguments `{}`\n{USAGE}", words.join(" "));
            ExitCode::from(2)
        }
    }
}

/// Prints the integrator study table. The output is identical on every run and platform.
fn validate_integrators() -> ExitCode {
    match integrator_study() {
        Ok(rows) => {
            print!("{}", integrators::markdown(&rows));
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("error: the integrator study body is not physical: {error}");
            ExitCode::FAILURE
        }
    }
}

/// Prints the determinism study table: the reference run's checkpoint hashes beside those of a
/// replay of its inputs. The output is identical on every run and platform.
fn validate_determinism() -> ExitCode {
    match determinism_study() {
        Ok((recorded, replayed)) => {
            print!("{}", determinism::markdown(&recorded, &replayed));
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

/// Prints the wall-clock time per step of each method. Unlike the study, the numbers depend on
/// the machine and vary between runs; build with `--release` for meaningful values.
fn timing() -> ExitCode {
    let (dt_s, steps) = TIMING_RUN;
    println!("Wall-clock time per step, h = {dt_s} s over {steps} steps (machine dependent):");
    for method in Method::ALL {
        let start = Instant::now();
        let result = black_box(method.advance(black_box(dt_s), black_box(steps)));
        let elapsed_ns = start.elapsed().as_secs_f64() * 1e9;
        if let Err(error) = result {
            eprintln!("error: the integrator study body is not physical: {error}");
            return ExitCode::FAILURE;
        }
        let per_step = elapsed_ns / f64::from(steps);
        println!("  {:<24} {per_step:>8.0} ns", method.name());
    }
    ExitCode::SUCCESS
}
