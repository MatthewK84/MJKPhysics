//! `mjk validate`: the validation studies, as the Markdown tables the book includes.
//!
//! Every table is identical on every run and platform. A study that fails, such as an analytic
//! check outside its tolerance, still writes its table, and `mjk` then exits with code 1.

use std::hint::black_box;
use std::path::Path;
use std::time::Instant;

use mjk_sim::validation::analytic::{self, analytic_study};
use mjk_sim::validation::atmosphere::{self, atmosphere_study};
use mjk_sim::validation::determinism::{self, determinism_study};
use mjk_sim::validation::integrators::{self, Method, integrator_study};
use mjk_sim::validation::motor::{self, motor_study};

use crate::args::Study;
use crate::exit::{Exit, Failure};
use crate::files::{self, STDIO};

/// Step size and step count for `validate integrators --timing`.
const TIMING_RUN: (f64, u32) = (0.01, 10_000);

/// A study's table, and whether the study passed.
struct Table {
    markdown: String,
    passed: bool,
}

/// Runs `study`. The integrator study compares two methods and has nothing to fail.
fn table(study: Study) -> Result<Table, Failure> {
    let could_not_run = |error: String| {
        Failure::simulation(format!("the {} study could not run: {error}", study.name()))
    };
    match study {
        Study::Integrators => integrator_study()
            .map(|rows| Table {
                markdown: integrators::markdown(&rows),
                passed: true,
            })
            .map_err(|error| could_not_run(error.to_string())),
        Study::Determinism => determinism_study()
            .map(|(recorded, replayed)| Table {
                markdown: determinism::markdown(&recorded, &replayed),
                passed: determinism::reproduced(&recorded, &replayed),
            })
            .map_err(|error| could_not_run(error.to_string())),
        Study::Atmosphere => atmosphere_study()
            .map(|rows| Table {
                markdown: atmosphere::markdown(&rows),
                passed: atmosphere::passed(&rows),
            })
            .map_err(|error| could_not_run(error.to_string())),
        Study::Analytic => analytic_study()
            .map(|checks| Table {
                markdown: analytic::markdown(&checks),
                passed: analytic::passed(&checks),
            })
            .map_err(|error| could_not_run(error.to_string())),
        Study::Motor => motor_study()
            .map(|checks| Table {
                markdown: motor::markdown(&checks),
                passed: motor::passed(&checks),
            })
            .map_err(|error| could_not_run(error.to_string())),
    }
}

/// `mjk validate <study>`: prints the study's table.
///
/// # Errors
///
/// A failure if the study could not run or its table could not be written.
pub fn one(study: Study) -> Result<Exit, Failure> {
    let table = table(study)?;
    files::write(STDIO, &table.markdown)?;
    if table.passed {
        return Ok(Exit::Success);
    }
    eprintln!("error: the {} study failed", study.name());
    Ok(Exit::Failed)
}

/// `mjk validate [--out <dir>]`: runs every study and writes each table into `out`, listing
/// the files written.
///
/// # Errors
///
/// A failure if a study could not run or a table could not be written.
pub fn all(out: &str) -> Result<Exit, Failure> {
    let dir = Path::new(out);
    files::create_dir(dir)?;
    let mut failed = Vec::new();
    for study in Study::ALL {
        let table = table(study)?;
        let path = dir.join(study.file());
        files::write(&path.to_string_lossy(), &table.markdown)?;
        let verdict = if table.passed { "pass" } else { "FAIL" };
        println!("{verdict}  {:<12} {}", study.name(), path.display());
        if !table.passed {
            failed.push(study.name());
        }
    }
    if failed.is_empty() {
        return Ok(Exit::Success);
    }
    eprintln!("error: failed: {}", failed.join(", "));
    Ok(Exit::Failed)
}

/// `mjk validate integrators --timing`: the wall-clock time per step of each method. Unlike the
/// study, the numbers depend on the machine and vary between runs; build with `--release` for
/// meaningful values.
///
/// # Errors
///
/// A failure if the study's body is not physical, which tests rule out.
pub fn timing() -> Result<Exit, Failure> {
    let (dt_s, steps) = TIMING_RUN;
    println!("Wall-clock time per step, h = {dt_s} s over {steps} steps (machine dependent):");
    for method in Method::ALL {
        let start = Instant::now();
        let result = black_box(method.advance(black_box(dt_s), black_box(steps)));
        let elapsed_ns = start.elapsed().as_secs_f64() * 1e9;
        result.map_err(|error| {
            Failure::simulation(format!(
                "the integrator study body is not physical: {error}"
            ))
        })?;
        let per_step = elapsed_ns / f64::from(steps);
        println!("  {:<24} {per_step:>8.0} ns", method.name());
    }
    Ok(Exit::Success)
}
