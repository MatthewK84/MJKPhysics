//! `mjk run`: flies a scenario and writes its log.
//!
//! Scenario files arrive with [#28](https://github.com/MatthewK84/MJKPhysics/issues/28); until
//! then the only scenario is the built-in reference scenario of
//! [#14](https://github.com/MatthewK84/MJKPhysics/issues/14).

use mjk_sim::log::write_csv;
use mjk_sim::run_log::{REFERENCE, RunLog, record_reference};

use crate::args::RunOptions;
use crate::exit::{Exit, Failure};
use crate::files::{self, STDIO};

/// Flies the scenario, writes the requested logs, and summarizes the run on standard output, or
/// on standard error when a log goes to standard output.
///
/// # Errors
///
/// A usage failure for any scenario but `reference`, and an I/O failure if a log cannot be
/// written.
pub fn run(options: &RunOptions) -> Result<Exit, Failure> {
    if options.scenario != REFERENCE {
        return Err(Failure::usage(format!(
            "cannot run `{}`: scenario files arrive with #28 \
             (https://github.com/MatthewK84/MJKPhysics/issues/28); the only built-in scenario is \
             `{REFERENCE}`",
            options.scenario
        )));
    }
    let log = record_reference(options.seed).map_err(Failure::simulation)?;
    if let Some(path) = &options.log {
        files::write(path, &json(&log)?)?;
    }
    if let Some(path) = &options.csv {
        files::write(path, &csv(&log)?)?;
    }
    let summary = summary(&log);
    let stdout_taken = [&options.log, &options.csv]
        .into_iter()
        .any(|path| path.as_deref() == Some(STDIO));
    if stdout_taken {
        eprint!("{summary}");
    } else {
        print!("{summary}");
    }
    Ok(Exit::Success)
}

/// The run log as one line of JSON.
///
/// # Errors
///
/// A simulation failure if the log cannot be serialized, which its types rule out.
pub fn json(log: &RunLog) -> Result<String, Failure> {
    let mut text = serde_json::to_string(log).map_err(Failure::simulation)?;
    text.push('\n');
    Ok(text)
}

fn csv(log: &RunLog) -> Result<String, Failure> {
    let mut text = String::new();
    write_csv(&log.channels, &mut text).map_err(Failure::simulation)?;
    Ok(text)
}

fn summary(log: &RunLog) -> String {
    format!(
        "{} run, seed {}: {} ticks at {} Hz, {} inputs, {} checkpoints, final fingerprint {}\n",
        log.scenario,
        log.seed,
        log.ticks,
        log.physics_hz,
        log.inputs.len(),
        log.checkpoints.len(),
        log.final_hash
    )
}
