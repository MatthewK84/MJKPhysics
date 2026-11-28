//! `mjk replay`: flies a recorded run again with its recorded inputs, and checks that every
//! sample, every checkpoint and the final fingerprint come out the same, to the bit.

use mjk_core::{EngineError, FieldPath};
use mjk_sim::config::field_path;
use mjk_sim::run_log::{ReplayError, RunLog, replay_run};

use crate::exit::{Exit, Failure};
use crate::files;

/// Replays the run log at `path`, or on standard input for `-`.
///
/// # Errors
///
/// An I/O failure if the log cannot be read, an input failure naming the field at fault if it
/// is malformed or describes a run this build cannot fly.
pub fn run(path: &str) -> Result<Exit, Failure> {
    let document = files::input_name(path);
    let text = files::read(path)?;
    let log = parse(document, &text)?;
    let report = replay_run(document, &log).map_err(|error| match error {
        ReplayError::Log(error) => Failure::engine(&error),
        ReplayError::Scenario(error) => Failure::simulation(error),
    })?;
    match report.divergence {
        None => {
            println!(
                "replay matches its record: {} ticks, {} checkpoints and {} samples, bit for bit",
                report.ticks, report.checkpoints, report.samples
            );
            Ok(Exit::Success)
        }
        Some(divergence) => {
            println!("replay diverged at {divergence}");
            Ok(Exit::Failed)
        }
    }
}

/// Parses a run log, naming the field at fault on error.
///
/// # Errors
///
/// An input failure for malformed JSON, a missing, unknown or mistyped field, or text after
/// the log.
pub fn parse(document: &str, text: &str) -> Result<RunLog, Failure> {
    let mut deserializer = serde_json::Deserializer::from_str(text);
    let log: RunLog = serde_path_to_error::deserialize(&mut deserializer).map_err(|error| {
        let message = error.inner().to_string();
        let path = field_path(error.path(), &message);
        Failure::engine(&EngineError::config(document, path, message))
    })?;
    deserializer.end().map_err(|error| {
        Failure::engine(&EngineError::config(
            document,
            FieldPath::root(),
            error.to_string(),
        ))
    })?;
    Ok(log)
}
