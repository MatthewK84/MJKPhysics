//! Recorded runs ([#17](https://github.com/MatthewK84/MJKPhysics/issues/17)): what `mjk run`
//! writes and `mjk replay` reads.
//!
//! A [`RunLog`] holds what it takes to fly a run again and check it: the scenario, its seed and
//! rates, each input with the tick it was applied at, the state fingerprint at every
//! checkpoint, and the state log. Front ends write it as JSON. [`replay_run`] flies the scenario
//! again with the recorded inputs and reports the first tick at which the two runs differ.
//!
//! The only scenario so far is the built-in reference scenario of
//! [#14](https://github.com/MatthewK84/MJKPhysics/issues/14); scenario files arrive with
//! [#28](https://github.com/MatthewK84/MJKPhysics/issues/28).

use std::fmt;

use mjk_core::{EngineError, FieldPath};
use mjk_dynamics::{Checkpoint, InputLog, RecordError};
use mjk_math::Vec3;
use serde::{Deserialize, Serialize};

use crate::log::{ChannelLog, ColumnLog, Sample};
use crate::validation::determinism::{
    self, CHANNELS, PHYSICS_HZ, PilotCommand, SAMPLE_HZ, ScenarioError, TICKS,
};

/// The format version this build writes, and the only one it reads.
pub const FORMAT_VERSION: u32 = 1;

/// The name of the built-in reference scenario.
pub const REFERENCE: &str = "reference";

/// A recorded run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunLog {
    /// Format version of the log.
    pub format_version: u32,
    /// The scenario flown.
    pub scenario: String,
    /// The master seed of every random stream.
    pub seed: u64,
    /// Physics steps per second.
    pub physics_hz: u32,
    /// Physics steps in the run.
    pub ticks: u64,
    /// Rows of the state log per second.
    pub sample_hz: u32,
    /// The inputs applied, in order.
    pub inputs: Vec<LoggedCommand>,
    /// The state fingerprint at each checkpoint.
    pub checkpoints: Vec<LoggedCheckpoint>,
    /// The state fingerprint at the end of the run, 16 hexadecimal digits.
    pub final_hash: String,
    /// The state log, one entry per channel.
    pub channels: Vec<ChannelLog>,
}

/// A pilot command and the tick it was applied at.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoggedCommand {
    /// The tick it was applied at.
    pub tick: u64,
    /// Commanded body rates, rad/s.
    pub rate_rad_s: [f64; 3],
    /// Commanded thrust along body −z, N.
    pub thrust_n: f64,
}

/// A state fingerprint at one tick.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoggedCheckpoint {
    /// The tick.
    pub tick: u64,
    /// The fingerprint, 16 hexadecimal digits.
    pub hash: String,
}

/// Flies the reference scenario seeded with `seed` and records it.
///
/// # Errors
///
/// A [`ScenarioError`], which the reference configuration never produces.
pub fn record_reference(seed: u64) -> Result<RunLog, ScenarioError> {
    let (run, inputs, log) = determinism::record_logged(seed)?;
    let commands = inputs
        .events()
        .iter()
        .map(|event| LoggedCommand {
            tick: event.tick,
            rate_rad_s: event.input.rate_rad_s.to_array(),
            thrust_n: event.input.thrust_n,
        })
        .collect();
    let checkpoints = run
        .checkpoints
        .iter()
        .map(|checkpoint| LoggedCheckpoint {
            tick: checkpoint.tick,
            hash: hex(checkpoint.hash),
        })
        .collect();
    Ok(RunLog {
        format_version: FORMAT_VERSION,
        scenario: REFERENCE.to_owned(),
        seed,
        physics_hz: PHYSICS_HZ,
        ticks: TICKS,
        sample_hz: SAMPLE_HZ,
        inputs: commands,
        checkpoints,
        final_hash: hex(run.final_hash),
        channels: log.channel_logs(),
    })
}

/// A fingerprint as 16 hexadecimal digits.
#[must_use]
pub fn hex(hash: u64) -> String {
    format!("{hash:016x}")
}

/// Where a replay first differed from its record.
#[derive(Clone, Debug, PartialEq)]
pub enum Divergence {
    /// A sample of the state log differs.
    Sample {
        /// The tick of the sample.
        tick: u64,
        /// The channel.
        channel: String,
        /// The recorded value.
        recorded: f64,
        /// The replayed value.
        replayed: f64,
    },
    /// A checkpoint's fingerprint differs.
    Checkpoint {
        /// The tick of the checkpoint.
        tick: u64,
        /// The recorded fingerprint.
        recorded: u64,
        /// The replayed fingerprint.
        replayed: u64,
    },
    /// The fingerprint at the end of the run differs.
    Final {
        /// The last tick.
        tick: u64,
        /// The recorded fingerprint.
        recorded: u64,
        /// The replayed fingerprint.
        replayed: u64,
    },
}

impl Divergence {
    /// The tick at which the runs differ.
    #[must_use]
    pub const fn tick(&self) -> u64 {
        match self {
            Self::Sample { tick, .. }
            | Self::Checkpoint { tick, .. }
            | Self::Final { tick, .. } => *tick,
        }
    }
}

impl fmt::Display for Divergence {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Sample {
                tick,
                channel,
                recorded,
                replayed,
            } => write!(
                f,
                "tick {tick}: {channel} was {recorded:?} and is {replayed:?} on replay"
            ),
            Self::Checkpoint {
                tick,
                recorded,
                replayed,
            } => write!(
                f,
                "tick {tick}: the checkpoint fingerprint was {} and is {} on replay",
                hex(*recorded),
                hex(*replayed)
            ),
            Self::Final {
                tick,
                recorded,
                replayed,
            } => write!(
                f,
                "tick {tick}: the final fingerprint was {} and is {} on replay",
                hex(*recorded),
                hex(*replayed)
            ),
        }
    }
}

/// What a replay found.
#[derive(Clone, Debug, PartialEq)]
pub struct ReplayReport {
    /// Physics steps flown.
    pub ticks: u64,
    /// Checkpoints compared.
    pub checkpoints: usize,
    /// Samples compared: rows times channels.
    pub samples: usize,
    /// The first difference, or `None` if the replay reproduced the record to the bit.
    pub divergence: Option<Divergence>,
}

/// Why a log could not be replayed.
#[derive(Clone, Debug)]
pub enum ReplayError {
    /// The log is malformed or describes a run this build cannot fly.
    Log(EngineError),
    /// The scenario could not run, which the reference configuration never causes.
    Scenario(ScenarioError),
}

impl fmt::Display for ReplayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Log(error) => write!(f, "{error}"),
            Self::Scenario(error) => write!(f, "{error}"),
        }
    }
}

impl From<EngineError> for ReplayError {
    fn from(error: EngineError) -> Self {
        Self::Log(error)
    }
}

/// Flies the run recorded in `log` again with its recorded inputs and compares the two: every
/// sample, every checkpoint and the final fingerprint. `document` names the log in errors.
///
/// # Errors
///
/// [`ReplayError::Log`] naming the field at fault if the log is malformed or describes a run
/// this build cannot fly: another format version, scenario, rate or length, other channels,
/// inputs out of order, or a fingerprint that is not 16 hexadecimal digits.
pub fn replay_run(document: &str, log: &RunLog) -> Result<ReplayReport, ReplayError> {
    check_supported(document, log)?;
    let recorded_checkpoints = checkpoints(document, &log.checkpoints)?;
    let recorded_final = parse_hash(
        document,
        FieldPath::root().key("final_hash"),
        &log.final_hash,
    )?;
    let inputs = input_log(document, &log.inputs)?;
    let (run, replayed) =
        determinism::replay_logged(log.seed, &inputs).map_err(ReplayError::Scenario)?;
    check_lengths(
        document,
        log,
        &recorded_checkpoints,
        &run.checkpoints,
        &replayed,
    )?;
    let candidates = [
        first_sample_difference(&log.channels, &replayed),
        first_checkpoint_difference(&recorded_checkpoints, &run.checkpoints),
        (recorded_final != run.final_hash).then_some(Divergence::Final {
            tick: log.ticks,
            recorded: recorded_final,
            replayed: run.final_hash,
        }),
    ];
    Ok(ReplayReport {
        ticks: log.ticks,
        checkpoints: recorded_checkpoints.len(),
        samples: replayed.rows() * CHANNELS.len(),
        divergence: candidates
            .into_iter()
            .flatten()
            .min_by_key(Divergence::tick),
    })
}

/// The header and channels must describe the reference scenario as this build flies it.
fn check_supported(document: &str, log: &RunLog) -> Result<(), EngineError> {
    let expected = [
        (
            "format_version",
            log.format_version.to_string(),
            FORMAT_VERSION.to_string(),
        ),
        ("scenario", log.scenario.clone(), REFERENCE.to_owned()),
        (
            "physics_hz",
            log.physics_hz.to_string(),
            PHYSICS_HZ.to_string(),
        ),
        ("ticks", log.ticks.to_string(), TICKS.to_string()),
        (
            "sample_hz",
            log.sample_hz.to_string(),
            SAMPLE_HZ.to_string(),
        ),
    ];
    for (field, found, wanted) in expected {
        if found != wanted {
            let reason = format!("this build replays only {wanted}; found {found}");
            return Err(EngineError::config(
                document,
                FieldPath::root().key(field),
                reason,
            ));
        }
    }
    check_channels(document, &log.channels)
}

fn check_channels(document: &str, channels: &[ChannelLog]) -> Result<(), EngineError> {
    let path = FieldPath::root().key("channels");
    if channels.len() != CHANNELS.len() {
        let reason = format!(
            "the reference scenario logs {} channels; found {}",
            CHANNELS.len(),
            channels.len()
        );
        return Err(EngineError::config(document, path, reason));
    }
    let mismatch = channels
        .iter()
        .zip(CHANNELS)
        .position(|(found, wanted)| found.name != wanted.name || found.unit != wanted.unit);
    mismatch.map_or(Ok(()), |index| {
        let wanted = CHANNELS.get(index).map_or("", |channel| channel.name);
        let reason = format!("expected channel `{wanted}` with its unit here");
        Err(EngineError::config(document, path.index(index), reason))
    })
}

fn checkpoints(
    document: &str,
    logged: &[LoggedCheckpoint],
) -> Result<Vec<Checkpoint>, EngineError> {
    logged
        .iter()
        .enumerate()
        .map(|(index, checkpoint)| {
            let path = FieldPath::root()
                .key("checkpoints")
                .index(index)
                .key("hash");
            let hash = parse_hash(document, path, &checkpoint.hash)?;
            Ok(Checkpoint {
                tick: checkpoint.tick,
                hash,
            })
        })
        .collect()
}

fn parse_hash(document: &str, path: FieldPath, text: &str) -> Result<u64, EngineError> {
    let digits = text.len() == 16 && text.bytes().all(|byte| byte.is_ascii_hexdigit());
    let parsed = u64::from_str_radix(text, 16).ok().filter(|_| digits);
    parsed.ok_or_else(|| {
        let reason = format!("a fingerprint is 16 hexadecimal digits; found \"{text}\"");
        EngineError::config(document, path, reason)
    })
}

fn input_log(
    document: &str,
    commands: &[LoggedCommand],
) -> Result<InputLog<PilotCommand>, EngineError> {
    let mut inputs = InputLog::with_capacity(commands.len());
    for (index, command) in commands.iter().enumerate() {
        let input = PilotCommand {
            rate_rad_s: Vec3::from_array(command.rate_rad_s),
            thrust_n: command.thrust_n,
        };
        inputs
            .record(command.tick, input)
            .map_err(|error| input_error(document, index, error))?;
    }
    Ok(inputs)
}

fn input_error(document: &str, index: usize, error: RecordError) -> EngineError {
    let path = FieldPath::root().key("inputs").index(index).key("tick");
    EngineError::config(document, path, error.to_string())
}

/// The record must have as many checkpoints and samples as the replay: a shorter record is a
/// truncated file, not a divergence.
fn check_lengths(
    document: &str,
    log: &RunLog,
    recorded: &[Checkpoint],
    replayed: &[Checkpoint],
    replayed_log: &ColumnLog,
) -> Result<(), EngineError> {
    if recorded.len() != replayed.len() {
        let reason = format!(
            "the run takes {} checkpoints; found {}",
            replayed.len(),
            recorded.len()
        );
        return Err(EngineError::config(
            document,
            FieldPath::root().key("checkpoints"),
            reason,
        ));
    }
    let short = log
        .channels
        .iter()
        .position(|channel| channel.values.len() != replayed_log.rows());
    short.map_or(Ok(()), |index| {
        let reason = format!("the run logs {} samples per channel", replayed_log.rows());
        let path = FieldPath::root().key("channels").index(index).key("values");
        Err(EngineError::config(document, path, reason))
    })
}

/// The earliest differing sample, by row and then by channel.
fn first_sample_difference(recorded: &[ChannelLog], replayed: &ColumnLog) -> Option<Divergence> {
    let ticks_per_row = u64::from(PHYSICS_HZ / SAMPLE_HZ);
    let differences = recorded.iter().enumerate().filter_map(|(index, channel)| {
        let column = replayed.column(index)?;
        let row = channel
            .values
            .iter()
            .zip(column)
            .position(|(logged, flown)| *logged != Sample(*flown))?;
        Some((row, index))
    });
    let (row, index) = differences.min()?;
    let channel = recorded.get(index)?;
    Some(Divergence::Sample {
        tick: u64::try_from(row).unwrap_or(u64::MAX) * ticks_per_row,
        channel: channel.name.clone(),
        recorded: channel.values.get(row)?.0,
        replayed: *replayed.column(index)?.get(row)?,
    })
}

fn first_checkpoint_difference(
    recorded: &[Checkpoint],
    replayed: &[Checkpoint],
) -> Option<Divergence> {
    recorded
        .iter()
        .zip(replayed)
        .find(|(recorded, replayed)| recorded != replayed)
        .map(|(recorded, replayed)| Divergence::Checkpoint {
            tick: recorded.tick.min(replayed.tick),
            recorded: recorded.hash,
            replayed: replayed.hash,
        })
}
