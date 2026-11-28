//! The command line, parsed into a [`Command`].

use mjk_sim::run_log::REFERENCE;
use mjk_sim::validation::determinism::MASTER_SEED;

use crate::exit::Failure;
use crate::files::STDIO;

/// What `mjk --help` prints, and what follows a usage error.
pub const USAGE: &str = "\
usage: mjk --version | --help
       mjk validate [--out <dir>]
       mjk validate (integrators [--timing] | determinism | atmosphere | analytic)
       mjk run reference [--seed <n>] [--log <file.json>] [--csv <file.csv>]
       mjk replay <file.json>
       mjk random <seed> <name>... --bytes <n>
A file named - is standard input or output.";

/// Where `mjk validate` writes the tables: the book's validation pages, from the repository root.
pub const DEFAULT_OUT: &str = "docs/src/validation";

/// A validation study.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Study {
    /// RK4 against RKMK4 on a tumbling body (#12).
    Integrators,
    /// The reference scenario's fingerprints beside a replay of its inputs (#14).
    Determinism,
    /// The atmosphere model beside the 1976 standard's tables (#15).
    Atmosphere,
    /// The simulation loop against closed-form solutions (#16).
    Analytic,
}

impl Study {
    /// Every study, in the order `mjk validate` runs them.
    pub const ALL: [Self; 4] = [
        Self::Integrators,
        Self::Determinism,
        Self::Atmosphere,
        Self::Analytic,
    ];

    /// Its name on the command line.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Integrators => "integrators",
            Self::Determinism => "determinism",
            Self::Atmosphere => "atmosphere",
            Self::Analytic => "analytic",
        }
    }

    /// The file its table is written to.
    #[must_use]
    pub const fn file(self) -> &'static str {
        match self {
            Self::Integrators => "integrator-study.md",
            Self::Determinism => "determinism.md",
            Self::Atmosphere => "atmosphere.md",
            Self::Analytic => "analytic.md",
        }
    }

    fn named(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|study| study.name() == name)
    }
}

/// Options of `mjk run`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RunOptions {
    /// The scenario: only `reference` so far.
    pub scenario: String,
    /// The master seed.
    pub seed: u64,
    /// Where to write the run log as JSON.
    pub log: Option<String>,
    /// Where to write the state log as CSV.
    pub csv: Option<String>,
}

/// A parsed command line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Command {
    /// Print the usage.
    Help,
    /// Print the version.
    Version,
    /// Run every study and write each table into a directory.
    ValidateAll {
        /// The directory.
        out: String,
    },
    /// Run one study and print its table.
    Validate(Study),
    /// Time each integration method.
    Timing,
    /// Fly a scenario.
    Run(RunOptions),
    /// Fly a recorded run again and compare.
    Replay {
        /// The run log.
        log: String,
    },
    /// Write random streams.
    Random(Vec<String>),
}

/// Parses the arguments after the program name.
///
/// # Errors
///
/// A usage failure saying what is wrong.
pub fn parse(words: &[&str]) -> Result<Command, Failure> {
    match words {
        [] | ["--help" | "-h"] => Ok(Command::Help),
        ["--version" | "-V"] => Ok(Command::Version),
        ["validate", rest @ ..] => validate(rest),
        ["run", rest @ ..] => run(rest).map(Command::Run),
        ["replay", log] => Ok(Command::Replay {
            log: (*log).to_owned(),
        }),
        ["replay", ..] => Err(Failure::usage("name one run log to replay")),
        ["random", rest @ ..] => Ok(Command::Random(
            rest.iter().map(|word| (*word).to_owned()).collect(),
        )),
        _ => Err(Failure::usage(format!(
            "unknown arguments `{}`",
            words.join(" ")
        ))),
    }
}

fn validate(words: &[&str]) -> Result<Command, Failure> {
    match words {
        [] => Ok(Command::ValidateAll {
            out: DEFAULT_OUT.to_owned(),
        }),
        ["--out", dir] => Ok(Command::ValidateAll {
            out: (*dir).to_owned(),
        }),
        ["--out"] => Err(Failure::usage("--out takes a directory")),
        ["integrators", "--timing"] => Ok(Command::Timing),
        [name] => Study::named(name).map(Command::Validate).ok_or_else(|| {
            Failure::usage(format!(
                "unknown study `{name}`: the studies are integrators, determinism, atmosphere \
                 and analytic"
            ))
        }),
        _ => Err(Failure::usage(format!(
            "unknown arguments to validate: `{}`",
            words.join(" ")
        ))),
    }
}

fn run(words: &[&str]) -> Result<RunOptions, Failure> {
    let Some((scenario, rest)) = words.split_first() else {
        return Err(Failure::usage(format!("name a scenario: `{REFERENCE}`")));
    };
    let mut options = RunOptions {
        scenario: (*scenario).to_owned(),
        seed: MASTER_SEED,
        log: None,
        csv: None,
    };
    for pair in rest.chunks(2) {
        match pair {
            ["--seed", value] => options.seed = seed(value)?,
            ["--log", path] => options.log = Some((*path).to_owned()),
            ["--csv", path] => options.csv = Some((*path).to_owned()),
            _ => {
                let option = pair.join(" ");
                return Err(Failure::usage(format!("unknown option to run: `{option}`")));
            }
        }
    }
    if options.log.as_deref() == Some(STDIO) && options.csv.as_deref() == Some(STDIO) {
        return Err(Failure::usage(
            "only one of --log and --csv can write to standard output",
        ));
    }
    Ok(options)
}

fn seed(value: &str) -> Result<u64, Failure> {
    value.parse().map_err(|_| {
        Failure::usage(format!(
            "--seed takes a whole number from 0 to {}; found `{value}`",
            u64::MAX
        ))
    })
}
