//! Exit codes, and the failures that produce them.

use std::fmt;
use std::process::ExitCode;

use mjk_core::EngineError;

/// How `mjk` exits. The codes are part of its interface: scripts and CI test them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exit {
    /// 0: done; every check passed, or the replay matched its record.
    Success,
    /// 1: done, but a check failed or the replay diverged.
    Failed,
    /// 2: the command line is wrong.
    Usage,
    /// 3: an input file is malformed, unsupported or physically implausible.
    Input,
    /// 4: the simulation failed: a numerical or convergence failure, or a built-in scenario
    /// that could not run.
    Simulation,
    /// 5: a file could not be read or written.
    Io,
}

impl Exit {
    /// The process exit code.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::Success => 0,
            Self::Failed => 1,
            Self::Usage => 2,
            Self::Input => 3,
            Self::Simulation => 4,
            Self::Io => 5,
        }
    }

    /// The exit for an engine error: a configuration error is bad input; anything else is a
    /// failed simulation.
    #[must_use]
    pub const fn of(error: &EngineError) -> Self {
        match error {
            EngineError::Config { .. } => Self::Input,
            _ => Self::Simulation,
        }
    }
}

impl From<Exit> for ExitCode {
    fn from(exit: Exit) -> Self {
        Self::from(exit.code())
    }
}

/// A failure to report: what went wrong, and how to exit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Failure {
    /// How to exit.
    pub exit: Exit,
    /// What went wrong, as one line.
    pub message: String,
}

impl Failure {
    /// A failure with `exit` and `message`.
    #[must_use]
    pub fn new(exit: Exit, message: impl Into<String>) -> Self {
        Self {
            exit,
            message: message.into(),
        }
    }

    /// A command-line mistake.
    #[must_use]
    pub fn usage(message: impl Into<String>) -> Self {
        Self::new(Exit::Usage, message)
    }

    /// An engine error, with its message and exit code.
    #[must_use]
    pub fn engine(error: &EngineError) -> Self {
        Self::new(Exit::of(error), error.to_string())
    }

    /// A simulation that could not run, for errors outside [`EngineError`].
    #[must_use]
    pub fn simulation(error: impl fmt::Display) -> Self {
        Self::new(Exit::Simulation, error.to_string())
    }
}
