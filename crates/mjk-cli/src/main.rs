//! The `mjk` command-line tool ([#17](https://github.com/MatthewK84/MJKPhysics/issues/17)).
//!
//! A front end, not part of the engine: it reads and writes files, and turns engine errors into
//! messages and exit codes. The book's *Command-line tool* page describes every command, the
//! exit codes and the log format.
//!
//! - `mjk validate` runs every validation study and writes the tables the book includes;
//!   `mjk validate <study>` prints one.
//! - `mjk run reference` flies the reference scenario and writes its log; scenario files arrive
//!   with [#28](https://github.com/MatthewK84/MJKPhysics/issues/28).
//! - `mjk replay <log>` flies a recorded run again and checks it, to the bit.
//! - `mjk random` writes named random streams for statistical test suites.

mod args;
mod exit;
mod files;
mod random;
mod replay;
mod run;
mod validate;

use std::process::ExitCode;

use args::{Command, USAGE};
use exit::{Exit, Failure};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let words: Vec<&str> = args.iter().map(String::as_str).collect();
    args::parse(&words)
        .and_then(execute)
        .unwrap_or_else(|failure| report(&failure))
        .into()
}

fn execute(command: Command) -> Result<Exit, Failure> {
    match command {
        Command::Help => {
            println!("{USAGE}");
            Ok(Exit::Success)
        }
        Command::Version => {
            println!("mjk {}", env!("CARGO_PKG_VERSION"));
            Ok(Exit::Success)
        }
        Command::ValidateAll { out } => validate::all(&out),
        Command::Validate(study) => validate::one(study),
        Command::Timing => validate::timing(),
        Command::Run(options) => run::run(&options),
        Command::Replay { log } => replay::run(&log),
        Command::Random(rest) => random::run(&rest),
    }
}

/// Prints the failure, and the usage after a command-line mistake.
fn report(failure: &Failure) -> Exit {
    eprintln!("error: {}", failure.message);
    if failure.exit == Exit::Usage {
        eprintln!("{USAGE}");
    }
    failure.exit
}
