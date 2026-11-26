//! The `mjk` command-line tool.
//!
//! A front end, not part of the engine. Commands (`run`, `validate`, `replay`) arrive
//! with [#17](https://github.com/MatthewK84/MJKPhysics/issues/17); until then the tool only reports its version.

use std::process::ExitCode;

const USAGE: &str = "usage: mjk [--version | --help]";

fn main() -> ExitCode {
    let first = std::env::args().nth(1);
    match first.as_deref() {
        None | Some("--help" | "-h") => {
            println!("{USAGE}");
            ExitCode::SUCCESS
        }
        Some("--version" | "-V") => {
            println!("mjk {}", env!("CARGO_PKG_VERSION"));
            ExitCode::SUCCESS
        }
        Some(other) => {
            eprintln!("error: unknown argument `{other}`\n{USAGE}");
            ExitCode::from(2)
        }
    }
}
