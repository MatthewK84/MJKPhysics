//! Repository automation. Run tasks with `cargo xtask <task>`.
//!
//! Tasks:
//! - `layering`: checks that workspace crates only depend on what their layer allows.

mod layering;
mod metadata;

use std::process::ExitCode;

const USAGE: &str = "usage: cargo xtask layering";

fn main() -> ExitCode {
    match std::env::args().nth(1).as_deref() {
        Some("layering") => run_layering(),
        Some("--help" | "-h") => {
            println!("{USAGE}");
            ExitCode::SUCCESS
        }
        _ => {
            eprintln!("{USAGE}");
            ExitCode::from(2)
        }
    }
}

fn run_layering() -> ExitCode {
    let packages = match metadata::workspace_packages() {
        Ok(packages) => packages,
        Err(error) => {
            eprintln!("error: {error}");
            return ExitCode::FAILURE;
        }
    };
    let violations = layering::check(&packages);
    if violations.is_empty() {
        println!("layering ok: {} workspace crates checked", packages.len());
        return ExitCode::SUCCESS;
    }
    for violation in &violations {
        eprintln!("layering violation: {violation}");
    }
    ExitCode::FAILURE
}
