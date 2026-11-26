//! Repository automation. Run tasks with `cargo xtask <task>`.
//!
//! Tasks:
//! - `layering`: checks that workspace crates only depend on what their layer allows.
//! - `equations`: checks equation IDs in `docs/src` and their citations in code.

mod equations;
mod files;
mod layering;
mod metadata;

use std::process::ExitCode;

const USAGE: &str = "usage: cargo xtask <layering | equations>";

fn main() -> ExitCode {
    match std::env::args().nth(1).as_deref() {
        Some("layering") => run_layering(),
        Some("equations") => run_equations(),
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

/// Source trees whose comments may cite equations, as (directory, file extension).
const CITING_SOURCES: &[(&str, &str)] = &[("crates", "rs"), ("viewer/src", "ts")];

fn run_equations() -> ExitCode {
    let root = files::repo_root();
    let docs = match files::read_tree(&root, "docs/src", "md") {
        Ok(docs) => docs,
        Err(error) => {
            eprintln!("error: reading docs/src: {error}");
            return ExitCode::FAILURE;
        }
    };
    let mut sources = Vec::new();
    for (dir, extension) in CITING_SOURCES {
        match files::read_tree(&root, dir, extension) {
            Ok(found) => sources.extend(found),
            Err(error) => {
                eprintln!("error: reading {dir}: {error}");
                return ExitCode::FAILURE;
            }
        }
    }
    let problems = equations::check(&docs, &sources);
    if problems.is_empty() {
        let tagged: usize = docs
            .iter()
            .map(|(_, text)| equations::tags_in_markdown(text).len())
            .sum();
        println!(
            "equations ok: {tagged} tagged equations in {} pages, {} source files checked",
            docs.len(),
            sources.len()
        );
        return ExitCode::SUCCESS;
    }
    for problem in &problems {
        eprintln!("equation problem: {problem}");
    }
    ExitCode::FAILURE
}
