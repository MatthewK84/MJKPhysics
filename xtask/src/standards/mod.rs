//! `cargo xtask standards`: the Power of Ten checks that clippy cannot express (#88).
//!
//! - Rule 1, no recursion: cycles in the compiled call graph of every workspace target except
//!   xtask.
//! - Rules 2, 8 and 9: bounded loops, no macros or conditional compilation, no trait objects or
//!   function pointers, in every workspace crate.
//! - Rule 5: assertion density per crate, reported but not enforced.

mod callgraph;
mod ir_build;
mod source;

use std::fs;
use std::path::Path;

use crate::files;
use crate::layering::ENGINE;
use crate::metadata::{self, Target};

/// Workspace crates outside the engine, held to the same source rules.
const OTHER_SOURCES: &[(&str, &str)] = &[
    ("mjk-cli", "crates/mjk-cli/src"),
    ("mjk-wasm", "crates/mjk-wasm/src"),
    ("xtask", "xtask/src"),
];

/// Packages exempt from the recursion check. xtask is repository tooling, not shipped
/// software, and walks syntax trees with syn's recursive visitor (CONTRIBUTING.md).
const RECURSION_EXEMPT: &[&str] = &["xtask"];

/// Runs every check and prints a report.
///
/// # Errors
///
/// Returns a description when a file cannot be read or parsed, or a target cannot be built.
/// Violations are not errors: their count is returned.
pub fn run(root: &Path) -> Result<usize, String> {
    let source_problems = run_source_checks(root)?;
    let recursion_problems = run_recursion_check(root)?;
    Ok(source_problems + recursion_problems)
}

/// Every workspace source tree, as (crate, directory).
fn source_trees() -> Vec<(String, String)> {
    let engine = ENGINE
        .iter()
        .map(|name| ((*name).to_owned(), format!("crates/{name}/src")));
    let others = OTHER_SOURCES
        .iter()
        .map(|(name, dir)| ((*name).to_owned(), (*dir).to_owned()));
    engine.chain(others).collect()
}

/// Checks one source tree, returning its findings and counts.
fn check_tree(
    root: &Path,
    dir: &str,
) -> Result<(Vec<source::Finding>, source::AssertionCount), String> {
    let sources =
        files::read_tree(root, dir, "rs").map_err(|error| format!("reading {dir}: {error}"))?;
    let mut findings = Vec::new();
    let mut counts = source::AssertionCount::default();
    for (path, text) in &sources {
        let report =
            source::check_source(path, text).map_err(|error| format!("{path}: {error}"))?;
        counts.functions += report.counts.functions;
        counts.assertions += report.counts.assertions;
        findings.extend(report.findings);
    }
    Ok((findings, counts))
}

fn run_source_checks(root: &Path) -> Result<usize, String> {
    let mut findings = Vec::new();
    println!("rule 5 (assertion density), reported only:");
    for (crate_name, dir) in source_trees() {
        let (found, counts) = check_tree(root, &dir)?;
        findings.extend(found);
        println!(
            "  {crate_name:<14} {:>4} functions  {:>4} assertions",
            counts.functions, counts.assertions
        );
    }
    findings.sort();
    for finding in &findings {
        eprintln!("{finding}");
    }
    Ok(findings.len())
}

fn run_recursion_check(root: &Path) -> Result<usize, String> {
    let json = metadata::workspace_metadata().map_err(|error| error.to_string())?;
    let targets: Vec<Target> = metadata::parse_targets(&json)
        .map_err(|error| error.to_string())?
        .into_iter()
        .filter(|target| !RECURSION_EXEMPT.contains(&target.package.as_str()))
        .collect();
    let prefixes: Vec<String> = targets
        .iter()
        .map(|target| format!("{}::", target.crate_name))
        .collect();
    let is_ours = |name: &str| {
        let bare = name.trim_start_matches('<');
        prefixes
            .iter()
            .any(|prefix| bare.starts_with(prefix.as_str()))
    };
    let mut problems = 0;
    for target in &targets {
        let path = ir_build::emit_ir(root, target)?;
        let text =
            fs::read_to_string(&path).map_err(|error| format!("{}: {error}", path.display()))?;
        for cycle in callgraph::CallGraph::from_ir(&text).recursion(is_ours) {
            eprintln!(
                "{}: rule 1 (no recursion): {}",
                target.package,
                cycle.join(" → ")
            );
            problems += 1;
        }
        println!("rule 1 (no recursion): checked `{}`", target.package);
    }
    Ok(problems)
}
