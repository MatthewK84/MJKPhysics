//! Builds unoptimized LLVM IR for workspace targets, for the recursion check.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::metadata::{Selector, Target};

/// Compiles `target` with one codegen unit and returns its `.ll` file.
///
/// Builds go to `target/standards` so they never disturb the normal build cache.
///
/// # Errors
///
/// Returns a description when cargo cannot run, the build fails, or no IR is found.
pub fn emit_ir(root: &Path, target: &Target) -> Result<PathBuf, String> {
    let target_dir = root.join("target").join("standards");
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let mut command = Command::new(cargo);
    command
        .current_dir(root)
        .env("CARGO_TARGET_DIR", &target_dir)
        .env("CARGO_INCREMENTAL", "0")
        .args(["rustc", "--quiet", "--locked", "--package", &target.package]);
    match &target.selector {
        Selector::Lib => command.arg("--lib"),
        Selector::Bin(name) => command.args(["--bin", name]),
    };
    command.args(["--", "-C", "codegen-units=1", "--emit=llvm-ir,link"]);
    let status = command
        .status()
        .map_err(|error| format!("could not run cargo: {error}"))?;
    if !status.success() {
        return Err(format!("building IR for `{}` failed", target.package));
    }
    newest_ir_file(&target_dir.join("debug").join("deps"), &target.crate_name)
        .ok_or_else(|| format!("no LLVM IR was found for `{}`", target.package))
}

/// The most recently written `.ll` file for `crate_name`: `NAME-HASH.ll`, or `NAME.ll` for
/// `cdylib` crates, which rustc writes without a hash.
///
/// When nothing changed, cargo skips the build and the newest file is still current; when the
/// source or flags change, cargo rebuilds and the new file is the newest.
fn newest_ir_file(dir: &Path, crate_name: &str) -> Option<PathBuf> {
    let prefix = format!("{crate_name}-");
    let exact = format!("{crate_name}.ll");
    fs::read_dir(dir)
        .ok()?
        .flatten()
        .filter_map(|entry| {
            let path = entry.path();
            let name = path.file_name()?.to_str()?.to_owned();
            let is_ir = path.extension().is_some_and(|ext| ext == "ll");
            let named = name == exact || (name.starts_with(&prefix) && is_ir);
            let modified = entry.metadata().and_then(|meta| meta.modified()).ok()?;
            named.then_some((modified, path))
        })
        .max_by_key(|(modified, _)| *modified)
        .map(|(_, path)| path)
}
