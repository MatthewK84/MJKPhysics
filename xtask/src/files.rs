//! Collects repository files for checks, without recursion.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

/// Directory names never descended into.
const SKIPPED_DIRS: &[&str] = &["target", "node_modules", "book", "dist"];

/// Root of the repository (the parent of the `xtask` crate).
#[must_use]
pub fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

/// Returns `(path relative to root, contents)` for every file under `dir` with extension
/// `extension`, sorted by path. Hidden and build directories are skipped.
///
/// # Errors
///
/// Returns any I/O error from reading directories or files.
pub fn read_tree(root: &Path, dir: &str, extension: &str) -> io::Result<Vec<(String, String)>> {
    let mut pending = vec![root.join(dir)];
    let mut found = Vec::new();
    // bound: every directory is visited once; symlinks are never followed, so the walk ends.
    while let Some(current) = pending.pop() {
        for entry in fs::read_dir(&current)? {
            let entry = entry?;
            let kind = entry.file_type()?;
            let path = entry.path();
            if kind.is_dir() && !is_skipped(&path) {
                pending.push(path);
            } else if kind.is_file() && path.extension().is_some_and(|ext| ext == extension) {
                found.push((relative(root, &path), fs::read_to_string(&path)?));
            }
        }
    }
    found.sort();
    Ok(found)
}

fn is_skipped(path: &Path) -> bool {
    path.file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| name.starts_with('.') || SKIPPED_DIRS.contains(&name))
}

fn relative(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}
