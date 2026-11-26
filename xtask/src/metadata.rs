//! Reads workspace packages from `cargo metadata`.

use std::fmt;
use std::path::PathBuf;
use std::process::Command;

use serde_json::Value;

use crate::layering::{DepKind, Dependency, Package};

/// Failure to obtain or interpret `cargo metadata` output.
#[derive(Debug)]
pub enum MetadataError {
    /// `cargo` could not be started.
    Spawn(std::io::Error),
    /// `cargo metadata` exited unsuccessfully; holds its stderr.
    Cargo(String),
    /// The output was not valid JSON.
    Json(serde_json::Error),
    /// The JSON did not have the expected shape.
    Shape(&'static str),
}

impl fmt::Display for MetadataError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Spawn(error) => write!(f, "could not run cargo: {error}"),
            Self::Cargo(stderr) => write!(f, "cargo metadata failed: {stderr}"),
            Self::Json(error) => write!(f, "cargo metadata returned invalid JSON: {error}"),
            Self::Shape(what) => write!(f, "unexpected cargo metadata shape: {what}"),
        }
    }
}

/// Runs `cargo metadata --no-deps` on this workspace and returns its packages.
///
/// # Errors
///
/// Returns [`MetadataError`] if cargo cannot be run, fails, or prints unexpected JSON.
pub fn workspace_packages() -> Result<Vec<Package>, MetadataError> {
    parse_packages(&workspace_metadata()?)
}

/// Runs `cargo metadata --no-deps` on this workspace and returns the raw JSON.
///
/// # Errors
///
/// Returns [`MetadataError`] if cargo cannot be run, fails, or prints invalid JSON.
pub fn workspace_metadata() -> Result<Value, MetadataError> {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("Cargo.toml");
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let output = Command::new(cargo)
        .args([
            "metadata",
            "--format-version",
            "1",
            "--no-deps",
            "--manifest-path",
        ])
        .arg(manifest)
        .output()
        .map_err(MetadataError::Spawn)?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
        return Err(MetadataError::Cargo(stderr));
    }
    serde_json::from_slice(&output.stdout).map_err(MetadataError::Json)
}

/// How to select a target with `cargo rustc`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Selector {
    /// The package's library (`--lib`).
    Lib,
    /// A named binary (`--bin NAME`).
    Bin(String),
}

/// A compilable library or binary target of a workspace package.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Target {
    /// Package name, such as `mjk-math`.
    pub package: String,
    /// Crate name as it appears in symbols, such as `mjk_math`.
    pub crate_name: String,
    /// How to build it.
    pub selector: Selector,
}

/// Lists library and binary targets of every workspace package.
///
/// # Errors
///
/// Returns [`MetadataError::Shape`] when a required field is missing or malformed.
pub fn parse_targets(json: &Value) -> Result<Vec<Target>, MetadataError> {
    let packages = json
        .get("packages")
        .and_then(Value::as_array)
        .ok_or(MetadataError::Shape("missing `packages` array"))?;
    let mut targets = Vec::new();
    for package in packages {
        let name = string_field(package, "name")?;
        let entries = package
            .get("targets")
            .and_then(Value::as_array)
            .ok_or(MetadataError::Shape("package without `targets` array"))?;
        targets.extend(entries.iter().filter_map(|entry| target_of(&name, entry)));
    }
    Ok(targets)
}

fn target_of(package: &str, entry: &Value) -> Option<Target> {
    let kinds = entry.get("kind").and_then(Value::as_array)?;
    let name = entry.get("name").and_then(Value::as_str)?;
    let is_kind = |wanted: &str| kinds.iter().any(|kind| kind.as_str() == Some(wanted));
    let selector = if is_kind("lib") || is_kind("cdylib") {
        Selector::Lib
    } else if is_kind("bin") {
        Selector::Bin(name.to_owned())
    } else {
        return None;
    };
    Some(Target {
        package: package.to_owned(),
        crate_name: name.replace('-', "_"),
        selector,
    })
}

/// Extracts packages and their dependencies from `cargo metadata` JSON.
///
/// # Errors
///
/// Returns [`MetadataError::Shape`] when a required field is missing or malformed.
pub fn parse_packages(json: &Value) -> Result<Vec<Package>, MetadataError> {
    json.get("packages")
        .and_then(Value::as_array)
        .ok_or(MetadataError::Shape("missing `packages` array"))?
        .iter()
        .map(parse_package)
        .collect()
}

fn parse_package(value: &Value) -> Result<Package, MetadataError> {
    let dependencies = value
        .get("dependencies")
        .and_then(Value::as_array)
        .ok_or(MetadataError::Shape("package without `dependencies` array"))?
        .iter()
        .map(parse_dependency)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Package {
        name: string_field(value, "name")?,
        dependencies,
    })
}

fn parse_dependency(value: &Value) -> Result<Dependency, MetadataError> {
    let kind = match value.get("kind").and_then(Value::as_str) {
        None => DepKind::Normal,
        Some("build") => DepKind::Build,
        Some("dev") => DepKind::Dev,
        Some(_) => return Err(MetadataError::Shape("unknown dependency kind")),
    };
    Ok(Dependency {
        name: string_field(value, "name")?,
        kind,
        internal: value.get("path").is_some_and(|path| !path.is_null()),
    })
}

fn string_field(value: &Value, field: &'static str) -> Result<String, MetadataError> {
    value
        .get(field)
        .and_then(Value::as_str)
        .map(str::to_owned)
        .ok_or(MetadataError::Shape(field))
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{parse_packages, workspace_packages};
    use crate::layering::{DepKind, check};

    #[test]
    fn parses_kinds_and_path_dependencies() {
        let json = json!({ "packages": [{
            "name": "mjk-aero",
            "dependencies": [
                { "name": "mjk-math", "kind": null, "path": "/repo/crates/mjk-math" },
                { "name": "libm", "kind": null },
                { "name": "proptest", "kind": "dev" }
            ]
        }]});
        let packages = parse_packages(&json).expect("fixture parses");
        let deps = &packages[0].dependencies;
        assert!(deps[0].internal && deps[0].kind == DepKind::Normal);
        assert!(!deps[1].internal);
        assert_eq!(deps[2].kind, DepKind::Dev);
    }

    #[test]
    fn rejects_missing_packages_array() {
        assert!(parse_packages(&json!({})).is_err());
    }

    #[test]
    fn this_workspace_obeys_the_layering_rules() {
        let packages = workspace_packages().expect("cargo metadata runs");
        assert!(packages.len() >= 11, "expected every workspace crate");
        assert_eq!(check(&packages), vec![]);
    }
}
