//! Crate layering rules and the check that enforces them.
//!
//! Every workspace crate must appear in [`RULES`] with the internal crates it may depend
//! on. Engine crates may additionally use only the external crates in
//! [`ENGINE_EXTERNAL_ALLOWED`] (development dependencies such as test tools are exempt).

use std::fmt;

/// How a dependency is used, as reported by `cargo metadata`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DepKind {
    /// A regular dependency.
    Normal,
    /// A build-script dependency.
    Build,
    /// A test, example or benchmark dependency.
    Dev,
}

/// One declared dependency of a workspace package.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Dependency {
    /// Package name of the dependency.
    pub name: String,
    /// How the dependency is used.
    pub kind: DepKind,
    /// True when the dependency is another crate in this workspace (a path dependency).
    pub internal: bool,
}

/// A workspace package and its declared dependencies.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Package {
    /// Package name.
    pub name: String,
    /// Declared dependencies of every kind.
    pub dependencies: Vec<Dependency>,
}

/// A broken layering rule.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Violation {
    /// A workspace crate has no entry in [`RULES`].
    UnknownCrate {
        /// The unlisted crate.
        name: String,
    },
    /// A crate depends on a workspace crate outside its allowed set.
    ForbiddenInternal {
        /// The depending crate.
        from: String,
        /// The forbidden dependency.
        to: String,
    },
    /// An engine crate depends on an external crate that is not approved.
    ForbiddenExternal {
        /// The engine crate.
        from: String,
        /// The unapproved external crate.
        to: String,
    },
}

impl fmt::Display for Violation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownCrate { name } => write!(
                f,
                "`{name}` has no layering rule; add it to RULES in xtask/src/layering.rs"
            ),
            Self::ForbiddenInternal { from, to } => {
                write!(f, "`{from}` may not depend on workspace crate `{to}`")
            }
            Self::ForbiddenExternal { from, to } => write!(
                f,
                "engine crate `{from}` may not depend on external crate `{to}` \
                 (approved: {})",
                ENGINE_EXTERNAL_ALLOWED.join(", ")
            ),
        }
    }
}

/// Engine crates: physics only, no I/O front ends, restricted external dependencies.
pub const ENGINE: &[&str] = &[
    "mjk-core",
    "mjk-math",
    "mjk-dynamics",
    "mjk-env",
    "mjk-aero",
    "mjk-contact",
    "mjk-control",
    "mjk-sensors",
    "mjk-sim",
];

/// External crates engine crates may use outside of tests.
/// See ADR 0001 and ADR 0006.
pub const ENGINE_EXTERNAL_ALLOWED: &[&str] = &["libm", "serde", "serde_path_to_error", "toml"];

/// Test-support crates: any crate may use them as development dependencies, and no crate may
/// use them for anything else. `mjk-alloc-counter` holds the repository's only unsafe code.
pub const TEST_SUPPORT: &[&str] = &["mjk-alloc-counter"];

/// Every workspace crate and the workspace crates it may depend on.
pub const RULES: &[(&str, &[&str])] = &[
    ("mjk-core", &[]),
    ("mjk-math", &["mjk-core"]),
    ("mjk-dynamics", &["mjk-core", "mjk-math"]),
    ("mjk-env", &["mjk-core", "mjk-math"]),
    (
        "mjk-aero",
        &["mjk-core", "mjk-math", "mjk-dynamics", "mjk-env"],
    ),
    ("mjk-contact", &["mjk-core", "mjk-math", "mjk-dynamics"]),
    ("mjk-control", &["mjk-core", "mjk-math", "mjk-dynamics"]),
    (
        "mjk-sensors",
        &[
            "mjk-core",
            "mjk-math",
            "mjk-dynamics",
            "mjk-env",
            "mjk-contact",
        ],
    ),
    (
        "mjk-sim",
        &[
            "mjk-core",
            "mjk-math",
            "mjk-dynamics",
            "mjk-env",
            "mjk-aero",
            "mjk-contact",
            "mjk-control",
            "mjk-sensors",
        ],
    ),
    ("mjk-cli", ENGINE),
    ("mjk-wasm", ENGINE),
    ("mjk-alloc-counter", &[]),
    ("xtask", &[]),
];

/// Returns every layering violation in the given workspace packages.
#[must_use]
pub fn check(packages: &[Package]) -> Vec<Violation> {
    packages.iter().flat_map(check_package).collect()
}

fn allowed_internal(name: &str) -> Option<&'static [&'static str]> {
    RULES
        .iter()
        .find(|(crate_name, _)| *crate_name == name)
        .map(|(_, allowed)| *allowed)
}

fn check_package(package: &Package) -> Vec<Violation> {
    let Some(allowed) = allowed_internal(&package.name) else {
        return vec![Violation::UnknownCrate {
            name: package.name.clone(),
        }];
    };
    package
        .dependencies
        .iter()
        .filter_map(|dep| check_dependency(&package.name, allowed, dep))
        .collect()
}

fn check_dependency(from: &str, allowed: &[&str], dep: &Dependency) -> Option<Violation> {
    if dep.internal {
        let test_support = TEST_SUPPORT.contains(&dep.name.as_str()) && dep.kind == DepKind::Dev;
        let permitted = test_support || allowed.contains(&dep.name.as_str());
        return (!permitted).then(|| Violation::ForbiddenInternal {
            from: from.to_owned(),
            to: dep.name.clone(),
        });
    }
    let restricted = ENGINE.contains(&from) && dep.kind != DepKind::Dev;
    let approved = ENGINE_EXTERNAL_ALLOWED.contains(&dep.name.as_str());
    (restricted && !approved).then(|| Violation::ForbiddenExternal {
        from: from.to_owned(),
        to: dep.name.clone(),
    })
}

#[cfg(test)]
mod tests {
    use super::{DepKind, Dependency, Package, Violation, check};

    fn dep(name: &str, kind: DepKind, internal: bool) -> Dependency {
        Dependency {
            name: name.to_owned(),
            kind,
            internal,
        }
    }

    fn package(name: &str, dependencies: Vec<Dependency>) -> Package {
        Package {
            name: name.to_owned(),
            dependencies,
        }
    }

    #[test]
    fn test_support_crates_are_development_dependencies_only() {
        let as_dev = package(
            "mjk-dynamics",
            vec![dep("mjk-alloc-counter", DepKind::Dev, true)],
        );
        assert!(check(&[as_dev]).is_empty());
        let as_normal = package(
            "mjk-dynamics",
            vec![dep("mjk-alloc-counter", DepKind::Normal, true)],
        );
        assert_eq!(
            check(&[as_normal]),
            vec![Violation::ForbiddenInternal {
                from: "mjk-dynamics".to_owned(),
                to: "mjk-alloc-counter".to_owned(),
            }]
        );
    }

    #[test]
    fn allowed_edges_pass() {
        let packages = [
            package("mjk-math", vec![]),
            package(
                "mjk-aero",
                vec![
                    dep("mjk-math", DepKind::Normal, true),
                    dep("mjk-env", DepKind::Normal, true),
                    dep("libm", DepKind::Normal, false),
                ],
            ),
            package("mjk-cli", vec![dep("mjk-sim", DepKind::Normal, true)]),
        ];
        assert_eq!(check(&packages), vec![]);
    }

    #[test]
    fn engine_crate_depending_on_front_end_is_rejected() {
        let packages = [package(
            "mjk-dynamics",
            vec![dep("mjk-cli", DepKind::Normal, true)],
        )];
        assert_eq!(
            check(&packages),
            vec![Violation::ForbiddenInternal {
                from: "mjk-dynamics".to_owned(),
                to: "mjk-cli".to_owned(),
            }]
        );
    }

    #[test]
    fn upward_dependency_between_engine_layers_is_rejected() {
        let packages = [package(
            "mjk-math",
            vec![dep("mjk-dynamics", DepKind::Normal, true)],
        )];
        assert_eq!(check(&packages).len(), 1);
    }

    #[test]
    fn internal_dev_dependencies_are_still_layered() {
        let packages = [package(
            "mjk-env",
            vec![dep("mjk-wasm", DepKind::Dev, true)],
        )];
        assert_eq!(check(&packages).len(), 1);
    }

    #[test]
    fn unapproved_external_crate_in_engine_is_rejected() {
        let packages = [package(
            "mjk-math",
            vec![dep("nalgebra", DepKind::Normal, false)],
        )];
        assert_eq!(
            check(&packages),
            vec![Violation::ForbiddenExternal {
                from: "mjk-math".to_owned(),
                to: "nalgebra".to_owned(),
            }]
        );
    }

    #[test]
    fn external_dev_dependencies_and_front_ends_are_unrestricted() {
        let packages = [
            package("mjk-math", vec![dep("proptest", DepKind::Dev, false)]),
            package(
                "mjk-wasm",
                vec![dep("wasm-bindgen", DepKind::Normal, false)],
            ),
        ];
        assert_eq!(check(&packages), vec![]);
    }

    #[test]
    fn unlisted_crate_is_rejected() {
        let packages = [package("mjk-render", vec![])];
        assert_eq!(
            check(&packages),
            vec![Violation::UnknownCrate {
                name: "mjk-render".to_owned(),
            }]
        );
    }
}
