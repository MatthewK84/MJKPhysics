//! The allocation counter's lint table must match the workspace's.
//!
//! Cargo cannot override one workspace lint, so `crates/mjk-alloc-counter/Cargo.toml` repeats
//! the whole table to change a single rule. This check fails if the copy drifts: the only
//! differences allowed are `unsafe_code = "deny"` instead of `"forbid"` and one added Clippy lint
//! that requires a `SAFETY:` comment on every unsafe block.

use std::fs;
use std::path::Path;

use toml::{Table, Value};

/// The crate whose lint table repeats the workspace's.
const COPY: &str = "crates/mjk-alloc-counter/Cargo.toml";

/// Every difference between the copy and the workspace table, described for a person.
///
/// # Errors
///
/// Returns a description when either manifest cannot be read or parsed.
pub fn check(root: &Path) -> Result<Vec<String>, String> {
    let workspace = read_table(&root.join("Cargo.toml"))?;
    let copy = read_table(&root.join(COPY))?;
    let mut expected_rust = section(&workspace, &["workspace", "lints", "rust"]);
    expected_rust.insert("unsafe_code".to_owned(), Value::from("deny"));
    let mut expected_clippy = section(&workspace, &["workspace", "lints", "clippy"]);
    expected_clippy.insert("undocumented_unsafe_blocks".to_owned(), Value::from("deny"));
    let mut problems = differences(
        "lints.rust",
        &expected_rust,
        &section(&copy, &["lints", "rust"]),
    );
    problems.extend(differences(
        "lints.clippy",
        &expected_clippy,
        &section(&copy, &["lints", "clippy"]),
    ));
    Ok(problems)
}

fn read_table(path: &Path) -> Result<Table, String> {
    let text = fs::read_to_string(path).map_err(|error| format!("{}: {error}", path.display()))?;
    text.parse::<Table>()
        .map_err(|error| format!("{}: {error}", path.display()))
}

/// The table at `keys`, or an empty table if any key is missing.
fn section(table: &Table, keys: &[&str]) -> Table {
    let mut current = table;
    for key in keys {
        match current.get(*key).and_then(Value::as_table) {
            Some(next) => current = next,
            None => return Table::new(),
        }
    }
    current.clone()
}

/// Entries of `expected` missing or different in `actual`, and entries only in `actual`.
fn differences(name: &str, expected: &Table, actual: &Table) -> Vec<String> {
    let mut found: Vec<String> = expected
        .iter()
        .filter(|(key, value)| actual.get(*key) != Some(*value))
        .map(|(key, value)| {
            format!("{COPY}: [{name}] `{key}` should be {value}, as in the workspace")
        })
        .collect();
    found.extend(
        actual
            .keys()
            .filter(|key| !expected.contains_key(*key))
            .map(|key| format!("{COPY}: [{name}] `{key}` is not in the workspace table")),
    );
    found
}

#[cfg(test)]
mod tests {
    use toml::Table;

    use super::differences;

    fn table(text: &str) -> Table {
        text.parse().expect("valid TOML")
    }

    #[test]
    fn matching_tables_have_no_differences() {
        let lints = table("a = \"deny\"\nb = { level = \"warn\", priority = -1 }");
        assert!(differences("lints.rust", &lints, &lints).is_empty());
    }

    #[test]
    fn changed_missing_and_extra_entries_are_reported() {
        let expected = table("a = \"deny\"\nb = \"warn\"");
        let actual = table("a = \"allow\"\nc = \"deny\"");
        let found = differences("lints.clippy", &expected, &actual);
        assert_eq!(found.len(), 3, "{found:?}");
        assert!(found.iter().any(|f| f.contains("`a` should be \"deny\"")));
        assert!(found.iter().any(|f| f.contains("`b` should be \"warn\"")));
        assert!(
            found
                .iter()
                .any(|f| f.contains("`c` is not in the workspace table"))
        );
    }
}
