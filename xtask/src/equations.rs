//! Equation IDs in the documentation and their citations in code.
//!
//! A display equation in `docs/src/**/*.md` is a fenced block with the language `math`; its ID is
//! the trailing `\text{(ID)}`, for example `\text{(CONV-3)}`. Code cites equations in comments as
//! `Eq. (ID)`. This module extracts both and reports duplicates, malformed IDs and citations of
//! equations that do not exist. Fences nested inside longer fences (examples in templates) are
//! ignored.

use std::collections::BTreeMap;
use std::fmt;

const TAG_OPEN: &str = "\\text{(";
const TAG_CLOSE: &str = ")}";
const CITE_OPEN: &str = "Eq. (";

/// Where something was found.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Location {
    /// Path relative to the repository root.
    pub file: String,
    /// One-based line number.
    pub line: usize,
}

impl fmt::Display for Location {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.file, self.line)
    }
}

/// A broken equation-ID rule.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Problem {
    /// The same ID tags two equations.
    Duplicate {
        /// The repeated ID.
        id: String,
        /// The first definition.
        first: Location,
        /// The repeated definition.
        second: Location,
    },
    /// A tag or citation is not of the form `PREFIX-NUMBER`.
    Malformed {
        /// The text found.
        text: String,
        /// Where it was found.
        at: Location,
    },
    /// Code cites an ID that no equation defines.
    UnknownCitation {
        /// The cited ID.
        id: String,
        /// Where it was cited.
        at: Location,
    },
}

impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Duplicate { id, first, second } => {
                write!(f, "{second}: equation ID ({id}) already defined at {first}")
            }
            Self::Malformed { text, at } => {
                write!(f, "{at}: `{text}` is not an equation ID like CONV-3")
            }
            Self::UnknownCitation { id, at } => {
                write!(
                    f,
                    "{at}: cites Eq. ({id}), which no documentation page defines"
                )
            }
        }
    }
}

/// True when `id` is an uppercase prefix, a hyphen and a decimal number, like `BEMT-4`.
#[must_use]
pub fn is_valid_id(id: &str) -> bool {
    let Some((prefix, number)) = id.split_once('-') else {
        return false;
    };
    let prefix_ok = prefix
        .chars()
        .next()
        .is_some_and(|c| c.is_ascii_uppercase())
        && prefix
            .chars()
            .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit());
    let number_ok = !number.is_empty() && number.chars().all(|c| c.is_ascii_digit());
    prefix_ok && number_ok
}

/// Returns the text between each `open` and the following `close` on one line.
fn enclosed<'a>(line: &'a str, open: &str, close: &str) -> Vec<&'a str> {
    let mut found = Vec::new();
    let mut rest = line;
    // bound: each pass consumes at least `open` and `close` from `rest`, or stops; at most
    // `line.len()` passes.
    while let Some(start) = rest.find(open) {
        let after = rest.get(start + open.len()..).unwrap_or_default();
        let Some(end) = after.find(close) else {
            break;
        };
        found.push(after.get(..end).unwrap_or_default());
        rest = after.get(end + close.len()..).unwrap_or_default();
    }
    found
}

/// Number of leading backticks once indentation is removed, and the text after them.
fn fence(line: &str) -> Option<(usize, &str)> {
    let trimmed = line.trim_start();
    let ticks = trimmed.chars().take_while(|&c| c == '`').count();
    (ticks >= 3).then(|| (ticks, trimmed.get(ticks..).unwrap_or_default().trim()))
}

/// Extracts `(id, line)` for every tag inside top-level ```` ```math ```` fences.
#[must_use]
pub fn tags_in_markdown(text: &str) -> Vec<(String, usize)> {
    let mut tags = Vec::new();
    let mut open: Option<(usize, bool)> = None;
    for (index, line) in text.lines().enumerate() {
        match (open, fence(line)) {
            (None, Some((ticks, info))) => open = Some((ticks, info == "math")),
            (Some((ticks, _)), Some((closing, ""))) if closing >= ticks => open = None,
            (Some((_, true)), _) => tags.extend(
                enclosed(line, TAG_OPEN, TAG_CLOSE)
                    .into_iter()
                    .map(|id| (id.to_owned(), index + 1)),
            ),
            _ => {}
        }
    }
    tags
}

/// Extracts `(id, line)` for every `Eq. (ID)` citation in source text.
#[must_use]
pub fn citations_in_source(text: &str) -> Vec<(String, usize)> {
    text.lines()
        .enumerate()
        .flat_map(|(index, line)| {
            enclosed(line, CITE_OPEN, ")")
                .into_iter()
                .map(move |id| (id.to_owned(), index + 1))
        })
        .collect()
}

/// Checks tags from documentation files and citations from source files.
///
/// Each input is `(path, contents)`. Returns every problem found, in a stable order.
#[must_use]
pub fn check(docs: &[(String, String)], sources: &[(String, String)]) -> Vec<Problem> {
    let mut problems = Vec::new();
    let mut defined: BTreeMap<String, Location> = BTreeMap::new();
    for (file, text) in docs {
        for (id, line) in tags_in_markdown(text) {
            let at = Location {
                file: file.clone(),
                line,
            };
            if !is_valid_id(&id) {
                problems.push(Problem::Malformed { text: id, at });
                continue;
            }
            if let Some(first) = defined.get(&id) {
                problems.push(Problem::Duplicate {
                    id,
                    first: first.clone(),
                    second: at,
                });
                continue;
            }
            defined.insert(id, at);
        }
    }
    for (file, text) in sources {
        for (id, line) in citations_in_source(text) {
            let at = Location {
                file: file.clone(),
                line,
            };
            if !is_valid_id(&id) {
                problems.push(Problem::Malformed { text: id, at });
            } else if !defined.contains_key(&id) {
                problems.push(Problem::UnknownCitation { id, at });
            }
        }
    }
    problems
}

#[cfg(test)]
mod tests {
    use super::{Problem, check, citations_in_source, is_valid_id, tags_in_markdown};

    fn doc(text: &str) -> Vec<(String, String)> {
        vec![("docs/src/page.md".to_owned(), text.to_owned())]
    }

    #[test]
    fn valid_ids() {
        assert!(is_valid_id("CONV-3"));
        assert!(is_valid_id("BEMT-12"));
        assert!(is_valid_id("A2B-1"));
        assert!(!is_valid_id("conv-3"));
        assert!(!is_valid_id("CONV3"));
        assert!(!is_valid_id("CONV-"));
        assert!(!is_valid_id("2A-1"));
        assert!(!is_valid_id("CONV-3a"));
    }

    #[test]
    fn tags_come_only_from_top_level_math_fences() {
        let text = "\
Inline \\text{(NOT-1)} prose.
```math
x = 1 \\qquad\\text{(A-1)}
```
```rust
// \\text{(NOT-2)}
```
````markdown
```math
y = 2 \\qquad\\text{(NOT-3)}
```
````
  ```math
  z = 3 \\qquad\\text{(A-2)}
  ```
";
        let ids: Vec<String> = tags_in_markdown(text)
            .into_iter()
            .map(|(id, _)| id)
            .collect();
        assert_eq!(ids, vec!["A-1".to_owned(), "A-2".to_owned()]);
    }

    #[test]
    fn citations_are_found_with_line_numbers() {
        let source = "fn f() {}\n// Eq. (CONV-9) and Eq. (RB-2)\n";
        assert_eq!(
            citations_in_source(source),
            vec![("CONV-9".to_owned(), 2), ("RB-2".to_owned(), 2)]
        );
    }

    #[test]
    fn duplicate_ids_are_reported() {
        let text = "```math\na \\text{(A-1)}\n```\n```math\nb \\text{(A-1)}\n```\n";
        let problems = check(&doc(text), &[]);
        assert!(matches!(problems.as_slice(), [Problem::Duplicate { id, .. }] if id == "A-1"));
    }

    #[test]
    fn unknown_and_malformed_citations_are_reported() {
        let text = "```math\na \\text{(A-1)}\n```\n";
        let sources = vec![(
            "crates/x/src/lib.rs".to_owned(),
            "// Eq. (A-1)\n// Eq. (A-2)\n// Eq. (a1)\n".to_owned(),
        )];
        let problems = check(&doc(text), &sources);
        assert_eq!(problems.len(), 2);
        assert!(matches!(&problems[0], Problem::UnknownCitation { id, .. } if id == "A-2"));
        assert!(matches!(&problems[1], Problem::Malformed { text: found, .. } if found == "a1"));
    }

    #[test]
    fn malformed_tags_are_reported() {
        let problems = check(&doc("```math\na \\text{(see below)}\n```\n"), &[]);
        assert!(matches!(problems.as_slice(), [Problem::Malformed { .. }]));
    }
}
