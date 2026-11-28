//! Parsing shared by every document kind: syntax, header, field paths.

use mjk_core::{EngineError, FieldPath, Segment};
use serde::de::DeserializeOwned;

/// Oldest `schema_version` this build reads.
pub const OLDEST_SCHEMA_VERSION: i64 = 1;
/// Newest `schema_version` this build reads, and the one it writes.
pub const NEWEST_SCHEMA_VERSION: i64 = 1;

/// The kinds of configuration document, named by the header's `kind` field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DocumentKind {
    /// An aircraft definition.
    Airframe,
}

impl DocumentKind {
    /// The value of the `kind` field for this document.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Airframe => "airframe",
        }
    }
}

/// Parses `text` as a document of `kind` into `T`.
///
/// Checks, in order: TOML syntax, the header (`schema_version`, then `kind`), then the shape
/// of the whole document, rejecting unknown fields. Physical plausibility is checked by each
/// document type afterward.
pub(crate) fn parse_document<T: DeserializeOwned>(
    document: &str,
    text: &str,
    kind: DocumentKind,
) -> Result<T, EngineError> {
    let table: toml::Table =
        toml::from_str(text).map_err(|error| syntax_error(document, text, &error))?;
    check_header(document, &table, kind)?;
    let deserializer =
        toml::Deserializer::parse(text).map_err(|error| syntax_error(document, text, &error))?;
    serde_path_to_error::deserialize(deserializer)
        .map_err(|error| shape_error(document, text, &error))
}

fn check_header(
    document: &str,
    table: &toml::Table,
    kind: DocumentKind,
) -> Result<(), EngineError> {
    let version_path = FieldPath::root().key("schema_version");
    let version = match table.get("schema_version") {
        Some(toml::Value::Integer(version)) => *version,
        Some(other) => {
            let found = other.type_str();
            return Err(EngineError::config(
                document,
                version_path,
                format!("must be an integer; found a {found}"),
            ));
        }
        None => {
            let reason =
                format!("missing; documents start with `schema_version = {NEWEST_SCHEMA_VERSION}`");
            return Err(EngineError::config(document, version_path, reason));
        }
    };
    check_version(document, version_path, version)?;
    let kind_path = FieldPath::root().key("kind");
    let expected = kind.name();
    match table.get("kind") {
        Some(toml::Value::String(found)) if found == expected => Ok(()),
        Some(toml::Value::String(found)) => Err(EngineError::config(
            document,
            kind_path,
            format!("expected \"{expected}\" for this loader; found \"{found}\""),
        )),
        Some(other) => {
            let found = other.type_str();
            Err(EngineError::config(
                document,
                kind_path,
                format!("must be a string; found a {found}"),
            ))
        }
        None => Err(EngineError::config(
            document,
            kind_path,
            format!("missing; add `kind = \"{expected}\"`"),
        )),
    }
}

fn check_version(document: &str, path: FieldPath, version: i64) -> Result<(), EngineError> {
    if version > NEWEST_SCHEMA_VERSION {
        let reason = format!(
            "this file uses schema version {version}, but this build reads {}; update MJKPhysics",
            supported_versions()
        );
        return Err(EngineError::config(document, path, reason));
    }
    if version < OLDEST_SCHEMA_VERSION {
        let reason = format!(
            "schema version {version} is not supported (oldest supported: \
             {OLDEST_SCHEMA_VERSION}); see the migration notes in the Configuration files page"
        );
        return Err(EngineError::config(document, path, reason));
    }
    Ok(())
}

fn supported_versions() -> String {
    if OLDEST_SCHEMA_VERSION == NEWEST_SCHEMA_VERSION {
        return format!("version {NEWEST_SCHEMA_VERSION}");
    }
    format!("versions {OLDEST_SCHEMA_VERSION} to {NEWEST_SCHEMA_VERSION}")
}

/// One-based line and column of a byte offset.
fn line_column(text: &str, offset: usize) -> (usize, usize) {
    let before = text.get(..offset).unwrap_or(text);
    let line = before.matches('\n').count() + 1;
    let column = before
        .rsplit('\n')
        .next()
        .map_or(0, |tail| tail.chars().count())
        + 1;
    (line, column)
}

fn located(text: &str, error: &toml::de::Error) -> String {
    let message = error.message().trim();
    error.span().map_or_else(
        || message.to_owned(),
        |span| {
            let (line, column) = line_column(text, span.start);
            format!("{message} (line {line}, column {column})")
        },
    )
}

fn syntax_error(document: &str, text: &str, error: &toml::de::Error) -> EngineError {
    EngineError::config(
        document,
        FieldPath::root(),
        format!("invalid TOML: {}", located(text, error)),
    )
}

fn shape_error(
    document: &str,
    text: &str,
    error: &serde_path_to_error::Error<toml::de::Error>,
) -> EngineError {
    let inner = error.inner();
    EngineError::config(
        document,
        field_path(error.path(), inner.message()),
        located(text, inner),
    )
}

/// The field a deserialization error names: serde's path to it and, for serde's "unknown field"
/// error, the unknown key itself. `message` is the error's message.
///
/// Front ends use it for documents in other formats, such as the JSON of a recorded run.
#[must_use]
pub fn field_path(path: &serde_path_to_error::Path, message: &str) -> FieldPath {
    let mut segments: Vec<Segment> = path
        .iter()
        .filter_map(|segment| match segment {
            serde_path_to_error::Segment::Map { key } => Some(Segment::Key(key.clone())),
            serde_path_to_error::Segment::Seq { index } => Some(Segment::Index(*index)),
            serde_path_to_error::Segment::Enum { variant } => Some(Segment::Key(variant.clone())),
            serde_path_to_error::Segment::Unknown => None,
        })
        .collect();
    if let Some(unknown) = unknown_field(message) {
        // Point at the unknown key itself, unless the path already ends with it.
        let key = Segment::Key(unknown);
        if segments.last() != Some(&key) {
            segments.push(key);
        }
    }
    FieldPath::from_segments(segments)
}

/// The key named in serde's "unknown field `x`" message, if this is that error.
fn unknown_field(message: &str) -> Option<String> {
    let rest = message.strip_prefix("unknown field `")?;
    rest.split_once('`').map(|(name, _)| name.to_owned())
}

#[cfg(test)]
mod tests {
    use super::{line_column, unknown_field};

    #[test]
    fn line_and_column_are_one_based() {
        let text = "a = 1\nbb = 2\n";
        assert_eq!(line_column(text, 0), (1, 1));
        assert_eq!(line_column(text, 6), (2, 1));
        assert_eq!(line_column(text, 9), (2, 4));
    }

    #[test]
    fn unknown_field_name_is_extracted() {
        let message = "unknown field `mas_kg`, expected one of `mass_kg`, `cg_m`";
        assert_eq!(unknown_field(message).as_deref(), Some("mas_kg"));
        assert_eq!(unknown_field("invalid type: string"), None);
    }
}
