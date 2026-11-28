//! Paths to values inside configuration documents.

use std::fmt;

/// One step in a [`FieldPath`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Segment {
    /// A table key, such as `mass`.
    Key(String),
    /// A zero-based array index.
    Index(usize),
}

/// The location of a value inside a configuration document.
///
/// Displayed the way the value is written in a file, for example `rotors[2].position_m`. A key
/// that is not a bare TOML key, made of letters, digits, `_` and `-`, is quoted, as in
/// `propellers."10x4.5".diameter`. The empty path is the whole document and is displayed as
/// `(document)`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FieldPath {
    segments: Vec<Segment>,
}

impl FieldPath {
    /// The path to the whole document.
    #[must_use]
    pub fn root() -> Self {
        Self::default()
    }

    /// Builds a path from segments in document order.
    #[must_use]
    pub fn from_segments(segments: Vec<Segment>) -> Self {
        Self { segments }
    }

    /// This path extended by a table key.
    #[must_use]
    pub fn key(&self, name: &str) -> Self {
        self.with(Segment::Key(name.to_owned()))
    }

    /// This path extended by an array index.
    #[must_use]
    pub fn index(&self, index: usize) -> Self {
        self.with(Segment::Index(index))
    }

    /// The segments in document order.
    #[must_use]
    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    fn with(&self, segment: Segment) -> Self {
        let mut segments = self.segments.clone();
        segments.push(segment);
        Self { segments }
    }
}

impl fmt::Display for FieldPath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.segments.is_empty() {
            return f.write_str("(document)");
        }
        for (position, segment) in self.segments.iter().enumerate() {
            match segment {
                Segment::Key(name) if position == 0 => f.write_str(&quoted(name))?,
                Segment::Key(name) => write!(f, ".{}", quoted(name))?,
                Segment::Index(index) => write!(f, "[{index}]")?,
            }
        }
        Ok(())
    }
}

/// A key as TOML writes it: bare if it can be, otherwise in quotes.
fn quoted(name: &str) -> String {
    let bare = !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
    if bare {
        return name.to_owned();
    }
    format!("\"{}\"", name.replace('\\', "\\\\").replace('"', "\\\""))
}

#[cfg(test)]
mod tests {
    use super::{FieldPath, Segment};

    #[test]
    fn displays_like_the_document() {
        let path = FieldPath::root().key("rotors").index(2).key("position_m");
        assert_eq!(path.to_string(), "rotors[2].position_m");
        let nested = FieldPath::root()
            .key("mass")
            .key("inertia_kg_m2")
            .index(1)
            .index(0);
        assert_eq!(nested.to_string(), "mass.inertia_kg_m2[1][0]");
    }

    #[test]
    fn built_from_segments_like_extended_paths() {
        let segments = vec![
            Segment::Key("rotors".to_owned()),
            Segment::Index(2),
            Segment::Key("position_m".to_owned()),
        ];
        let built = FieldPath::from_segments(segments);
        assert_eq!(built.to_string(), "rotors[2].position_m");
        assert_eq!(
            built,
            FieldPath::root().key("rotors").index(2).key("position_m")
        );
    }

    #[test]
    fn keys_that_are_not_bare_are_quoted() {
        let path = FieldPath::root()
            .key("propellers")
            .key("10x4.5")
            .key("diameter");
        assert_eq!(path.to_string(), "propellers.\"10x4.5\".diameter");
        assert_eq!(FieldPath::root().key("a b").to_string(), "\"a b\"");
        assert_eq!(FieldPath::root().key("").to_string(), "\"\"");
        assert_eq!(
            FieldPath::root().key("say \"hi\" \\").to_string(),
            "\"say \\\"hi\\\" \\\\\""
        );
        assert_eq!(
            FieldPath::root().key("2212-920_b").to_string(),
            "2212-920_b"
        );
    }

    #[test]
    fn root_is_the_document() {
        assert_eq!(FieldPath::root().to_string(), "(document)");
    }

    #[test]
    fn extending_does_not_change_the_original() {
        let base = FieldPath::root().key("mass");
        let _child = base.key("mass_kg");
        assert_eq!(base.segments(), &[Segment::Key("mass".to_owned())]);
    }
}
