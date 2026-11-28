//! The bibliography of an airframe file ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)):
//! `[sources.<key>]` tables that parameters cite by key.

use std::collections::BTreeMap;

use mjk_core::{EngineError, FieldPath};
use serde::{Deserialize, Serialize};

/// A publication, dataset or measurement that parameters come from.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Source {
    /// What it is called.
    pub title: String,
    /// Who wrote or published it.
    pub author: Option<String>,
    /// When it was published.
    pub year: Option<u16>,
    /// Where to find it.
    pub url: Option<String>,
    /// Its digital object identifier.
    pub doi: Option<String>,
    /// Anything a reader should know, such as which table or page the values are on.
    pub note: Option<String>,
}

impl Source {
    /// The source as one line of the airframe's page.
    #[must_use]
    pub fn citation(&self) -> String {
        let mut parts = vec![self.title.clone()];
        parts.extend(self.author.clone());
        parts.extend(self.year.map(|year| year.to_string()));
        parts.extend(self.doi.as_ref().map(|doi| format!("doi:{doi}")));
        parts.extend(self.url.as_ref().map(|url| format!("<{url}>")));
        let line = parts.join(", ");
        self.note
            .as_ref()
            .map_or_else(|| format!("{line}."), |note| format!("{line}. {note}"))
    }
}

/// Every source needs a title.
///
/// # Errors
///
/// [`EngineError::Config`] naming the first source with an empty title.
pub fn check_sources(
    document: &str,
    sources: &BTreeMap<String, Source>,
) -> Result<(), EngineError> {
    let untitled = sources
        .iter()
        .find(|(_, source)| source.title.trim().is_empty());
    untitled.map_or(Ok(()), |(key, _)| {
        let path = FieldPath::root().key("sources").key(key).key("title");
        Err(EngineError::config(document, path, "must not be empty"))
    })
}
