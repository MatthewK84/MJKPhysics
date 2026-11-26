//! Airframe documents.
//!
//! This is the first slice of the airframe format: identity and mass properties. Rotors,
//! propulsion, aerodynamics and parameter sources arrive with
//! [#18](https://github.com/MatthewK84/MJKPhysics/issues/18) and
//! [#43](https://github.com/MatthewK84/MJKPhysics/issues/43).

use mjk_core::{EngineError, FieldPath};
use serde::{Deserialize, Serialize};

use super::document::{DocumentKind, parse_document};
use super::mass::MassConfig;

/// A parsed and validated airframe document.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AirframeConfig {
    /// Format version of the document.
    pub schema_version: i64,
    /// Always `"airframe"`.
    pub kind: String,
    /// Human-readable name.
    pub name: String,
    /// Mass properties.
    pub mass: MassConfig,
}

impl AirframeConfig {
    /// Checks every field for physical plausibility.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::Config`] naming the first implausible field.
    pub fn validate(&self, document: &str) -> Result<(), EngineError> {
        if self.name.trim().is_empty() {
            let reason = "must not be empty".to_owned();
            return Err(EngineError::config(
                document,
                FieldPath::root().key("name"),
                reason,
            ));
        }
        self.mass.validate(document, &FieldPath::root().key("mass"))
    }
}

/// Parses and validates an airframe document.
///
/// `document` names the source in error messages, usually its file path; `text` is its
/// contents.
///
/// # Errors
///
/// Returns [`EngineError::Config`] for invalid TOML, an unsupported or missing header, a
/// missing, unknown or mistyped field, or physically implausible values. The error names the
/// exact field.
pub fn load_airframe(document: &str, text: &str) -> Result<AirframeConfig, EngineError> {
    let airframe: AirframeConfig = parse_document(document, text, DocumentKind::Airframe)?;
    airframe.validate(document)?;
    Ok(airframe)
}
