//! Configuration documents: TOML text in, validated data out.
//!
//! Engine crates do no I/O, so loaders take the document's text and a name to use in error
//! messages, usually its file path. Front ends read the file. Every document starts with a
//! header:
//!
//! ```toml
//! schema_version = 1
//! kind = "airframe"
//! ```
//!
//! Unknown fields are rejected, and every error is an [`EngineError::Config`] naming the exact
//! field, for example `mass.inertia_kg_m2[1][0]`. The format is described in the book's
//! *Configuration files* page.
//!
//! [`EngineError::Config`]: mjk_core::EngineError::Config

mod airframe;
mod document;
mod mass;

pub use airframe::{AirframeConfig, load_airframe};
pub use document::{DocumentKind, NEWEST_SCHEMA_VERSION, OLDEST_SCHEMA_VERSION};
pub use mass::MassConfig;
