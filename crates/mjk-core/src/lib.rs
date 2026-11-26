//! Foundation types shared by every engine crate.
//!
//! The lowest layer of the engine: it depends on nothing, and any engine crate may depend on
//! it. It holds the engine error model, [`EngineError`], and the [`FieldPath`] used to point
//! at the exact value in a configuration document that caused an error. See ADR 0006.

mod error;
mod field_path;

pub use error::EngineError;
pub use field_path::{FieldPath, Segment};
