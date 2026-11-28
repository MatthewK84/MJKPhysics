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
//! field, for example `mass.inertia.value[1][0]`. In an airframe, every number is a parameter
//! table with a unit and a source ([`Parameter`]); loading converts it to SI. The format is
//! described in the book's *Configuration files* page.
//!
//! [`EngineError::Config`]: mjk_core::EngineError::Config

mod airframe;
mod body;
mod document;
mod mass;
mod page;
mod parameter;
mod plausibility;
mod propulsion;
mod rotors;
mod sources;
pub mod units;

pub use airframe::{Airframe, AirframeConfig, LoadedAirframe, load_airframe};
pub use body::{BodyDrag, Controller, ControllerConfig, DragConfig, Visual, VisualConfig};
pub use document::{DocumentKind, NEWEST_SCHEMA_VERSION, OLDEST_SCHEMA_VERSION, field_path};
pub use mass::{ComponentConfig, MassConfig, ShapeKind};
pub use page::airframe_page;
pub use parameter::{
    ConfigWarning, Format, MISSING_UNIT, Parameter, ParameterRecord, Resolver, Value, plain,
    significant,
};
pub use plausibility::{COPLANAR_M, check_rotors, thrust_to_weight};
pub use propulsion::{Battery, BatteryConfig, Braking, Esc, EscConfig, Motor, MotorConfig};
pub use rotors::{Propeller, PropellerConfig, Rotor, RotorConfig, Spin};
pub use sources::Source;
