//! Airframe documents ([#7](https://github.com/MatthewK84/MJKPhysics/issues/7),
//! [#18](https://github.com/MatthewK84/MJKPhysics/issues/18)).
//!
//! An airframe file gives mass properties, rotors, propellers, motors, ESCs, the battery, body
//! drag, controller gains and visual hints. Every number is a [`Parameter`] with a unit and,
//! ideally, a source from the file's `[sources]`. Loading converts every value to SI, checks
//! each against its quantity and the whole airframe for plausibility, and records every
//! parameter for the airframe's generated page.
//!
//! [`Parameter`]: super::parameter::Parameter

use std::collections::{BTreeMap, BTreeSet};

use mjk_core::{EngineError, FieldPath};
use mjk_dynamics::mass::MassProperties;
use serde::{Deserialize, Serialize};

use super::body::{BodyDrag, Controller, ControllerConfig, DragConfig, Visual, VisualConfig};
use super::document::{DocumentKind, parse_document};
use super::drive::check_drive_trains;
use super::mass::MassConfig;
use super::parameter::{ConfigWarning, ParameterRecord, Resolver};
use super::plausibility::check_rotors;
use super::propulsion::{
    Battery, BatteryConfig, Esc, EscConfig, Motor, MotorConfig, resolve_table,
};
use super::rotors::{Catalog, Propeller, PropellerConfig, Rotor, RotorConfig, resolve_propellers};
use super::sources::{Source, check_sources};

/// An airframe document as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AirframeConfig {
    /// Format version of the document.
    pub schema_version: i64,
    /// Always `"airframe"`.
    pub kind: String,
    /// Human-readable name.
    pub name: String,
    /// Where the numbers come from, by key.
    #[serde(default)]
    pub sources: BTreeMap<String, Source>,
    /// Mass properties.
    pub mass: MassConfig,
    /// The rotors.
    pub rotors: Vec<RotorConfig>,
    /// Propellers, by key.
    #[serde(default)]
    pub propellers: BTreeMap<String, PropellerConfig>,
    /// Motors, by key.
    #[serde(default)]
    pub motors: BTreeMap<String, MotorConfig>,
    /// ESCs, by key.
    #[serde(default)]
    pub escs: BTreeMap<String, EscConfig>,
    /// The battery.
    pub battery: Option<BatteryConfig>,
    /// Body drag.
    pub drag: Option<DragConfig>,
    /// The flight controller.
    pub controller: Option<ControllerConfig>,
    /// Hints for drawing it.
    pub visual: Option<VisualConfig>,
}

/// An airframe in SI units.
#[derive(Clone, Debug, PartialEq)]
pub struct Airframe {
    /// Human-readable name.
    pub name: String,
    /// Mass properties.
    pub mass: MassProperties,
    /// The rotors.
    pub rotors: Vec<Rotor>,
    /// Propellers, by key.
    pub propellers: BTreeMap<String, Propeller>,
    /// Motors, by key.
    pub motors: BTreeMap<String, Motor>,
    /// ESCs, by key.
    pub escs: BTreeMap<String, Esc>,
    /// The battery.
    pub battery: Option<Battery>,
    /// Body drag.
    pub drag: Option<BodyDrag>,
    /// The flight controller.
    pub controller: Option<Controller>,
    /// Hints for drawing it.
    pub visual: Option<Visual>,
}

/// A loaded airframe, with what its page needs and what is worth fixing.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadedAirframe {
    /// The file as written.
    pub config: AirframeConfig,
    /// The airframe in SI units.
    pub airframe: Airframe,
    /// Every parameter, in the order read.
    pub parameters: Vec<ParameterRecord>,
    /// Warnings, such as numbers without a source.
    pub warnings: Vec<ConfigWarning>,
}

/// Parses, converts and checks an airframe document.
///
/// `document` names the source in error messages, usually its file path; `text` is its
/// contents.
///
/// # Errors
///
/// Returns [`EngineError::Config`] naming the exact field for invalid TOML, an unsupported or
/// missing header, a missing, unknown or mistyped field, a number without a unit, a unit of
/// the wrong quantity, a source that is not in `[sources]`, a physically implausible value, too
/// little thrust, or rotor disks that overlap.
pub fn load_airframe(document: &str, text: &str) -> Result<LoadedAirframe, EngineError> {
    let config: AirframeConfig = parse_document(document, text, DocumentKind::Airframe)?;
    config.resolve(document)
}

impl AirframeConfig {
    /// The airframe in SI units, checked.
    ///
    /// # Errors
    ///
    /// As [`load_airframe`], after parsing.
    pub fn resolve(&self, document: &str) -> Result<LoadedAirframe, EngineError> {
        if self.name.trim().is_empty() {
            let path = FieldPath::root().key("name");
            return Err(EngineError::config(document, path, "must not be empty"));
        }
        check_sources(document, &self.sources)?;
        let mut resolver = Resolver::new(document, &self.sources);
        let airframe = self.airframe(&mut resolver)?;
        check_rotors(&resolver, &airframe)?;
        check_drive_trains(&resolver, &airframe)?;
        self.warn_unused(&mut resolver);
        let (parameters, warnings) = resolver.finish();
        Ok(LoadedAirframe {
            config: self.clone(),
            airframe,
            parameters,
            warnings,
        })
    }

    fn airframe(&self, resolver: &mut Resolver<'_>) -> Result<Airframe, EngineError> {
        let root = FieldPath::root();
        let mass = self.mass.resolve(resolver, &root.key("mass"))?;
        let rotors = self.rotors(resolver)?;
        let propellers = resolve_propellers(&self.propellers, resolver)?;
        let motors = resolve_table(&self.motors, "motors", resolver, MotorConfig::resolve)?;
        let escs = resolve_table(&self.escs, "escs", resolver, EscConfig::resolve)?;
        Ok(Airframe {
            name: self.name.clone(),
            mass,
            rotors,
            propellers,
            motors,
            escs,
            battery: optional(
                self.battery.as_ref(),
                resolver,
                "battery",
                BatteryConfig::resolve,
            )?,
            drag: optional(self.drag.as_ref(), resolver, "drag", DragConfig::resolve)?,
            controller: optional(
                self.controller.as_ref(),
                resolver,
                "controller",
                ControllerConfig::resolve,
            )?,
            visual: optional(
                self.visual.as_ref(),
                resolver,
                "visual",
                VisualConfig::resolve,
            )?,
        })
    }

    fn rotors(&self, resolver: &mut Resolver<'_>) -> Result<Vec<Rotor>, EngineError> {
        let catalog = Catalog {
            propellers: self.propellers.keys().map(String::as_str).collect(),
            motors: self.motors.keys().map(String::as_str).collect(),
            escs: self.escs.keys().map(String::as_str).collect(),
        };
        let path = FieldPath::root().key("rotors");
        self.rotors
            .iter()
            .enumerate()
            .map(|(index, rotor)| rotor.resolve(resolver, &path.index(index), &catalog))
            .collect()
    }

    /// Warns about each propeller, motor and ESC that no rotor uses.
    fn warn_unused(&self, resolver: &mut Resolver<'_>) {
        let propellers: BTreeSet<&str> = self.rotors.iter().map(|r| r.propeller.as_str()).collect();
        let motors: BTreeSet<&str> = self
            .rotors
            .iter()
            .filter_map(|r| r.motor.as_deref())
            .collect();
        let escs: BTreeSet<&str> = self
            .rotors
            .iter()
            .filter_map(|r| r.esc.as_deref())
            .collect();
        let tables = [
            ("propellers", keys(&self.propellers), propellers),
            ("motors", keys(&self.motors), motors),
            ("escs", keys(&self.escs), escs),
        ];
        for (table, defined, used) in tables {
            for key in defined.into_iter().filter(|key| !used.contains(key)) {
                let path = FieldPath::root().key(table).key(key);
                resolver.warn(path, "no rotor uses it");
            }
        }
    }
}

/// The keys of a table.
fn keys<T>(table: &BTreeMap<String, T>) -> Vec<&str> {
    table.keys().map(String::as_str).collect()
}

/// Resolves an optional table at `name`.
fn optional<C, T>(
    config: Option<&C>,
    resolver: &mut Resolver<'_>,
    name: &str,
    resolve: impl Fn(&C, &mut Resolver<'_>, &FieldPath) -> Result<T, EngineError>,
) -> Result<Option<T>, EngineError> {
    let path = FieldPath::root().key(name);
    config
        .map(|config| resolve(config, resolver, &path))
        .transpose()
}
