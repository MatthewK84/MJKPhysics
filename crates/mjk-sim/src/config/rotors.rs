//! Rotors and propellers ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)).
//!
//! Each `[[rotors]]` entry places a rotor: where its hub is, which way its thrust points, which
//! way it spins, and which propeller, motor and ESC it uses. Propellers, motors and ESCs are
//! defined once, in `[propellers.<key>]`, `[motors.<key>]` and `[escs.<key>]`, and rotors name
//! them by key.

use std::collections::BTreeMap;

use mjk_core::{EngineError, FieldPath};
use mjk_math::{Body, Vec3};
use serde::{Deserialize, Serialize};

use super::parameter::{Parameter, Resolver};
use super::units::quantities::{FORCE_N, INERTIA_KG_M2, LENGTH_M, MASS_KG, PURE};

/// Which way a rotor turns, seen from the side its thrust points to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Spin {
    /// Clockwise: the rotor's angular velocity points against its thrust axis.
    Cw,
    /// Counterclockwise: the rotor's angular velocity points along its thrust axis.
    Ccw,
}

impl Spin {
    /// As written in a file.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Cw => "cw",
            Self::Ccw => "ccw",
        }
    }
}

/// A rotor as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RotorConfig {
    /// What it is called, such as `"front right"`.
    pub name: String,
    /// The hub, from the airframe's reference point, body axes.
    pub position: Parameter<[f64; 3]>,
    /// The direction of thrust, body axes; normalized when read. Up is `[0, 0, -1]`.
    pub axis: Parameter<[f64; 3]>,
    /// Which way it turns.
    pub spin: Spin,
    /// The key of its propeller in `[propellers]`.
    pub propeller: String,
    /// The key of its motor in `[motors]`.
    pub motor: Option<String>,
    /// The key of its ESC in `[escs]`.
    pub esc: Option<String>,
}

/// A propeller as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerConfig {
    /// Tip-to-tip diameter.
    pub diameter: Parameter<f64>,
    /// Geometric pitch.
    pub pitch: Option<Parameter<f64>>,
    /// Number of blades.
    pub blades: Parameter<u32>,
    /// Largest static thrust with its motor, used for the thrust-to-weight check.
    pub max_thrust: Parameter<f64>,
    /// Mass.
    pub mass: Option<Parameter<f64>>,
    /// Moment of inertia about its spin axis.
    pub inertia: Option<Parameter<f64>>,
    /// Where its measured performance is, such as a UIUC propeller database entry.
    pub data: Option<String>,
}

/// A rotor, in SI units.
#[derive(Clone, Debug, PartialEq)]
pub struct Rotor {
    /// What it is called.
    pub name: String,
    /// The hub, body axes, m.
    pub position_m: Vec3<Body>,
    /// The unit direction of thrust, body axes.
    pub axis: Vec3<Body>,
    /// Which way it turns.
    pub spin: Spin,
    /// The key of its propeller.
    pub propeller: String,
    /// The key of its motor.
    pub motor: Option<String>,
    /// The key of its ESC.
    pub esc: Option<String>,
}

/// A propeller, in SI units.
#[derive(Clone, Debug, PartialEq)]
pub struct Propeller {
    /// Diameter, m.
    pub diameter_m: f64,
    /// Geometric pitch, m.
    pub pitch_m: Option<f64>,
    /// Number of blades.
    pub blades: u32,
    /// Largest static thrust, N.
    pub max_thrust_n: f64,
    /// Mass, kg.
    pub mass_kg: Option<f64>,
    /// Moment of inertia about the spin axis, kg·m².
    pub inertia_kg_m2: Option<f64>,
    /// Where its measured performance is.
    pub data: Option<String>,
}

/// The keys of the propellers, motors and ESCs a rotor may name.
pub struct Catalog<'a> {
    /// Keys in `[propellers]`.
    pub propellers: Vec<&'a str>,
    /// Keys in `[motors]`.
    pub motors: Vec<&'a str>,
    /// Keys in `[escs]`.
    pub escs: Vec<&'a str>,
}

impl RotorConfig {
    /// The rotor in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] naming the field at fault: an empty name, a bad parameter, a
    /// zero axis, or a propeller, motor or ESC key that is not defined.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
        catalog: &Catalog<'_>,
    ) -> Result<Rotor, EngineError> {
        if self.name.trim().is_empty() {
            return Err(resolver.error(path.key("name"), "must not be empty"));
        }
        let position_m = resolver.si(&path.key("position"), &self.position, LENGTH_M)?;
        let axis = resolver.si(&path.key("axis"), &self.axis, PURE)?;
        let axis = Vec3::from_array(axis)
            .normalized()
            .ok_or_else(|| resolver.error(path.key("axis").key("value"), "must not be zero"))?;
        let references = [
            (
                "propeller",
                Some(self.propeller.as_str()),
                &catalog.propellers,
                "propellers",
            ),
            ("motor", self.motor.as_deref(), &catalog.motors, "motors"),
            ("esc", self.esc.as_deref(), &catalog.escs, "escs"),
        ];
        for (field, key, keys, table) in references {
            if let Some(key) = key.filter(|key| !keys.contains(key)) {
                let reason = format!("no entry `{key}` in [{table}]");
                return Err(resolver.error(path.key(field), reason));
            }
        }
        Ok(Rotor {
            name: self.name.clone(),
            position_m: Vec3::from_array(position_m),
            axis,
            spin: self.spin,
            propeller: self.propeller.clone(),
            motor: self.motor.clone(),
            esc: self.esc.clone(),
        })
    }
}

impl PropellerConfig {
    /// The propeller in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] naming the field at fault: a bad parameter, or a diameter,
    /// thrust, mass or inertia that is not positive.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<Propeller, EngineError> {
        let diameter_m = positive(resolver, path, "diameter", &self.diameter, LENGTH_M)?;
        let pitch_m = optional(resolver, path, "pitch", self.pitch.as_ref(), LENGTH_M)?;
        let blades = resolver.count(&path.key("blades"), &self.blades)?;
        let max_thrust_n = positive(resolver, path, "max_thrust", &self.max_thrust, FORCE_N)?;
        let mass_kg = optional(resolver, path, "mass", self.mass.as_ref(), MASS_KG)?;
        let inertia_kg_m2 = optional(
            resolver,
            path,
            "inertia",
            self.inertia.as_ref(),
            INERTIA_KG_M2,
        )?;
        Ok(Propeller {
            diameter_m,
            pitch_m,
            blades,
            max_thrust_n,
            mass_kg,
            inertia_kg_m2,
            data: self.data.clone(),
        })
    }
}

/// The parameter `field` below `path` in SI units, which must be positive.
///
/// # Errors
///
/// [`EngineError::Config`] for a bad parameter or a value that is not positive.
pub fn positive(
    resolver: &mut Resolver<'_>,
    path: &FieldPath,
    field: &str,
    parameter: &Parameter<f64>,
    quantity: super::units::Quantity,
) -> Result<f64, EngineError> {
    let at = path.key(field);
    let value = resolver.si(&at, parameter, quantity)?;
    if value > 0.0 {
        return Ok(value);
    }
    let reason = format!("must be positive; found {}", parameter.value);
    Err(resolver.error(at.key("value"), reason))
}

/// [`positive`] for an optional parameter.
///
/// # Errors
///
/// As [`positive`].
pub fn optional(
    resolver: &mut Resolver<'_>,
    path: &FieldPath,
    field: &str,
    parameter: Option<&Parameter<f64>>,
    quantity: super::units::Quantity,
) -> Result<Option<f64>, EngineError> {
    parameter
        .map(|parameter| positive(resolver, path, field, parameter, quantity))
        .transpose()
}

/// Every propeller, by key.
///
/// # Errors
///
/// The first propeller's error.
pub fn resolve_propellers(
    propellers: &BTreeMap<String, PropellerConfig>,
    resolver: &mut Resolver<'_>,
) -> Result<BTreeMap<String, Propeller>, EngineError> {
    let path = FieldPath::root().key("propellers");
    propellers
        .iter()
        .map(|(key, propeller)| Ok((key.clone(), propeller.resolve(resolver, &path.key(key))?)))
        .collect()
}
