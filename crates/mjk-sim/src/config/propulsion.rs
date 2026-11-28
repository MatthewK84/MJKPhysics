//! Motors, ESCs and the battery ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)).
//!
//! A motor is described either by its electrical constants (speed constant, winding resistance
//! and no-load current, all three) or only by its time constant, for airframes where only that
//! is published; the models that use them arrive with
//! [#19](https://github.com/MatthewK84/MJKPhysics/issues/19) and
//! [#20](https://github.com/MatthewK84/MJKPhysics/issues/20).

use std::collections::BTreeMap;

use mjk_core::{EngineError, FieldPath};
use serde::{Deserialize, Serialize};

use super::parameter::{Parameter, Resolver};
use super::rotors::{optional, positive};
use super::units::quantities::{
    ANGULAR_RATE_RAD_S, CHARGE_C, CURRENT_A, INERTIA_KG_M2, MASS_KG, RATE_PER_S, RESISTANCE_OHM,
    SPEED_CONSTANT, TIME_S,
};

/// A motor as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MotorConfig {
    /// Speed constant, such as rpm/V.
    pub kv: Option<Parameter<f64>>,
    /// Winding resistance.
    pub resistance: Option<Parameter<f64>>,
    /// Current with no load.
    pub no_load_current: Option<Parameter<f64>>,
    /// First-order time constant of its speed, for airframes where only that is published.
    pub time_constant: Option<Parameter<f64>>,
    /// Speed at full duty, for the time-constant tier: speed follows duty times this.
    pub max_speed: Option<Parameter<f64>>,
    /// Largest continuous current.
    pub max_current: Option<Parameter<f64>>,
    /// Moment of inertia of its rotor about the spin axis.
    pub inertia: Option<Parameter<f64>>,
    /// Mass.
    pub mass: Option<Parameter<f64>>,
}

/// A motor, in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motor {
    /// Speed constant, rad/(s·V).
    pub kv_rad_s_v: Option<f64>,
    /// Winding resistance, Ω.
    pub resistance_ohm: Option<f64>,
    /// No-load current, A.
    pub no_load_current_a: Option<f64>,
    /// Time constant, s.
    pub time_constant_s: Option<f64>,
    /// Speed at full duty, for the time-constant tier, rad/s.
    pub max_speed_rad_s: Option<f64>,
    /// Largest continuous current, A.
    pub max_current_a: Option<f64>,
    /// Rotor inertia, kg·m².
    pub inertia_kg_m2: Option<f64>,
    /// Mass, kg.
    pub mass_kg: Option<f64>,
}

impl MotorConfig {
    /// The motor in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] naming the field at fault: only some of the electrical
    /// constants, neither description, or a bad parameter.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<Motor, EngineError> {
        let electrical = [
            self.kv.is_some(),
            self.resistance.is_some(),
            self.no_load_current.is_some(),
        ];
        if electrical.contains(&true) && electrical.contains(&false) {
            let reason = "give kv, resistance and no_load_current together";
            return Err(resolver.error(path.clone(), reason));
        }
        if self.time_constant.is_some() != self.max_speed.is_some() {
            let reason = "give time_constant and max_speed together";
            return Err(resolver.error(path.clone(), reason));
        }
        if !electrical.contains(&true) && self.time_constant.is_none() {
            let reason = "give kv, resistance and no_load_current, or time_constant and max_speed";
            return Err(resolver.error(path.clone(), reason));
        }
        let mut motor = self.values(resolver, path)?;
        motor.max_speed_rad_s = optional(
            resolver,
            path,
            "max_speed",
            self.max_speed.as_ref(),
            ANGULAR_RATE_RAD_S,
        )?;
        Ok(motor)
    }

    fn values(&self, resolver: &mut Resolver<'_>, path: &FieldPath) -> Result<Motor, EngineError> {
        Ok(Motor {
            max_speed_rad_s: None,
            kv_rad_s_v: optional(resolver, path, "kv", self.kv.as_ref(), SPEED_CONSTANT)?,
            resistance_ohm: optional(
                resolver,
                path,
                "resistance",
                self.resistance.as_ref(),
                RESISTANCE_OHM,
            )?,
            no_load_current_a: optional(
                resolver,
                path,
                "no_load_current",
                self.no_load_current.as_ref(),
                CURRENT_A,
            )?,
            time_constant_s: optional(
                resolver,
                path,
                "time_constant",
                self.time_constant.as_ref(),
                TIME_S,
            )?,
            max_current_a: optional(
                resolver,
                path,
                "max_current",
                self.max_current.as_ref(),
                CURRENT_A,
            )?,
            inertia_kg_m2: optional(
                resolver,
                path,
                "inertia",
                self.inertia.as_ref(),
                INERTIA_KG_M2,
            )?,
            mass_kg: optional(resolver, path, "mass", self.mass.as_ref(), MASS_KG)?,
        })
    }
}

/// What an ESC does to a motor when the command drops.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Braking {
    /// Active braking: the motor is driven down.
    Active,
    /// None: the motor coasts down.
    None,
}

/// An ESC as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct EscConfig {
    /// Delay from command to output.
    pub latency: Option<Parameter<f64>>,
    /// Fastest change of output, as a fraction of full scale per second.
    pub slew_rate: Option<Parameter<f64>>,
    /// What happens when the command drops.
    pub braking: Braking,
    /// Largest continuous current.
    pub max_current: Option<Parameter<f64>>,
}

/// An ESC, in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Esc {
    /// Latency, s.
    pub latency_s: Option<f64>,
    /// Slew rate, full scale per second.
    pub slew_rate_per_s: Option<f64>,
    /// Braking.
    pub braking: Braking,
    /// Largest continuous current, A.
    pub max_current_a: Option<f64>,
}

impl EscConfig {
    /// The ESC in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] naming the field at fault.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<Esc, EngineError> {
        Ok(Esc {
            latency_s: optional(resolver, path, "latency", self.latency.as_ref(), TIME_S)?,
            slew_rate_per_s: optional(
                resolver,
                path,
                "slew_rate",
                self.slew_rate.as_ref(),
                RATE_PER_S,
            )?,
            braking: self.braking,
            max_current_a: optional(
                resolver,
                path,
                "max_current",
                self.max_current.as_ref(),
                CURRENT_A,
            )?,
        })
    }
}

/// The battery as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BatteryConfig {
    /// Cells in series.
    pub cells: Parameter<u32>,
    /// Rated capacity, such as mAh.
    pub capacity: Parameter<f64>,
    /// Internal resistance of the whole pack.
    pub internal_resistance: Option<Parameter<f64>>,
}

/// The battery, in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Battery {
    /// Cells in series.
    pub cells: u32,
    /// Capacity, A·s.
    pub capacity_a_s: f64,
    /// Internal resistance, Ω.
    pub internal_resistance_ohm: Option<f64>,
}

impl BatteryConfig {
    /// The battery in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] naming the field at fault.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<Battery, EngineError> {
        Ok(Battery {
            cells: resolver.count(&path.key("cells"), &self.cells)?,
            capacity_a_s: positive(resolver, path, "capacity", &self.capacity, CHARGE_C)?,
            internal_resistance_ohm: optional(
                resolver,
                path,
                "internal_resistance",
                self.internal_resistance.as_ref(),
                RESISTANCE_OHM,
            )?,
        })
    }
}

/// Every entry of a table of motors or ESCs, by key.
///
/// # Errors
///
/// The first entry's error.
pub fn resolve_table<C, T>(
    table: &BTreeMap<String, C>,
    name: &str,
    resolver: &mut Resolver<'_>,
    resolve: impl Fn(&C, &mut Resolver<'_>, &FieldPath) -> Result<T, EngineError>,
) -> Result<BTreeMap<String, T>, EngineError> {
    let path = FieldPath::root().key(name);
    table
        .iter()
        .map(|(key, entry)| Ok((key.clone(), resolve(entry, resolver, &path.key(key))?)))
        .collect()
}
