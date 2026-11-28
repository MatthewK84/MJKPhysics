//! From an airframe's motors and ESCs to the models that simulate them
//! ([#19](https://github.com/MatthewK84/MJKPhysics/issues/19)).
//!
//! An electrical motor turns its own rotor and its propeller, so a rotor with one needs at least
//! one of those inertias; loading checks that. A rotor without an ESC gets an ideal one: no
//! latency, no slew limit, and active braking.

use std::fmt;

use mjk_aero::propulsion::{
    Braking as ModelBraking, DcMotor, Drive, Esc, EscError, LagMotor, MotorError, MotorModel,
};
use mjk_core::{EngineError, FieldPath};

use super::airframe::Airframe;
use super::parameter::Resolver;
use super::propulsion::{Braking, Esc as EscConfig, Motor};
use super::rotors::Rotor;

/// A rotor's motor and ESC, ready to simulate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveTrain {
    /// The motor.
    pub motor: MotorModel,
    /// The ESC.
    pub esc: Esc,
}

/// Why a rotor's drive train could not be built.
#[derive(Clone, Debug, PartialEq)]
pub enum DriveError {
    /// The motor's parameters are not usable.
    Motor {
        /// Its key in `[motors]`.
        key: String,
        /// What is wrong.
        error: MotorError,
    },
    /// The ESC's parameters are not usable at this physics rate.
    Esc {
        /// Its key in `[escs]`, or `None` for the ideal ESC.
        key: Option<String>,
        /// What is wrong.
        error: EscError,
    },
}

impl fmt::Display for DriveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Motor { key, error } => write!(f, "motor `{key}`: {error}"),
            Self::Esc {
                key: Some(key),
                error,
            } => write!(f, "ESC `{key}`: {error}"),
            Self::Esc { key: None, error } => write!(f, "the ideal ESC: {error}"),
        }
    }
}

/// The rotor's motor, if it has one.
fn motor_of<'a>(airframe: &'a Airframe, rotor: &Rotor) -> Option<(&'a String, &'a Motor)> {
    let key = rotor.motor.as_ref()?;
    airframe.motors.get_key_value(key)
}

/// The inertia a rotor's motor turns: its own rotor's and the propeller's, kg·m².
fn turned_inertia(airframe: &Airframe, rotor: &Rotor) -> f64 {
    let motor = motor_of(airframe, rotor)
        .and_then(|(_, motor)| motor.inertia_kg_m2)
        .unwrap_or(0.0);
    let propeller = airframe
        .propellers
        .get(&rotor.propeller)
        .and_then(|propeller| propeller.inertia_kg_m2)
        .unwrap_or(0.0);
    motor + propeller
}

/// Every rotor with an electrical motor turns some inertia.
///
/// # Errors
///
/// [`EngineError::Config`] at the first rotor whose electrical motor turns no inertia.
pub fn check_drive_trains(resolver: &Resolver<'_>, airframe: &Airframe) -> Result<(), EngineError> {
    let missing = airframe
        .rotors
        .iter()
        .enumerate()
        .find_map(|(index, rotor)| {
            let (key, motor) = motor_of(airframe, rotor)?;
            let without = motor.kv_rad_s_v.is_some() && turned_inertia(airframe, rotor) <= 0.0;
            without.then_some((index, key))
        });
    missing.map_or(Ok(()), |(index, key)| {
        let reason = format!(
            "motor `{key}` is electrical, so it needs the inertia it turns: give the motor's \
             inertia, the propeller's, or both"
        );
        Err(resolver.error(FieldPath::root().key("rotors").index(index), reason))
    })
}

/// The models of a rotor's motor and ESC at physics rate `physics_hz`, or `None` for a rotor
/// with no motor.
///
/// # Errors
///
/// A [`DriveError`] if the motor or ESC is not usable, such as a latency too long to count in
/// ticks at this rate.
pub fn drive_train(
    airframe: &Airframe,
    rotor: &Rotor,
    physics_hz: u32,
) -> Result<Option<DriveTrain>, DriveError> {
    let Some((key, motor)) = motor_of(airframe, rotor) else {
        return Ok(None);
    };
    let model =
        motor_model(motor, turned_inertia(airframe, rotor)).map_err(|error| DriveError::Motor {
            key: key.clone(),
            error,
        })?;
    let esc_config = rotor
        .esc
        .as_ref()
        .and_then(|esc_key| airframe.escs.get(esc_key));
    let esc = esc_model(motor, esc_config, physics_hz).map_err(|error| DriveError::Esc {
        key: rotor.esc.clone(),
        error,
    })?;
    Ok(Some(DriveTrain { motor: model, esc }))
}

/// The DC-equivalent model when the electrical constants are given, otherwise the lag tier.
fn motor_model(motor: &Motor, inertia_kg_m2: f64) -> Result<MotorModel, MotorError> {
    let electrical = (
        motor.kv_rad_s_v,
        motor.resistance_ohm,
        motor.no_load_current_a,
    );
    if let (Some(kv), Some(resistance), Some(no_load)) = electrical {
        return DcMotor::new(kv, resistance, no_load, inertia_kg_m2).map(MotorModel::Dc);
    }
    let time_constant = motor.time_constant_s.unwrap_or(f64::NAN);
    let max_speed = motor.max_speed_rad_s.unwrap_or(f64::NAN);
    LagMotor::new(time_constant, max_speed).map(MotorModel::Lag)
}

/// The ESC model: the file's ESC, or an ideal one; the current limit is the smaller of the
/// motor's and the ESC's.
fn esc_model(motor: &Motor, esc: Option<&EscConfig>, physics_hz: u32) -> Result<Esc, EscError> {
    let limit = [motor.max_current_a, esc.and_then(|esc| esc.max_current_a)]
        .into_iter()
        .flatten()
        .reduce(f64::min);
    let braking = match esc.map(|esc| esc.braking) {
        Some(Braking::None) => ModelBraking::None,
        Some(Braking::Active) | None => ModelBraking::Active,
    };
    let drive = Drive {
        braking,
        current_limit_a: limit,
    };
    let latency_s = esc.and_then(|esc| esc.latency_s).unwrap_or(0.0);
    let slew = esc.and_then(|esc| esc.slew_rate_per_s);
    Esc::new(latency_s, slew, drive, physics_hz)
}
