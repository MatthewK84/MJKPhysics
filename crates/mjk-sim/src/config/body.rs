//! Body drag, controller gains and visual hints
//! ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)).
//!
//! The format stores these and checks their units; the models that read them arrive with body
//! aerodynamics ([#24](https://github.com/MatthewK84/MJKPhysics/issues/24)), the flight
//! controllers ([#26](https://github.com/MatthewK84/MJKPhysics/issues/26),
//! [#27](https://github.com/MatthewK84/MJKPhysics/issues/27)) and the renderer (M3).

use std::collections::BTreeMap;

use mjk_core::{EngineError, FieldPath};
use serde::{Deserialize, Serialize};

use super::parameter::{Parameter, Resolver};
use super::rotors::optional;
use super::units::quantities::{AREA_M2, LENGTH_M};

/// Body drag as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DragConfig {
    /// Drag area, drag coefficient times reference area, along each body axis.
    pub area: Parameter<[f64; 3]>,
}

/// Body drag, in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyDrag {
    /// Drag area along each body axis, m².
    pub area_m2: [f64; 3],
}

impl DragConfig {
    /// The drag in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] for a bad parameter or a negative area.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<BodyDrag, EngineError> {
        let at = path.key("area");
        let area_m2 = resolver.si(&at, &self.area, AREA_M2)?;
        if let Some(axis) = area_m2.iter().position(|area| *area < 0.0) {
            let reason = "must not be negative";
            return Err(resolver.error(at.key("value").index(axis), reason));
        }
        Ok(BodyDrag { area_m2 })
    }
}

/// The flight controller as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerConfig {
    /// Which controller, such as `"cascaded-pid"`.
    pub kind: String,
    /// Its gains by name, each in its own unit.
    #[serde(default)]
    pub gains: BTreeMap<String, Parameter<f64>>,
}

/// The flight controller, gains in SI units.
#[derive(Clone, Debug, PartialEq)]
pub struct Controller {
    /// Which controller.
    pub kind: String,
    /// Its gains by name, SI.
    pub gains: BTreeMap<String, f64>,
}

impl ControllerConfig {
    /// The controller with its gains in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] for an empty kind or a gain with a bad unit or value.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<Controller, EngineError> {
        if self.kind.trim().is_empty() {
            return Err(resolver.error(path.key("kind"), "must not be empty"));
        }
        let gains_path = path.key("gains");
        let gains = self
            .gains
            .iter()
            .map(|(name, gain)| Ok((name.clone(), resolver.any(&gains_path.key(name), gain)?)))
            .collect::<Result<BTreeMap<String, f64>, EngineError>>()?;
        Ok(Controller {
            kind: self.kind.clone(),
            gains,
        })
    }
}

/// Hints for drawing the airframe, as written.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct VisualConfig {
    /// Size of the central body, along each body axis.
    pub body_size: Option<Parameter<[f64; 3]>>,
    /// Width of the arms.
    pub arm_width: Option<Parameter<f64>>,
    /// Color of the body, `#rrggbb`.
    pub color: Option<String>,
}

/// Hints for drawing the airframe, in SI units.
#[derive(Clone, Debug, PartialEq)]
pub struct Visual {
    /// Size of the central body, m.
    pub body_size_m: Option<[f64; 3]>,
    /// Width of the arms, m.
    pub arm_width_m: Option<f64>,
    /// Color, `#rrggbb`.
    pub color: Option<String>,
}

impl VisualConfig {
    /// The hints in SI units.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] for a bad parameter, a size that is not positive, or a color
    /// that is not `#` and six hexadecimal digits.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<Visual, EngineError> {
        let body_size_m = self
            .body_size
            .as_ref()
            .map(|size| resolver.si(&path.key("body_size"), size, LENGTH_M))
            .transpose()?;
        if let Some(axis) = body_size_m.and_then(|size| size.iter().position(|x| *x <= 0.0)) {
            let at = path.key("body_size").key("value").index(axis);
            return Err(resolver.error(at, "must be positive"));
        }
        let arm_width_m = optional(
            resolver,
            path,
            "arm_width",
            self.arm_width.as_ref(),
            LENGTH_M,
        )?;
        if let Some(color) = self.color.as_ref().filter(|color| !is_color(color)) {
            let reason = format!("a color is # and six hexadecimal digits; found \"{color}\"");
            return Err(resolver.error(path.key("color"), reason));
        }
        Ok(Visual {
            body_size_m,
            arm_width_m,
            color: self.color.clone(),
        })
    }
}

fn is_color(text: &str) -> bool {
    text.strip_prefix('#')
        .is_some_and(|digits| digits.len() == 6 && digits.bytes().all(|b| b.is_ascii_hexdigit()))
}
