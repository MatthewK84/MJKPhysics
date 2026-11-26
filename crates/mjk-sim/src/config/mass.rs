//! The `[mass]` table: mass, center of mass and inertia tensor.

use mjk_core::{EngineError, FieldPath};
use mjk_dynamics::mass::{InertiaProblem, check_inertia};
use serde::{Deserialize, Serialize};

/// Mass properties of a rigid airframe, in body axes (FRD).
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MassConfig {
    /// Total mass.
    pub mass_kg: f64,
    /// Center of mass relative to the airframe reference point, body axes.
    pub cg_m: [f64; 3],
    /// Inertia tensor about the center of mass, body axes; `inertia_kg_m2[row][col]`.
    pub inertia_kg_m2: [[f64; 3]; 3],
}

impl MassConfig {
    /// Checks that the values describe a physically possible rigid body.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::Config`] naming the first implausible field.
    pub fn validate(&self, document: &str, path: &FieldPath) -> Result<(), EngineError> {
        if !(self.mass_kg.is_finite() && self.mass_kg > 0.0) {
            let reason = format!(
                "must be a positive, finite mass in kilograms; found {}",
                self.mass_kg
            );
            return Err(EngineError::config(document, path.key("mass_kg"), reason));
        }
        if let Some(axis) = self.cg_m.iter().position(|value| !value.is_finite()) {
            let reason = "must be finite".to_owned();
            return Err(EngineError::config(
                document,
                path.key("cg_m").index(axis),
                reason,
            ));
        }
        check_inertia(&self.inertia_kg_m2)
            .map_err(|problem| inertia_error(document, &path.key("inertia_kg_m2"), problem))
    }
}

fn inertia_error(document: &str, path: &FieldPath, problem: InertiaProblem) -> EngineError {
    let located = match problem {
        InertiaProblem::NotFinite { row, col } | InertiaProblem::NotSymmetric { row, col, .. } => {
            path.index(row).index(col)
        }
        InertiaProblem::NotPositiveDefinite { .. }
        | InertiaProblem::ViolatesTriangleInequality { .. } => path.clone(),
    };
    EngineError::config(document, located, problem.to_string())
}
