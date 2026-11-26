//! The `[mass]` table: mass, center of mass and inertia tensor.

use mjk_core::{EngineError, FieldPath};
use mjk_dynamics::mass::{InertiaProblem, MassError, MassProperties};
use mjk_math::{Mat3, Vec3};
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
    /// The validated mass properties of the airframe.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::Config`] naming the first implausible field: the mass, a
    /// center-of-mass coordinate, or the inertia tensor (an entry, for a non-finite or
    /// asymmetric one).
    pub fn mass_properties(
        &self,
        document: &str,
        path: &FieldPath,
    ) -> Result<MassProperties, EngineError> {
        let cg_m = Vec3::from_array(self.cg_m);
        let inertia = Mat3::from_rows(self.inertia_kg_m2);
        MassProperties::new(self.mass_kg, cg_m, inertia)
            .map_err(|error| mass_error(document, path, error))
    }

    /// Checks that the values describe a physically possible rigid body.
    ///
    /// # Errors
    ///
    /// Returns [`EngineError::Config`] naming the first implausible field.
    pub fn validate(&self, document: &str, path: &FieldPath) -> Result<(), EngineError> {
        self.mass_properties(document, path).map(|_| ())
    }
}

fn mass_error(document: &str, path: &FieldPath, error: MassError) -> EngineError {
    let reason = error.to_string();
    match error {
        MassError::Mass { .. } => EngineError::config(document, path.key("mass_kg"), reason),
        MassError::CenterOfMass { axis } => {
            EngineError::config(document, path.key("cg_m").index(axis), reason)
        }
        MassError::Inertia(problem) => inertia_error(document, &path.key("inertia_kg_m2"), problem),
        MassError::Singular => EngineError::config(document, path.key("inertia_kg_m2"), reason),
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
