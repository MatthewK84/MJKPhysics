//! The `[mass]` table: mass properties in body axes (FRD), given directly or built up from
//! components.
//!
//! Directly: `mass`, `cg` (the center of mass from the airframe's reference point) and `inertia`
//! (about the center of mass). Built up: `[[mass.components]]`, each a point, box or cylinder
//! with a mass and a position, combined by the parallel-axis theorem in `mjk-dynamics`.

use mjk_core::{EngineError, FieldPath};
use mjk_dynamics::mass::{
    Component, ComponentProblem, CompositeError, InertiaProblem, MassError, MassProperties, Shape,
    composite,
};
use mjk_math::{EulerAngles, Mat3, Rotation, Vec3};
use serde::{Deserialize, Serialize};

use super::parameter::{Parameter, Resolver};
use super::units::quantities::{ANGLE_RAD, INERTIA_KG_M2, LENGTH_M, MASS_KG};

/// Mass properties of a rigid airframe.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MassConfig {
    /// Total mass.
    pub mass: Option<Parameter<f64>>,
    /// Center of mass from the airframe's reference point, body axes.
    pub cg: Option<Parameter<[f64; 3]>>,
    /// Inertia tensor about the center of mass, body axes; `inertia[row][col]`.
    pub inertia: Option<Parameter<[[f64; 3]; 3]>>,
    /// Parts whose masses add up to the airframe's, instead of the three fields above.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<ComponentConfig>,
}

/// The shape of a component.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum ShapeKind {
    /// All its mass at one point.
    Point,
    /// A solid box; needs `size`.
    Box,
    /// A solid cylinder along the part's z axis; needs `radius` and `length`.
    Cylinder,
}

impl ShapeKind {
    /// As written in a file.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Point => "point",
            Self::Box => "box",
            Self::Cylinder => "cylinder",
        }
    }
}

/// One part of a built-up airframe.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentConfig {
    /// What it is, such as `"battery"`.
    pub name: String,
    /// Its mass.
    pub mass: Parameter<f64>,
    /// Its center from the airframe's reference point, body axes.
    pub position: Parameter<[f64; 3]>,
    /// Its shape.
    pub shape: ShapeKind,
    /// Edge lengths of a box, along the part's x, y and z axes.
    pub size: Option<Parameter<[f64; 3]>>,
    /// Radius of a cylinder.
    pub radius: Option<Parameter<f64>>,
    /// Length of a cylinder.
    pub length: Option<Parameter<f64>>,
    /// Roll, pitch and yaw of the part's axes from body axes; none means aligned.
    pub orientation: Option<Parameter<[f64; 3]>>,
}

impl MassConfig {
    /// The mass properties.
    ///
    /// # Errors
    ///
    /// [`EngineError::Config`] naming the field at fault: a direct table missing a field, both
    /// forms at once, or values that are not a possible rigid body.
    pub fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<MassProperties, EngineError> {
        let given = [
            self.mass.is_some(),
            self.cg.is_some(),
            self.inertia.is_some(),
        ];
        if !self.components.is_empty() {
            if given.contains(&true) {
                let reason = "give either mass, cg and inertia, or components, not both";
                return Err(resolver.error(path.clone(), reason));
            }
            return resolve_components(&self.components, resolver, &path.key("components"));
        }
        let (Some(mass), Some(cg), Some(inertia)) = (&self.mass, &self.cg, &self.inertia) else {
            let missing = ["mass", "cg", "inertia"]
                .into_iter()
                .zip(given)
                .find_map(|(name, present)| (!present).then_some(name))
                .unwrap_or("mass");
            let reason =
                format!("missing `{missing}`: give mass, cg and inertia, or [[mass.components]]");
            return Err(resolver.error(path.clone(), reason));
        };
        let mass_kg = resolver.si(&path.key("mass"), mass, MASS_KG)?;
        let cg_m = resolver.si(&path.key("cg"), cg, LENGTH_M)?;
        let inertia_kg_m2 = resolver.si(&path.key("inertia"), inertia, INERTIA_KG_M2)?;
        MassProperties::new(
            mass_kg,
            Vec3::from_array(cg_m),
            Mat3::from_rows(inertia_kg_m2),
        )
        .map_err(|error| mass_error(resolver, path, error))
    }
}

fn mass_error(resolver: &Resolver<'_>, path: &FieldPath, error: MassError) -> EngineError {
    let reason = error.to_string();
    let located = match error {
        MassError::Mass { .. } => path.key("mass").key("value"),
        MassError::CenterOfMass { axis } => path.key("cg").key("value").index(axis),
        MassError::Inertia(problem) => inertia_path(&path.key("inertia").key("value"), problem),
        MassError::Singular => path.key("inertia").key("value"),
    };
    resolver.error(located, reason)
}

fn inertia_path(path: &FieldPath, problem: InertiaProblem) -> FieldPath {
    match problem {
        InertiaProblem::NotFinite { row, col } | InertiaProblem::NotSymmetric { row, col, .. } => {
            path.index(row).index(col)
        }
        InertiaProblem::NotPositiveDefinite { .. }
        | InertiaProblem::ViolatesTriangleInequality { .. } => path.clone(),
    }
}

fn resolve_components(
    components: &[ComponentConfig],
    resolver: &mut Resolver<'_>,
    path: &FieldPath,
) -> Result<MassProperties, EngineError> {
    let parts = components
        .iter()
        .enumerate()
        .map(|(index, component)| component.resolve(resolver, &path.index(index)))
        .collect::<Result<Vec<Component>, EngineError>>()?;
    composite(&parts).map_err(|error| composite_error(resolver, path, error))
}

fn composite_error(
    resolver: &Resolver<'_>,
    path: &FieldPath,
    error: CompositeError,
) -> EngineError {
    let reason = error.to_string();
    let located = match error {
        CompositeError::Component { index, problem } => {
            let component = path.index(index);
            match problem {
                ComponentProblem::Mass { .. } => component.key("mass").key("value"),
                ComponentProblem::Dimension { .. } => component,
                ComponentProblem::Position { axis } => {
                    component.key("position").key("value").index(axis)
                }
            }
        }
        CompositeError::Total(_) => path.clone(),
    };
    resolver.error(located, reason)
}

impl ComponentConfig {
    fn resolve(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<Component, EngineError> {
        if self.name.trim().is_empty() {
            return Err(resolver.error(path.key("name"), "must not be empty"));
        }
        let mass_kg = resolver.si(&path.key("mass"), &self.mass, MASS_KG)?;
        let position_m = resolver.si(&path.key("position"), &self.position, LENGTH_M)?;
        let shape = self.shape(resolver, path)?;
        let component = Component::new(mass_kg, shape, Vec3::from_array(position_m));
        let Some(orientation) = &self.orientation else {
            return Ok(component);
        };
        let [roll_rad, pitch_rad, yaw_rad] =
            resolver.si(&path.key("orientation"), orientation, ANGLE_RAD)?;
        Ok(component.rotated(Rotation::from_euler(EulerAngles {
            yaw_rad,
            pitch_rad,
            roll_rad,
        })))
    }

    fn shape(&self, resolver: &mut Resolver<'_>, path: &FieldPath) -> Result<Shape, EngineError> {
        let fields = [
            ("size", self.size.is_some()),
            ("radius", self.radius.is_some()),
            ("length", self.length.is_some()),
        ];
        let needs: &[&str] = match self.shape {
            ShapeKind::Point => &[],
            ShapeKind::Box => &["size"],
            ShapeKind::Cylinder => &["radius", "length"],
        };
        let kind = self.shape.name();
        let wrong = fields
            .into_iter()
            .find(|(name, present)| *present != needs.contains(name));
        if let Some((name, present)) = wrong {
            let verb = if present { "has no" } else { "needs a" };
            return Err(resolver.error(path.key(name), format!("a {kind} {verb} {name}")));
        }
        self.dimensions(resolver, path)
    }

    fn dimensions(
        &self,
        resolver: &mut Resolver<'_>,
        path: &FieldPath,
    ) -> Result<Shape, EngineError> {
        if let Some(size) = &self.size {
            let size_m = resolver.si(&path.key("size"), size, LENGTH_M)?;
            return Ok(Shape::Box { size_m });
        }
        if let (Some(radius), Some(length)) = (&self.radius, &self.length) {
            let radius_m = resolver.si(&path.key("radius"), radius, LENGTH_M)?;
            let length_m = resolver.si(&path.key("length"), length, LENGTH_M)?;
            return Ok(Shape::Cylinder { radius_m, length_m });
        }
        Ok(Shape::PointMass)
    }
}
