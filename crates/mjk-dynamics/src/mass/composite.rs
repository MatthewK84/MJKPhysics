//! Mass properties of a body built from simple components, Eqs. (RB-4) to (RB-7).

use std::fmt;

use mjk_math::{Body, Mat3, Part, Rotation, Vec3};

use super::properties::{MassError, MassProperties};

/// The shape of a uniform-density component, in its own part axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    /// All mass at one point, such as a small fastener or a sensor.
    PointMass,
    /// A solid rectangular box with edge lengths along the part's x, y and z axes, Eq. (RB-6).
    Box {
        /// Edge lengths along x, y and z.
        size_m: [f64; 3],
    },
    /// A solid circular cylinder with its axis along the part's z axis, Eq. (RB-7).
    Cylinder {
        /// Radius.
        radius_m: f64,
        /// Length along the axis.
        length_m: f64,
    },
}

impl Shape {
    /// Inertia of a component of this shape about its own center, in part axes.
    fn central_inertia(self, mass_kg: f64) -> Mat3<Part> {
        match self {
            Self::PointMass => Mat3::zero(),
            Self::Box { size_m: [a, b, c] } => {
                // Eq. (RB-6)
                let k = mass_kg / 12.0;
                Mat3::from_diagonal([
                    k * (b * b + c * c),
                    k * (a * a + c * c),
                    k * (a * a + b * b),
                ])
            }
            Self::Cylinder { radius_m, length_m } => {
                // Eq. (RB-7)
                let r2 = radius_m * radius_m;
                let transverse = mass_kg * (3.0 * r2 + length_m * length_m) / 12.0;
                Mat3::from_diagonal([transverse, transverse, 0.5 * mass_kg * r2])
            }
        }
    }

    /// The dimensions that must be finite and not negative.
    fn dimensions(self) -> [f64; 3] {
        match self {
            Self::PointMass => [0.0; 3],
            Self::Box { size_m } => size_m,
            Self::Cylinder { radius_m, length_m } => [radius_m, length_m, 0.0],
        }
    }
}

/// One component of a composite body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Component {
    /// Mass of the component.
    pub mass_kg: f64,
    /// Its shape, in its own part axes.
    pub shape: Shape,
    /// Its center, from the airframe reference point, in body axes.
    pub position_m: Vec3<Body>,
    /// The rotation from its part axes to body axes.
    pub orientation: Rotation<Body, Part>,
}

impl Component {
    /// A component whose part axes are aligned with the body axes.
    #[must_use]
    pub const fn new(mass_kg: f64, shape: Shape, position_m: Vec3<Body>) -> Self {
        Self {
            mass_kg,
            shape,
            position_m,
            orientation: Rotation::identity(),
        }
    }

    /// The same component, turned by `orientation` from part axes to body axes.
    #[must_use]
    pub const fn rotated(self, orientation: Rotation<Body, Part>) -> Self {
        Self {
            orientation,
            ..self
        }
    }

    fn check(&self) -> Result<(), ComponentProblem> {
        if !(self.mass_kg.is_finite() && self.mass_kg > 0.0) {
            return Err(ComponentProblem::Mass {
                value: self.mass_kg,
            });
        }
        let dimensions = self.shape.dimensions();
        if let Some(&value) = dimensions.iter().find(|d| !(d.is_finite() && **d >= 0.0)) {
            return Err(ComponentProblem::Dimension { value });
        }
        let position = self.position_m.to_array();
        match position.iter().position(|value| !value.is_finite()) {
            Some(axis) => Err(ComponentProblem::Position { axis }),
            None => Ok(()),
        }
    }

    /// Inertia about `point`, in body axes: the rotated central inertia plus the
    /// parallel-axis term for the offset, Eq. (RB-5).
    fn inertia_about(&self, point: Vec3<Body>) -> Mat3<Body> {
        let central = self
            .orientation
            .transform(self.shape.central_inertia(self.mass_kg));
        let d = self.position_m - point;
        let shift = Mat3::identity() * d.norm_squared() - Mat3::outer(d, d);
        central + shift * self.mass_kg
    }
}

/// Why a component cannot be part of a rigid body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ComponentProblem {
    /// The mass is not positive and finite.
    Mass {
        /// The value given, in kilograms.
        value: f64,
    },
    /// A dimension is negative or not finite.
    Dimension {
        /// The value given, in meters.
        value: f64,
    },
    /// A coordinate of the position is not finite.
    Position {
        /// The axis at fault: 0, 1 or 2.
        axis: usize,
    },
}

impl fmt::Display for ComponentProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Mass { value } => {
                write!(f, "mass must be positive and finite; found {value}")
            }
            Self::Dimension { value } => {
                write!(
                    f,
                    "dimensions must be finite and not negative; found {value}"
                )
            }
            Self::Position { axis } => write!(f, "position coordinate {axis} is not finite"),
        }
    }
}

/// Why a composite body cannot be built.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CompositeError {
    /// One component is invalid.
    Component {
        /// Its position in the list, from zero.
        index: usize,
        /// What is wrong with it.
        problem: ComponentProblem,
    },
    /// The components are valid but the whole is not a rigid body: no components at all, or
    /// a degenerate arrangement such as point masses on one line, whose inertia is singular.
    Total(MassError),
}

impl fmt::Display for CompositeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Component { index, problem } => write!(f, "component {index}: {problem}"),
            Self::Total(error) => write!(f, "{error}"),
        }
    }
}

/// Combines components into the mass properties of one rigid body, Eqs. (RB-4) and (RB-5).
///
/// The inertia is accumulated about the composite center of mass from each component's offset
/// to it, not about the reference point and shifted afterward: a shift subtracts two large
/// terms when the offsets are much larger than the components.
///
/// # Errors
///
/// The first invalid component, or the reason the whole is not a rigid body.
pub fn composite(components: &[Component]) -> Result<MassProperties, CompositeError> {
    for (index, component) in components.iter().enumerate() {
        component
            .check()
            .map_err(|problem| CompositeError::Component { index, problem })?;
    }
    let mass_kg: f64 = components.iter().map(|c| c.mass_kg).sum();
    // Eq. (RB-4)
    let first_moment = components
        .iter()
        .fold(Vec3::zero(), |sum, c| sum + c.position_m * c.mass_kg);
    let cg_m = first_moment / mass_kg;
    let inertia = components
        .iter()
        .fold(Mat3::zero(), |sum, c| sum + c.inertia_about(cg_m));
    MassProperties::new(mass_kg, cg_m, inertia).map_err(CompositeError::Total)
}
