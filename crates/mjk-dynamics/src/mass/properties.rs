//! Validated mass properties of a rigid body.

use std::fmt;

use mjk_math::{Body, Mat3, Vec3};

use super::{InertiaProblem, check_inertia};

/// Why mass properties cannot describe a rigid body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MassError {
    /// The mass is not positive and finite.
    Mass {
        /// The value given, in kilograms.
        value: f64,
    },
    /// A coordinate of the center of mass is not finite.
    CenterOfMass {
        /// The axis at fault: 0, 1 or 2.
        axis: usize,
    },
    /// The tensor cannot be the inertia of a rigid body.
    Inertia(InertiaProblem),
    /// The tensor is physical, but its inverse is not representable in double precision.
    Singular,
}

impl fmt::Display for MassError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Mass { value } => write!(
                f,
                "must be a positive, finite mass in kilograms; found {value}"
            ),
            Self::CenterOfMass { .. } => write!(f, "must be finite"),
            Self::Inertia(problem) => write!(f, "{problem}"),
            Self::Singular => write!(
                f,
                "cannot be inverted in double precision: its determinant overflows or underflows"
            ),
        }
    }
}

/// Mass, center of mass and inertia tensor of a rigid body, checked to be physical.
///
/// The center of mass is measured from the airframe reference point; the inertia tensor is
/// about the center of mass, in body axes. The inverse tensor is computed once here, so the
/// equations of motion never invert a matrix during a step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    mass_kg: f64,
    cg_m: Vec3<Body>,
    inertia_kg_m2: Mat3<Body>,
    inertia_inverse: Mat3<Body>,
}

impl MassProperties {
    /// Validates and stores mass properties. The tensor is symmetrized: rounding-level
    /// asymmetry that [`check_inertia`] tolerates is averaged out.
    ///
    /// # Errors
    ///
    /// The first problem found, checking the mass, then the center of mass, then the tensor.
    pub fn new(
        mass_kg: f64,
        cg_m: Vec3<Body>,
        inertia_kg_m2: Mat3<Body>,
    ) -> Result<Self, MassError> {
        if !(mass_kg.is_finite() && mass_kg > 0.0) {
            return Err(MassError::Mass { value: mass_kg });
        }
        if let Some(axis) = cg_m.to_array().iter().position(|value| !value.is_finite()) {
            return Err(MassError::CenterOfMass { axis });
        }
        check_inertia(&inertia_kg_m2.rows()).map_err(MassError::Inertia)?;
        let symmetric = (inertia_kg_m2 + inertia_kg_m2.transpose()) * 0.5;
        let inertia_inverse = symmetric.inverse().ok_or(MassError::Singular)?;
        Ok(Self {
            mass_kg,
            cg_m,
            inertia_kg_m2: symmetric,
            inertia_inverse,
        })
    }

    /// Total mass.
    #[must_use]
    pub const fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    /// Center of mass, from the airframe reference point, in body axes.
    #[must_use]
    pub const fn cg_m(&self) -> Vec3<Body> {
        self.cg_m
    }

    /// Inertia tensor about the center of mass, in body axes.
    #[must_use]
    pub const fn inertia_kg_m2(&self) -> Mat3<Body> {
        self.inertia_kg_m2
    }

    /// Inverse of the inertia tensor.
    #[must_use]
    pub const fn inertia_inverse(&self) -> Mat3<Body> {
        self.inertia_inverse
    }

    /// A point given from the airframe reference point, measured from the center of mass
    /// instead: the lever arm of a force applied there, Eq. (RB-3).
    #[must_use]
    pub fn from_cg(&self, point_from_reference_m: Vec3<Body>) -> Vec3<Body> {
        point_from_reference_m - self.cg_m
    }
}

#[cfg(test)]
mod tests {
    use mjk_math::{Body, Mat3, Vec3};

    use super::{MassError, MassProperties};
    use crate::mass::InertiaProblem;

    fn inertia() -> Mat3<Body> {
        Mat3::from_rows([[0.02, 0.0, -0.001], [0.0, 0.03, 0.0], [-0.001, 0.0, 0.04]])
    }

    #[test]
    fn valid_properties_are_stored_with_their_inverse() {
        let cg = Vec3::new(0.01, -0.02, 0.03);
        let props = MassProperties::new(1.5, cg, inertia()).expect("valid");
        assert!((props.mass_kg() - 1.5).abs() < 1e-15);
        assert_eq!(props.cg_m(), cg);
        assert_eq!(props.inertia_kg_m2(), inertia());
        let product = props.inertia_kg_m2() * props.inertia_inverse();
        assert!((product - Mat3::identity()).max_abs() < 1e-14);
        let lever = props.from_cg(Vec3::new(0.11, 0.08, 0.03));
        assert!((lever - Vec3::new(0.1, 0.1, 0.0)).norm() < 1e-15);
    }

    #[test]
    fn rounding_level_asymmetry_is_averaged_out() {
        let mut rows = inertia().rows();
        rows[0][2] += 2e-12;
        let props = MassProperties::new(1.5, Vec3::zero(), Mat3::from_rows(rows)).expect("ok");
        let stored = props.inertia_kg_m2().rows();
        assert!((stored[0][2] - (-0.001 + 1e-12)).abs() < 1e-18);
        assert!((stored[2][0] - stored[0][2]).abs() < 1e-18);
    }

    #[test]
    fn each_problem_is_reported_in_order() {
        let good = inertia();
        let bad_cg = Vec3::new(0.0, f64::NAN, f64::INFINITY);
        for mass in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let error = MassProperties::new(mass, bad_cg, good).expect_err("bad mass");
            assert!(matches!(error, MassError::Mass { .. }), "{mass}");
        }
        let cg_error = MassProperties::new(1.0, bad_cg, good).expect_err("bad cg");
        assert_eq!(cg_error, MassError::CenterOfMass { axis: 1 });
        let flat = Mat3::from_diagonal([0.01, 0.01, 0.03]);
        let inertia_error = MassProperties::new(1.0, Vec3::zero(), flat).expect_err("triangle");
        assert!(matches!(
            inertia_error,
            MassError::Inertia(InertiaProblem::ViolatesTriangleInequality { .. })
        ));
    }

    #[test]
    fn a_tensor_whose_determinant_overflows_is_singular() {
        let huge = Mat3::from_diagonal([1e120, 1e120, 1e120]);
        let error = MassProperties::new(1.0, Vec3::zero(), huge).expect_err("overflow");
        assert_eq!(error, MassError::Singular);
    }

    #[test]
    fn errors_describe_themselves() {
        let messages = [
            (
                MassError::Mass { value: -1.5 },
                "must be a positive, finite mass in kilograms; found -1.5",
            ),
            (MassError::CenterOfMass { axis: 2 }, "must be finite"),
            (
                MassError::Singular,
                "cannot be inverted in double precision: its determinant overflows or underflows",
            ),
            (
                MassError::Inertia(InertiaProblem::NotFinite { row: 1, col: 2 }),
                "entry [1][2] is not finite",
            ),
        ];
        for (error, expected) in messages {
            assert_eq!(error.to_string(), expected);
        }
    }
}
