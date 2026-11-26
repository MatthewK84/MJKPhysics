//! Mass properties.
//!
//! Currently the physical-consistency check for inertia tensors; the mass-properties builder
//! arrives with #10 and builds on it.

use std::fmt;

/// Relative tolerance for symmetry and for the semidefiniteness test, scaled by the largest
/// tensor entry. It absorbs rounding in published or computed tensors, not modelling error.
pub const RELATIVE_TOLERANCE: f64 = 1e-9;

/// Why a tensor cannot be the inertia of a rigid body about its centre of mass.
#[derive(Clone, Copy, Debug)]
pub enum InertiaProblem {
    /// Entry `[row][col]` is NaN or infinite.
    NotFinite {
        /// Row of the entry.
        row: usize,
        /// Column of the entry.
        col: usize,
    },
    /// `J[row][col]` differs from `J[col][row]` by more than the tolerance.
    NotSymmetric {
        /// Row of the upper-triangle entry.
        row: usize,
        /// Column of the upper-triangle entry.
        col: usize,
        /// Value of `J[row][col]`.
        upper: f64,
        /// Value of `J[col][row]`.
        lower: f64,
    },
    /// The leading principal minor of the given order is not positive.
    NotPositiveDefinite {
        /// Order of the failing minor: 1, 2 or 3.
        order: usize,
        /// Its value.
        minor: f64,
    },
    /// The principal moments violate the triangle inequality: the second-moment matrix
    /// `½·tr(J)·I − J` has a negative principal minor, so no mass distribution has this tensor.
    ViolatesTriangleInequality {
        /// The most negative principal minor found.
        minor: f64,
    },
}

impl fmt::Display for InertiaProblem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFinite { row, col } => write!(f, "entry [{row}][{col}] is not finite"),
            Self::NotSymmetric {
                row,
                col,
                upper,
                lower,
            } => write!(
                f,
                "not symmetric: [{row}][{col}] = {upper} but [{col}][{row}] = {lower}"
            ),
            Self::NotPositiveDefinite { order, minor } => write!(
                f,
                "not positive definite: leading principal minor of order {order} is {minor:.3e}"
            ),
            Self::ViolatesTriangleInequality { minor } => write!(
                f,
                "principal moments violate the triangle inequality, so no rigid body has this \
                 inertia (½·tr(J)·I − J has a principal minor of {minor:.3e})"
            ),
        }
    }
}

/// Checks that `j` (kg·m², about the centre of mass) can be the inertia of a rigid body.
///
/// A symmetric tensor is the inertia of some mass distribution exactly when the second-moment
/// matrix `S = ½·tr(J)·I − J` is positive semidefinite, which is equivalent to the triangle
/// inequality on the principal moments. The tensor must also be positive definite, since a
/// zero principal moment makes the rotational dynamics singular. Both are tested with
/// principal minors, so no eigen-decomposition is needed.
///
/// # Errors
///
/// Returns the first [`InertiaProblem`] found, in the order the variants are listed.
pub fn check_inertia(j: &[[f64; 3]; 3]) -> Result<(), InertiaProblem> {
    check_finite(j)?;
    let scale = j
        .iter()
        .flatten()
        .fold(0.0_f64, |acc, value| acc.max(value.abs()));
    let sym = symmetrized(j, scale)?;
    check_positive_definite(&sym)?;
    check_realizable(&sym, scale)
}

fn check_finite(j: &[[f64; 3]; 3]) -> Result<(), InertiaProblem> {
    for (row, values) in j.iter().enumerate() {
        if let Some(col) = values.iter().position(|value| !value.is_finite()) {
            return Err(InertiaProblem::NotFinite { row, col });
        }
    }
    Ok(())
}

/// Checks symmetry and returns `[xx, yy, zz, xy, xz, yz]` with averaged off-diagonals.
fn symmetrized(j: &[[f64; 3]; 3], scale: f64) -> Result<[f64; 6], InertiaProblem> {
    let [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]] = *j;
    let pairs = [(0, 1, xy, yx), (0, 2, xz, zx), (1, 2, yz, zy)];
    for (row, col, upper, lower) in pairs {
        if (upper - lower).abs() > RELATIVE_TOLERANCE * scale {
            return Err(InertiaProblem::NotSymmetric {
                row,
                col,
                upper,
                lower,
            });
        }
    }
    Ok([
        xx,
        yy,
        zz,
        0.5 * (xy + yx),
        0.5 * (xz + zx),
        0.5 * (yz + zy),
    ])
}

/// Determinant of the symmetric matrix given as `[xx, yy, zz, xy, xz, yz]`.
fn symmetric_det([xx, yy, zz, xy, xz, yz]: [f64; 6]) -> f64 {
    xx * (yy * zz - yz * yz) - xy * (xy * zz - yz * xz) + xz * (xy * yz - yy * xz)
}

fn check_positive_definite(sym: &[f64; 6]) -> Result<(), InertiaProblem> {
    let [xx, yy, _, xy, _, _] = *sym;
    let minors = [xx, xx * yy - xy * xy, symmetric_det(*sym)];
    for (order, minor) in (1..=3).zip(minors) {
        if minor <= 0.0 {
            return Err(InertiaProblem::NotPositiveDefinite { order, minor });
        }
    }
    Ok(())
}

fn check_realizable(sym: &[f64; 6], scale: f64) -> Result<(), InertiaProblem> {
    let [xx, yy, zz, xy, xz, yz] = *sym;
    let half_trace = 0.5 * (xx + yy + zz);
    let s = [
        half_trace - xx,
        half_trace - yy,
        half_trace - zz,
        -xy,
        -xz,
        -yz,
    ];
    let [sxx, syy, szz, sxy, sxz, syz] = s;
    let tol = RELATIVE_TOLERANCE;
    let minors = [
        (sxx, tol * scale),
        (syy, tol * scale),
        (szz, tol * scale),
        (sxx * syy - sxy * sxy, tol * scale * scale),
        (sxx * szz - sxz * sxz, tol * scale * scale),
        (syy * szz - syz * syz, tol * scale * scale),
        (symmetric_det(s), tol * scale * scale * scale),
    ];
    let worst = minors
        .iter()
        .filter(|(minor, allowance)| *minor < -allowance)
        .map(|(minor, _)| *minor)
        .fold(f64::INFINITY, f64::min);
    if worst.is_finite() {
        return Err(InertiaProblem::ViolatesTriangleInequality { minor: worst });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{InertiaProblem, check_inertia};

    fn diag(a: f64, b: f64, c: f64) -> [[f64; 3]; 3] {
        [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]]
    }

    /// `R · diag · Rᵀ` for a rotation of 0.6 rad about the axis (1, 2, 2)/3.
    fn rotated(principal: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
        let (sin, cos) = 0.6_f64.sin_cos();
        let axis = [1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0];
        let skew = [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ];
        let mut rot = [[0.0; 3]; 3];
        let mut out = [[0.0; 3]; 3];
        for row in 0..3 {
            for col in 0..3 {
                let eye = if row == col { 1.0 } else { 0.0 };
                rot[row][col] =
                    cos * eye + sin * skew[row][col] + (1.0 - cos) * axis[row] * axis[col];
            }
        }
        for row in 0..3 {
            for col in 0..3 {
                out[row][col] = (0..3)
                    .map(|k| rot[row][k] * principal[k][k] * rot[col][k])
                    .sum();
            }
        }
        out
    }

    #[test]
    fn solid_box_is_valid() {
        // 1.5 kg box, 0.4 x 0.4 x 0.1 m: I = m(b² + c²)/12 about each axis.
        let m = 1.5;
        let inertia = diag(
            m * (0.16 + 0.01) / 12.0,
            m * (0.16 + 0.01) / 12.0,
            m * (0.16 + 0.16) / 12.0,
        );
        assert!(check_inertia(&inertia).is_ok());
    }

    #[test]
    fn rotated_valid_tensor_with_products_of_inertia_is_valid() {
        assert!(check_inertia(&rotated(diag(0.02, 0.03, 0.04))).is_ok());
    }

    #[test]
    fn flat_plate_on_the_triangle_boundary_is_valid() {
        // A lamina in the x-y plane has Izz = Ixx + Iyy exactly.
        assert!(check_inertia(&diag(0.01, 0.02, 0.03)).is_ok());
    }

    #[test]
    fn triangle_violation_is_rejected_even_when_rotated() {
        for tensor in [diag(0.01, 0.01, 0.03), rotated(diag(0.01, 0.01, 0.03))] {
            assert!(matches!(
                check_inertia(&tensor),
                Err(InertiaProblem::ViolatesTriangleInequality { .. })
            ));
        }
    }

    #[test]
    fn indefinite_and_singular_tensors_are_rejected() {
        let indefinite = [[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        assert!(matches!(
            check_inertia(&indefinite),
            Err(InertiaProblem::NotPositiveDefinite { order: 2, .. })
        ));
        assert!(matches!(
            check_inertia(&diag(1.0, 1.0, 0.0)),
            Err(InertiaProblem::NotPositiveDefinite { order: 3, .. })
        ));
        assert!(matches!(
            check_inertia(&diag(0.0, 0.0, 0.0)),
            Err(InertiaProblem::NotPositiveDefinite { order: 1, .. })
        ));
    }

    #[test]
    fn asymmetry_and_non_finite_entries_are_located() {
        let mut asymmetric = diag(0.02, 0.03, 0.04);
        asymmetric[1][2] = 0.001;
        assert!(matches!(
            check_inertia(&asymmetric),
            Err(InertiaProblem::NotSymmetric { row: 1, col: 2, .. })
        ));
        let mut nan = diag(0.02, 0.03, 0.04);
        nan[2][0] = f64::NAN;
        assert!(matches!(
            check_inertia(&nan),
            Err(InertiaProblem::NotFinite { row: 2, col: 0 })
        ));
    }

    #[test]
    fn rounding_level_asymmetry_is_tolerated() {
        let mut nearly = rotated(diag(0.02, 0.03, 0.04));
        nearly[0][1] += 1e-15;
        assert!(check_inertia(&nearly).is_ok());
    }
}
