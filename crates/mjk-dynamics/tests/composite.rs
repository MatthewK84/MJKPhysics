//! Composite mass properties (#10): a hand-computed body, the shape formulas, invalid input,
//! and identities that hold for any arrangement of components.

use std::f64::consts::FRAC_PI_2;

use mjk_dynamics::mass::InertiaProblem;
use mjk_dynamics::mass::{
    Component, ComponentProblem, CompositeError, MassError, MassProperties, Shape, composite,
};
use mjk_math::{Body, Mat3, Part, Quat, Rotation, UnitQuat, Vec3};
use proptest::prelude::*;

fn v(x: f64, y: f64, z: f64) -> Vec3<Body> {
    Vec3::new(x, y, z)
}

fn built(components: &[Component]) -> Option<MassProperties> {
    composite(components).ok()
}

fn relative_error(a: Mat3<Body>, b: Mat3<Body>) -> f64 {
    (a - b).max_abs() / b.max_abs()
}

#[test]
fn two_component_body_matches_the_hand_computed_tensor() {
    // A 1.2 kg box, 0.2 × 0.1 × 0.05 m, centered at (0.05, 0, 0), and a 0.3 kg point mass at
    // (−0.1, 0.04, 0.02).
    // Mass 1.5 kg; center of mass (1.2·0.05 − 0.3·0.1, 0.3·0.04, 0.3·0.02)/1.5 = (0.02, 0.008, 0.004).
    // Offsets from it: d₁ = (0.03, −0.008, −0.004), d₂ = (−0.12, 0.032, 0.016).
    // Box about its center: (1.2/12)·(0.01 + 0.0025, 0.04 + 0.0025, 0.04 + 0.01)
    //   = diag(0.00125, 0.00425, 0.005).
    // Parallel-axis terms m(|d|²·I − d·dᵀ), with |d₁|² = 0.00098 and |d₂|² = 0.01568:
    //   xx: 1.2·0.00008 + 0.3·0.00128 = 0.000096 + 0.000384
    //   yy: 1.2·0.000916 + 0.3·0.014656 = 0.0010992 + 0.0043968
    //   zz: 1.2·0.000964 + 0.3·0.015424 = 0.0011568 + 0.0046272
    //   xy: −(1.2·0.03·(−0.008) + 0.3·(−0.12)·0.032) = 0.000288 + 0.001152
    //   xz: −(1.2·0.03·(−0.004) + 0.3·(−0.12)·0.016) = 0.000144 + 0.000576
    //   yz: −(1.2·(−0.008)·(−0.004) + 0.3·0.032·0.016) = −0.0000384 − 0.0001536
    let props = built(&[
        Component::new(
            1.2,
            Shape::Box {
                size_m: [0.2, 0.1, 0.05],
            },
            v(0.05, 0.0, 0.0),
        ),
        Component::new(0.3, Shape::PointMass, v(-0.1, 0.04, 0.02)),
    ])
    .expect("physical");
    let expected = Mat3::from_rows([
        [0.00173, 0.00144, 0.00072],
        [0.00144, 0.009_746, -0.000_192],
        [0.00072, -0.000_192, 0.010_784],
    ]);
    assert!((props.mass_kg() - 1.5).abs() < 1e-15);
    assert!((props.cg_m() - v(0.02, 0.008, 0.004)).norm() < 1e-17);
    assert!(relative_error(props.inertia_kg_m2(), expected) < 1e-14);
}

#[test]
fn box_and_cylinder_match_their_formulas() {
    // Box: m/12·(b² + c², a² + c², a² + b²). Cylinder along z: m(3r² + L²)/12 across, m·r²/2 along.
    let a_box = built(&[Component::new(
        3.0,
        Shape::Box {
            size_m: [0.4, 0.2, 0.1],
        },
        Vec3::zero(),
    )])
    .expect("physical");
    let expected_box = Mat3::from_diagonal([0.0125, 0.0425, 0.05]);
    assert!(relative_error(a_box.inertia_kg_m2(), expected_box) < 1e-15);
    let cylinder = built(&[Component::new(
        2.0,
        Shape::Cylinder {
            radius_m: 0.1,
            length_m: 0.3,
        },
        v(1.0, 2.0, 3.0),
    )])
    .expect("physical");
    let expected_cylinder = Mat3::from_diagonal([0.02, 0.02, 0.01]);
    assert!(relative_error(cylinder.inertia_kg_m2(), expected_cylinder) < 1e-15);
    assert!((cylinder.cg_m() - v(1.0, 2.0, 3.0)).norm() < 1e-15);
}

#[test]
fn a_quarter_turn_about_z_swaps_the_x_and_y_moments() {
    let quarter = Rotation::<Body, Part>::from_unit_quat(UnitQuat::exp([0.0, 0.0, FRAC_PI_2]));
    let arm = Component::new(
        1.0,
        Shape::Box {
            size_m: [0.6, 0.06, 0.03],
        },
        Vec3::zero(),
    );
    let along_x = built(&[arm]).expect("physical").inertia_kg_m2().diagonal();
    let along_y = built(&[arm.rotated(quarter)])
        .expect("physical")
        .inertia_kg_m2()
        .diagonal();
    assert!((along_y[0] - along_x[1]).abs() < 1e-16);
    assert!((along_y[1] - along_x[0]).abs() < 1e-16);
    assert!((along_y[2] - along_x[2]).abs() < 1e-16);
}

fn good_component() -> Component {
    Component::new(
        1.0,
        Shape::Box {
            size_m: [0.1, 0.1, 0.1],
        },
        Vec3::zero(),
    )
}

/// Components each spoiled in one way, with the problem that must be reported.
fn spoiled(good: Component) -> [(Component, ComponentProblem); 5] {
    [
        (
            Component {
                mass_kg: -1.0,
                ..good
            },
            ComponentProblem::Mass { value: -1.0 },
        ),
        (
            Component {
                mass_kg: 0.0,
                ..good
            },
            ComponentProblem::Mass { value: 0.0 },
        ),
        (
            Component {
                shape: Shape::Box {
                    size_m: [0.1, -0.2, 0.1],
                },
                ..good
            },
            ComponentProblem::Dimension { value: -0.2 },
        ),
        (
            Component {
                shape: Shape::Cylinder {
                    radius_m: 0.1,
                    length_m: f64::INFINITY,
                },
                ..good
            },
            ComponentProblem::Dimension {
                value: f64::INFINITY,
            },
        ),
        (
            Component {
                position_m: v(0.0, 0.0, f64::NAN),
                ..good
            },
            ComponentProblem::Position { axis: 2 },
        ),
    ]
}

#[test]
fn invalid_components_are_located() {
    let good = good_component();
    for (bad, problem) in spoiled(good) {
        let error = composite(&[good, bad]).expect_err("invalid component");
        assert_eq!(error, CompositeError::Component { index: 1, problem });
    }
}

#[test]
fn degenerate_wholes_are_rejected() {
    let empty = composite(&[]).expect_err("no components");
    assert_eq!(empty, CompositeError::Total(MassError::Mass { value: 0.0 }));
    // Point masses on the x axis have no moment of inertia about it.
    let on_a_line = composite(&[
        Component::new(1.0, Shape::PointMass, v(-0.1, 0.0, 0.0)),
        Component::new(1.0, Shape::PointMass, v(0.2, 0.0, 0.0)),
    ])
    .expect_err("singular");
    assert!(matches!(
        on_a_line,
        CompositeError::Total(MassError::Inertia(InertiaProblem::NotPositiveDefinite {
            order: 1,
            ..
        }))
    ));
    // A flat plate (zero thickness) is a valid rigid body.
    let plate = Component::new(
        1.0,
        Shape::Box {
            size_m: [0.3, 0.2, 0.0],
        },
        Vec3::zero(),
    );
    assert!(composite(&[plate]).is_ok());
}

#[test]
fn errors_describe_themselves() {
    let component = CompositeError::Component {
        index: 3,
        problem: ComponentProblem::Dimension { value: -0.2 },
    };
    assert_eq!(
        component.to_string(),
        "component 3: dimensions must be finite and not negative; found -0.2"
    );
    let mass = ComponentProblem::Mass { value: 0.0 };
    assert_eq!(
        mass.to_string(),
        "mass must be positive and finite; found 0"
    );
    let position = ComponentProblem::Position { axis: 1 };
    assert_eq!(position.to_string(), "position coordinate 1 is not finite");
    let total = CompositeError::Total(MassError::Mass { value: 0.0 });
    assert_eq!(
        total.to_string(),
        "must be a positive, finite mass in kilograms; found 0"
    );
}

// ---------------------------------------------------------------- identities

fn orientation() -> impl Strategy<Value = Rotation<Body, Part>> {
    (-1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64, -1.0..1.0_f64)
        .prop_filter_map("normalizable", |(w, x, y, z)| {
            let q = Quat::new(w, x, y, z);
            (q.norm() > 0.1).then(|| UnitQuat::new(q)).flatten()
        })
        .prop_map(Rotation::from_unit_quat)
}

fn position() -> impl Strategy<Value = Vec3<Body>> {
    (-0.5..0.5_f64, -0.5..0.5_f64, -0.5..0.5_f64).prop_map(|(x, y, z)| v(x, y, z))
}

fn component() -> impl Strategy<Value = Component> {
    let size = (0.01..0.4_f64, 0.01..0.4_f64, 0.01..0.4_f64);
    let shape = prop_oneof![
        size.prop_map(|(a, b, c)| Shape::Box { size_m: [a, b, c] }),
        (0.01..0.2_f64, 0.01..0.4_f64)
            .prop_map(|(radius_m, length_m)| Shape::Cylinder { radius_m, length_m }),
    ];
    (0.05..2.0_f64, shape, position(), orientation())
        .prop_map(|(mass, shape, at, turn)| Component::new(mass, shape, at).rotated(turn))
}

/// The two halves of a box or cylinder, cut across its part x axis (box) or z axis (cylinder).
fn halves(whole: Component) -> [Component; 2] {
    let (shape, offset_part) = match whole.shape {
        Shape::Box { size_m: [a, b, c] } => (
            Shape::Box {
                size_m: [0.5 * a, b, c],
            },
            [0.25 * a, 0.0, 0.0],
        ),
        Shape::Cylinder { radius_m, length_m } => (
            Shape::Cylinder {
                radius_m,
                length_m: 0.5 * length_m,
            },
            [0.0, 0.0, 0.25 * length_m],
        ),
        Shape::PointMass => (Shape::PointMass, [0.0; 3]),
    };
    let offset = whole.orientation.rotate(Vec3::from_array(offset_part));
    let half = Component {
        mass_kg: 0.5 * whole.mass_kg,
        shape,
        ..whole
    };
    [
        Component {
            position_m: whole.position_m + offset,
            ..half
        },
        Component {
            position_m: whole.position_m - offset,
            ..half
        },
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    #[test]
    fn cutting_a_component_in_half_changes_nothing(whole in component()) {
        let one = built(&[whole]).ok_or_else(|| TestCaseError::fail("not physical"))?;
        let two = built(&halves(whole)).ok_or_else(|| TestCaseError::fail("not physical"))?;
        prop_assert!((one.mass_kg() - two.mass_kg()).abs() < 1e-15);
        prop_assert!((one.cg_m() - two.cg_m()).norm() < 1e-15);
        prop_assert!(relative_error(two.inertia_kg_m2(), one.inertia_kg_m2()) < 1e-12);
    }

    #[test]
    fn turning_every_component_turns_the_whole(
        parts in prop::collection::vec(component(), 1..6),
        turn in orientation(),
    ) {
        let q = Rotation::<Body, Body>::from_unit_quat(turn.unit_quat());
        let turned: Vec<Component> = parts
            .iter()
            .map(|c| Component { position_m: q.rotate(c.position_m), orientation: q * c.orientation, ..*c })
            .collect();
        let before = built(&parts).ok_or_else(|| TestCaseError::fail("not physical"))?;
        let after = built(&turned).ok_or_else(|| TestCaseError::fail("not physical"))?;
        prop_assert!((after.cg_m() - q.rotate(before.cg_m())).norm() < 1e-14);
        let expected = q.transform(before.inertia_kg_m2());
        prop_assert!(relative_error(after.inertia_kg_m2(), expected) < 1e-12);
    }

    #[test]
    fn order_of_components_does_not_matter(parts in prop::collection::vec(component(), 2..6)) {
        let reversed: Vec<Component> = parts.iter().rev().copied().collect();
        let forward = built(&parts).ok_or_else(|| TestCaseError::fail("not physical"))?;
        let backward = built(&reversed).ok_or_else(|| TestCaseError::fail("not physical"))?;
        prop_assert!((forward.cg_m() - backward.cg_m()).norm() < 1e-15);
        prop_assert!(relative_error(backward.inertia_kg_m2(), forward.inertia_kg_m2()) < 1e-13);
    }
}
