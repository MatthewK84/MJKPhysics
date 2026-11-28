//! The airframe format (#18): units convert exactly and are checked against each field's
//! quantity; every part of an airframe loads into SI values or is refused with the field at
//! fault; and the generated page lists every parameter with its source.

use mjk_core::EngineError;
use mjk_math::Vec3;
use mjk_sim::config::units::{Dimension, UnitError, parse_unit, quantities};
use mjk_sim::config::{
    Braking, LoadedAirframe, Spin, airframe_page, load_airframe, plain, significant,
    thrust_to_weight,
};

const VALID: &str = include_str!("fixtures/airframe/valid.toml");
const EXAMPLE: &str = include_str!("../../../airframes/example-quadrotor.toml");
const EXAMPLE_PAGE: &str = include_str!("../../../docs/src/airframes/example-quadrotor.md");

/// The valid fixture with each `(from, to)` replacement made, or `None` if a `from` is missing.
fn edited(changes: &[(&str, &str)]) -> Option<String> {
    changes
        .iter()
        .try_fold(VALID.to_owned(), |text, (from, to)| {
            text.contains(from).then(|| text.replacen(from, to, 1))
        })
}

/// The valid fixture with `extra` appended.
fn extended(extra: &str) -> String {
    format!("{VALID}{extra}")
}

fn load(text: &str) -> Result<LoadedAirframe, EngineError> {
    load_airframe("test.toml", text)
}

/// The field and reason a document is refused with, if it is.
fn refusal(text: Option<String>) -> Option<(String, String)> {
    match load(&text?) {
        Err(EngineError::Config { path, reason, .. }) => Some((path.to_string(), reason)),
        _ => None,
    }
}

fn refused(path: &str, reason: &str) -> (String, String) {
    (path.to_owned(), reason.to_owned())
}

// ---------------------------------------------------------------- units

#[test]
fn units_convert_by_their_definitions() {
    let cases = [
        ("kg", 1.0),
        ("g", 1e-3),
        ("in", 0.0254),
        ("ft", 0.3048),
        ("lb", 0.453_592_37),
        ("oz", 0.028_349_523_125),
        ("gf", 9.806_65e-3),
        ("kgf", 9.806_65),
        ("lbf", 4.448_221_615_260_5),
        ("mAh", 3.6),
        ("Ah", 3600.0),
        ("min", 60.0),
        ("h", 3600.0),
        ("us", 1e-6),
        ("µs", 1e-6),
        ("mohm", 1e-3),
        ("Ω", 1.0),
        ("g cm^2", 1e-7),
        ("1/s", 1.0),
        ("mm^-1", 1e3),
        ("rev", 2.0 * core::f64::consts::PI),
        ("deg", core::f64::consts::PI / 180.0),
        ("rpm/V", 2.0 * core::f64::consts::PI / 60.0),
    ];
    for (text, factor) in cases {
        let unit = parse_unit(text).expect(text);
        assert!(
            (unit.factor / factor - 1.0).abs() <= 2.0 * f64::EPSILON,
            "{text}: {}",
            unit.factor
        );
    }
}

#[test]
fn spellings_of_one_unit_agree() {
    for text in ["kg·m²", "kg*m^2", "kg  m^2", "m^2 kg", "kg m m"] {
        assert_eq!(parse_unit(text), parse_unit("kg m^2"), "{text}");
    }
    assert_eq!(parse_unit("m³"), parse_unit("m^3"));
    assert_eq!(
        parse_unit("rad/s/V"),
        parse_unit("rad/(s V)").or(parse_unit("rad/s/V"))
    );
    let torque_per_rate = parse_unit("N m s/rad").expect("parses");
    assert_eq!(torque_per_rate.dimension, Dimension([2, 1, -1, 0, -1]));
}

#[test]
fn quantities_have_the_dimensions_of_their_units() {
    let cases = [
        (quantities::MASS_KG, "lb"),
        (quantities::LENGTH_M, "ft"),
        (quantities::AREA_M2, "cm^2"),
        (quantities::INERTIA_KG_M2, "g cm^2"),
        (quantities::FORCE_N, "lbf"),
        (quantities::TIME_S, "ms"),
        (quantities::RATE_PER_S, "Hz"),
        (quantities::ANGLE_RAD, "deg"),
        (quantities::CURRENT_A, "mA"),
        (quantities::RESISTANCE_OHM, "mohm"),
        (quantities::CHARGE_C, "mAh"),
        (quantities::SPEED_CONSTANT, "rpm/V"),
        (quantities::PURE, "1"),
    ];
    for (quantity, unit) in cases {
        assert_eq!(
            parse_unit(unit).map(|u| u.dimension),
            Ok(quantity.dimension),
            "{unit}"
        );
    }
    assert_ne!(
        parse_unit("rpm").map(|u| u.dimension),
        parse_unit("Hz").map(|u| u.dimension),
        "an rpm is not a Hz"
    );
}

#[test]
fn malformed_units_are_named() {
    let cases = [
        ("stone", UnitError::Unknown("stone".to_owned())),
        ("kg/", UnitError::Empty),
        ("", UnitError::Empty),
        ("m^0", UnitError::Power("m^0".to_owned())),
        ("m^x", UnitError::Power("m^x".to_owned())),
        ("m^10", UnitError::Power("m^10".to_owned())),
        ("Kg", UnitError::Unknown("Kg".to_owned())),
    ];
    for (text, error) in cases {
        assert_eq!(parse_unit(text), Err(error), "{text}");
    }
    assert_eq!(
        UnitError::Power("m^x".to_owned()).to_string(),
        "`m^x`: a power is a nonzero integer, as in m^2 or s^-1"
    );
    assert_eq!(
        UnitError::Empty.to_string(),
        "an empty unit; write 1 for a pure number"
    );
}

#[test]
fn dimensions_display_in_base_units() {
    assert_eq!(Dimension([1, 1, -2, 0, 0]).to_string(), "kg·m·s⁻²");
    assert_eq!(Dimension([0, 0, -1, 0, 0]).to_string(), "s⁻¹");
    assert_eq!(Dimension([2, 1, -1, 0, -1]).to_string(), "kg·m²·s⁻¹·rad⁻¹");
    assert_eq!(Dimension([0, 0, 0, 10, 0]).to_string(), "A¹⁰");
    assert_eq!(Dimension([4, 5, 6, 7, 8]).to_string(), "kg⁵·m⁴·s⁶·A⁷·rad⁸");
    assert_eq!(Dimension([0, 0, -9, 0, 3]).to_string(), "s⁻⁹·rad³");
    assert_eq!(Dimension::NONE.to_string(), "1");
}

#[test]
fn numbers_are_shown_as_written_and_to_six_figures() {
    assert_eq!(plain(920.0), "920");
    assert_eq!(plain(0.117), "0.117");
    let cases = [
        (0.0, "0"),
        (1.108, "1.108"),
        (96.342_174_710_087_39, "96.3422"),
        (-0.035, "-0.035"),
        (0.000_123_456_7, "0.000123457"),
        (0.000_012_345, "1.2345e-5"),
        (123_456.7, "123457"),
        (1_234_567.0, "1.23457e6"),
        (10_800.0, "10800"),
        (f64::NAN, "NaN"),
    ];
    for (value, text) in cases {
        assert_eq!(significant(value), text, "{value}");
    }
}

// ---------------------------------------------------------------- parameters

/// Checks that `text` is refused at `path` with a reason containing `fragment`.
fn assert_refused(text: Option<String>, path: &str, fragment: &str) {
    let found = refusal(text);
    assert_eq!(
        found.as_ref().map(|(at, _)| at.as_str()),
        Some(path),
        "{found:?}"
    );
    let reason = found.map(|(_, reason)| reason).unwrap_or_default();
    assert!(reason.contains(fragment), "{reason:?} lacks {fragment:?}");
}

#[test]
fn parameters_need_a_unit_of_the_right_quantity() {
    assert_refused(
        edited(&[(
            "cg = { value = [0.0, 0.0, 0.01], unit = \"m\", source = \"sheet\" }",
            "cg = [0.0, 0.0, 0.01]",
        )]),
        "mass.cg",
        mjk_sim::config::MISSING_UNIT,
    );
    assert_refused(
        edited(&[("unit = \"kg m^2\"", "unit = \"kg m\"")]),
        "mass.inertia.unit",
        "`kg m` is not a moment of inertia; use a unit such as kg m^2 or g cm^2",
    );
    assert_refused(
        edited(&[("unit = \"kg m^2\"", "unit = \"kg m^\"")]),
        "mass.inertia.unit",
        "`m^`: a power is a nonzero integer, as in m^2 or s^-1",
    );
    assert_refused(
        edited(&[(
            "value = 1.5, unit = \"kg\", source = \"sheet\"",
            "value = 1.5, unit = \"kg\", source = \"sheet\", note = \"x\"",
        )]),
        "mass.mass.note",
        "unknown field `note`, expected one of `value`, `unit`, `source`, `uncertainty`",
    );
}

#[test]
fn uncertainties_must_be_finite_and_not_negative() {
    assert_refused(
        edited(&[(
            "source = \"sheet\" }\ncg",
            "source = \"sheet\", uncertainty = -0.1 }\ncg",
        )]),
        "mass.mass.uncertainty",
        "an uncertainty must be finite and not negative; found -0.1",
    );
    assert_refused(
        edited(&[(
            "[0.0, 0.0, 0.01], unit = \"m\", source = \"sheet\"",
            "[0.0, 0.0, 0.01], unit = \"m\", source = \"sheet\", uncertainty = [0.0, inf, 0.0]",
        )]),
        "mass.cg.uncertainty[1]",
        "an uncertainty must be finite and not negative; found inf",
    );
}

#[test]
fn uncertainties_are_recorded_in_the_unit_of_the_value() {
    let text = edited(&[(
        "max_thrust = { value = 2, unit = \"kgf\", source = \"sheet\" }",
        "max_thrust = { value = 2, unit = \"kgf\", source = \"sheet\", uncertainty = 0.1 }",
    )])
    .expect("edits");
    let loaded = load(&text).expect("loads");
    let record = loaded
        .parameters
        .iter()
        .find(|record| record.path.to_string() == "propellers.p10.max_thrust")
        .expect("recorded");
    assert_eq!(record.uncertainty.as_deref(), Some("0.1"));
    assert_eq!(record.si, "19.6133 N");
    assert_eq!(record.value, "2");
    assert_eq!(record.unit, "kgf");
    assert_eq!(record.source.as_deref(), Some("sheet"));
}

#[test]
fn counts_have_unit_1_and_are_at_least_one() {
    assert_eq!(
        refusal(edited(&[(
            "blades = { value = 2, unit = \"1\"",
            "blades = { value = 2, unit = \"mm\""
        )])),
        Some(refused(
            "propellers.p10.blades.unit",
            "a count has unit `1`; found `mm`"
        ))
    );
    assert_eq!(
        refusal(edited(&[(
            "blades = { value = 2,",
            "blades = { value = 0,"
        )])),
        Some(refused("propellers.p10.blades.value", "must be at least 1"))
    );
    assert_eq!(
        refusal(edited(&[(
            "blades = { value = 2, unit = \"1\", source = \"sheet\"",
            "blades = { value = 2, unit = \"1\", source = \"web\""
        )])),
        Some(refused(
            "propellers.p10.blades.source",
            "no entry `web` in [sources]"
        ))
    );
}

// ---------------------------------------------------------------- sources

#[test]
fn sources_need_a_title_and_unused_ones_are_reported() {
    assert_eq!(
        refusal(edited(&[(
            "title = \"Example data sheet\"",
            "title = \" \""
        )])),
        Some(refused("sources.sheet.title", "must not be empty"))
    );
    let text = edited(&[(
        "[sources.sheet]",
        "[sources.unused]\ntitle = \"Never cited\"\nyear = 2015\n\n[sources.sheet]",
    )])
    .expect("edits");
    let loaded = load(&text).expect("loads");
    let warnings: Vec<String> = loaded.warnings.iter().map(ToString::to_string).collect();
    assert_eq!(warnings, ["sources.unused: no parameter cites this source"]);
}

#[test]
fn a_citation_lists_what_is_known() {
    let text = edited(&[(
        "title = \"Example data sheet\"",
        "title = \"Motor datasheet\"\nauthor = \"Example Motors\"\nyear = 2024\ndoi = \"10.0000/example\"\nurl = \"https://example.com/datasheet\"\nnote = \"Table 2.\"",
    )])
    .expect("edits");
    let loaded = load(&text).expect("loads");
    assert_eq!(
        loaded.config.sources["sheet"].citation(),
        "Motor datasheet, Example Motors, 2024, doi:10.0000/example, \
         <https://example.com/datasheet>. Table 2."
    );
    let bare = load(VALID).expect("loads");
    assert_eq!(
        bare.config.sources["sheet"].citation(),
        "Example data sheet."
    );
}

// ---------------------------------------------------------------- mass

const PARTS: &str = "[[mass.components]]
name = \"body\"
mass = { value = 1, unit = \"kg\", source = \"sheet\" }
position = { value = [0, 0, 0], unit = \"m\", source = \"sheet\" }
shape = \"box\"
size = { value = [0.2, 0.1, 0.1], unit = \"m\", source = \"sheet\" }

[[mass.components]]
name = \"battery\"
mass = { value = 0.5, unit = \"kg\", source = \"sheet\" }
position = { value = [0, 0, 0.06], unit = \"m\", source = \"sheet\" }
shape = \"point\"
";

/// The valid fixture with its direct mass table replaced by `parts`.
fn with_parts(parts: &str) -> Option<String> {
    let start = VALID.find("[mass]")?;
    let end = VALID.find("[[rotors]]")?;
    Some(format!("{}{parts}\n{}", &VALID[..start], &VALID[end..]))
}

#[test]
fn mass_can_be_built_up_from_parts() {
    let loaded = load(&with_parts(PARTS).expect("edits")).expect("loads");
    let mass = &loaded.airframe.mass;
    assert!((mass.mass_kg() - 1.5).abs() < 1e-15);
    assert!((mass.cg_m().z - 0.02).abs() < 1e-15);
    // Box about its center, plus each part's offset from the combined center of mass.
    let [ixx, iyy, izz] = mass.inertia_kg_m2().diagonal();
    let offsets = 1.0 * 0.02 * 0.02 + 0.5 * 0.04 * 0.04;
    assert!((ixx - ((0.01 + 0.01) / 12.0 + offsets)).abs() < 1e-15);
    assert!((iyy - ((0.04 + 0.01) / 12.0 + offsets)).abs() < 1e-15);
    assert!((izz - (0.04 + 0.01) / 12.0).abs() < 1e-15);
}

#[test]
fn a_part_turned_by_its_orientation_swaps_its_inertia() {
    let turned = PARTS.replace(
        "size = { value = [0.2, 0.1, 0.1], unit = \"m\", source = \"sheet\" }",
        "size = { value = [0.2, 0.1, 0.1], unit = \"m\", source = \"sheet\" }\norientation = { value = [0, 0, 90], unit = \"deg\", source = \"sheet\" }",
    );
    let straight = load(&with_parts(PARTS).expect("edits")).expect("loads");
    let rotated = load(&with_parts(&turned).expect("edits")).expect("loads");
    let [ixx, iyy, _] = straight.airframe.mass.inertia_kg_m2().diagonal();
    let [jxx, jyy, _] = rotated.airframe.mass.inertia_kg_m2().diagonal();
    let offsets = 1.0 * 0.02 * 0.02 + 0.5 * 0.04 * 0.04;
    assert!(((jxx - offsets) - (iyy - offsets)).abs() < 1e-15);
    assert!(((jyy - offsets) - (ixx - offsets)).abs() < 1e-15);
}

#[test]
fn parts_need_the_fields_of_their_shape_and_no_others() {
    let cases = [
        (
            PARTS.replace(
                "size = { value = [0.2, 0.1, 0.1], unit = \"m\", source = \"sheet\" }\n",
                "",
            ),
            "mass.components[0].size",
            "a box needs a size",
        ),
        (
            PARTS.replace(
                "shape = \"point\"",
                "shape = \"point\"\nradius = { value = 1, unit = \"m\", source = \"sheet\" }",
            ),
            "mass.components[1].radius",
            "a point has no radius",
        ),
        (
            PARTS.replace(
                "shape = \"point\"",
                "shape = \"cylinder\"\nradius = { value = 1, unit = \"m\", source = \"sheet\" }",
            ),
            "mass.components[1].length",
            "a cylinder needs a length",
        ),
        (
            PARTS.replace("name = \"body\"", "name = \"\""),
            "mass.components[0].name",
            "must not be empty",
        ),
        (
            PARTS.replace(
                "size = { value = [0.2, 0.1, 0.1]",
                "size = { value = [0.2, -0.1, 0.1]",
            ),
            "mass.components[0]",
            "dimensions must be finite and not negative; found -0.1",
        ),
        (
            PARTS.replace("mass = { value = 0.5,", "mass = { value = -0.5,"),
            "mass.components[1].mass.value",
            "mass must be positive and finite; found -0.5",
        ),
    ];
    for (parts, path, reason) in cases {
        let (found_path, found_reason) = refusal(with_parts(&parts)).expect("refused");
        assert_eq!(found_path, path);
        assert!(found_reason.contains(reason), "{found_reason}");
    }
}

#[test]
fn a_cylinder_part_loads() {
    let parts = PARTS.replace(
        "shape = \"point\"",
        "shape = \"cylinder\"\nradius = { value = 20, unit = \"mm\", source = \"sheet\" }\nlength = { value = 0.1, unit = \"m\", source = \"sheet\" }",
    );
    let loaded = load(&with_parts(&parts).expect("edits")).expect("loads");
    assert!((loaded.airframe.mass.mass_kg() - 1.5).abs() < 1e-15);
}

#[test]
fn mass_is_given_one_way_or_the_other() {
    let both = with_parts(PARTS).and_then(|text| {
        let at = text.find("[[mass.components]]")?;
        Some(format!(
            "{}[mass]\nmass = {{ value = 1.5, unit = \"kg\", source = \"sheet\" }}\n\n{}",
            &text[..at],
            &text[at..]
        ))
    });
    assert_eq!(
        refusal(both),
        Some(refused(
            "mass",
            "give either mass, cg and inertia, or components, not both"
        ))
    );
    let neither = with_parts("[mass]\n");
    assert_eq!(
        refusal(neither),
        Some(refused(
            "mass",
            "missing `mass`: give mass, cg and inertia, or [[mass.components]]"
        ))
    );
    let no_cg = edited(&[(
        "cg = { value = [0.0, 0.0, 0.01], unit = \"m\", source = \"sheet\" }\n",
        "",
    )]);
    assert_eq!(
        refusal(no_cg).map(|r| r.1).as_deref(),
        Some("missing `cg`: give mass, cg and inertia, or [[mass.components]]")
    );
}

// ---------------------------------------------------------------- rotors

const SECOND_ROTOR: &str = "
[[rotors]]
name = \"second\"
position = { value = [0.16, 0.16, -0.10], unit = \"m\", source = \"sheet\" }
axis = { value = [0, 0, -1], unit = \"1\", source = \"sheet\" }
spin = \"cw\"
propeller = \"p10\"
";

/// The valid fixture with a second rotor, `change` made to it.
fn with_second_rotor(from: &str, to: &str) -> Option<String> {
    let rotor = SECOND_ROTOR
        .contains(from)
        .then(|| SECOND_ROTOR.replacen(from, to, 1))?;
    let at = VALID.find("\n[propellers.p10]")?;
    Some(format!("{}{rotor}{}", &VALID[..at], &VALID[at..]))
}

#[test]
fn rotors_load_with_a_unit_axis_and_a_spin() {
    let text = edited(&[(
        "value = [0, 0, -1], unit = \"1\"",
        "value = [0, 0, -2], unit = \"1\"",
    )])
    .expect("edits");
    let loaded = load(&text).expect("loads");
    let rotor = &loaded.airframe.rotors[0];
    assert_eq!(rotor.axis, Vec3::new(0.0, 0.0, -1.0));
    assert_eq!(rotor.spin, Spin::Ccw);
    assert_eq!((Spin::Cw.name(), Spin::Ccw.name()), ("cw", "ccw"));
    assert_eq!(rotor.name, "front right");
    assert_eq!(rotor.propeller, "p10");
    assert_eq!((rotor.motor.as_deref(), rotor.esc.as_deref()), (None, None));
}

#[test]
fn stacked_rotors_pass_and_tilted_close_ones_are_refused() {
    let stacked = with_second_rotor("cw", "cw").expect("edits");
    assert_eq!(
        load(&stacked)
            .map(|loaded| loaded.airframe.rotors.len())
            .ok(),
        Some(2)
    );
    let barely = with_second_rotor("-0.10]", "-0.0311]").expect("edits");
    assert!(load(&barely).is_ok(), "planes 1.1 mm apart are stacked");
    let touching = with_second_rotor("-0.10]", "-0.0309]");
    assert_eq!(refusal(touching).map(|r| r.0), Some("rotors[1]".to_owned()));
    let tilted = with_second_rotor("[0, 0, -1]", "[0.1, 0, -1]");
    assert_eq!(
        refusal(tilted),
        Some(refused(
            "rotors[1]",
            "it is tilted to rotors[0] and their hubs are 0.07 m apart, less than the 0.254 m \
             their radii add up to: this version cannot verify their clearance"
        ))
    );
    let apart = with_second_rotor("[0.16, 0.16, -0.10]", "[-0.16, 0.16, -0.03]").expect("edits");
    assert!(load(&apart).is_ok(), "0.32 m apart clears 0.254 m");
    let exactly = with_second_rotor("[0.16, 0.16, -0.10]", "[0.16, -0.094, -0.03]").expect("edits");
    assert!(
        load(&exactly).is_ok(),
        "hubs exactly the radii apart just clear"
    );
}

#[test]
fn rotors_must_name_defined_parts_and_have_an_axis() {
    let cases = [
        (
            edited(&[("propeller = \"p10\"", "propeller = \"p11\"")]),
            "rotors[0].propeller",
            "no entry `p11` in [propellers]",
        ),
        (
            edited(&[("propeller = \"p10\"", "propeller = \"p10\"\nmotor = \"m1\"")]),
            "rotors[0].motor",
            "no entry `m1` in [motors]",
        ),
        (
            edited(&[("propeller = \"p10\"", "propeller = \"p10\"\nesc = \"e1\"")]),
            "rotors[0].esc",
            "no entry `e1` in [escs]",
        ),
        (
            edited(&[(
                "value = [0, 0, -1], unit = \"1\"",
                "value = [0, 0, 0], unit = \"1\"",
            )]),
            "rotors[0].axis.value",
            "must not be zero",
        ),
        (
            edited(&[("name = \"front right\"", "name = \"\"")]),
            "rotors[0].name",
            "must not be empty",
        ),
        (
            edited(&[("name = \"Example quadrotor\"", "name = \"  \"")]),
            "name",
            "must not be empty",
        ),
    ];
    for (text, path, reason) in cases {
        assert_eq!(refusal(text), Some(refused(path, reason)));
    }
    // Top-level keys come before the first table, so the empty list goes after the name.
    let no_rotors = VALID.find("[[rotors]]").and_then(|at| {
        let end = VALID.find("[propellers.p10]")?;
        let text = format!("{}{}", &VALID[..at], &VALID[end..]);
        let name = "name = \"Example quadrotor\"\n";
        text.contains(name)
            .then(|| text.replacen(name, &format!("{name}rotors = []\n"), 1))
    });
    assert_eq!(
        refusal(no_rotors),
        Some(refused("rotors", "an airframe needs at least one rotor"))
    );
}

#[test]
fn propellers_need_positive_sizes_and_thrust() {
    let cases = [
        (
            ("diameter = { value = 10,", "diameter = { value = 0,"),
            "propellers.p10.diameter.value",
            "must be positive; found 0",
        ),
        (
            ("max_thrust = { value = 2,", "max_thrust = { value = -2,"),
            "propellers.p10.max_thrust.value",
            "must be positive; found -2",
        ),
    ];
    for ((from, to), path, reason) in cases {
        assert_eq!(refusal(edited(&[(from, to)])), Some(refused(path, reason)));
    }
}

#[test]
fn thrust_to_weight_is_total_thrust_over_weight() {
    let loaded = load(VALID).expect("loads");
    assert!((thrust_to_weight(&loaded.airframe) - 2.0 / 1.5).abs() < 1e-15);
    let barely = edited(&[(
        "max_thrust = { value = 2, unit = \"kgf\"",
        "max_thrust = { value = 1.5, unit = \"kgf\"",
    )]);
    assert_eq!(
        refusal(barely).map(|r| r.0),
        Some("rotors".to_owned()),
        "a ratio of exactly 1 cannot climb"
    );
}

// ---------------------------------------------------------------- propulsion and the rest

#[test]
fn motors_are_electrical_or_a_time_constant() {
    let motor = |body: &str| {
        edited(&[("propeller = \"p10\"", "propeller = \"p10\"\nmotor = \"m\"")])
            .map(|text| format!("{text}\n[motors.m]\n{body}"))
    };
    let electrical = "kv = { value = 920, unit = \"rpm/V\", source = \"sheet\" }\nresistance = { value = 0.1, unit = \"ohm\", source = \"sheet\" }\nno_load_current = { value = 0.4, unit = \"A\", source = \"sheet\" }\n";
    let loaded = load(&motor(electrical).expect("edits")).expect("loads");
    let kv = loaded.airframe.motors["m"].kv_rad_s_v.expect("kv");
    assert!((kv - 920.0 * 2.0 * core::f64::consts::PI / 60.0).abs() < 1e-12);
    let lag = load(
        &motor("time_constant = { value = 30, unit = \"ms\", source = \"sheet\" }\n")
            .expect("edits"),
    )
    .expect("loads");
    assert_eq!(lag.airframe.motors["m"].time_constant_s, Some(0.03));
    assert_eq!(
        refusal(motor(
            "kv = { value = 920, unit = \"rpm/V\", source = \"sheet\" }\n"
        )),
        Some(refused(
            "motors.m",
            "give kv, resistance and no_load_current together"
        ))
    );
    assert_eq!(
        refusal(motor(
            "mass = { value = 50, unit = \"g\", source = \"sheet\" }\n"
        )),
        Some(refused(
            "motors.m",
            "give kv, resistance and no_load_current, or a time_constant"
        ))
    );
}

#[test]
fn escs_batteries_drag_controllers_and_visual_hints_load_in_si() {
    let loaded = load(EXAMPLE).expect("the example loads");
    let airframe = &loaded.airframe;
    let esc = airframe.escs["20a"];
    assert_eq!((esc.latency_s, esc.braking), (Some(0.002), Braking::Active));
    let battery = airframe.battery.expect("a battery");
    assert_eq!(battery.cells, 3);
    assert!((battery.capacity_a_s - 10_800.0).abs() < 1e-9);
    let drag = airframe.drag.expect("drag");
    assert!((drag.area_m2[2] - 0.03).abs() < 1e-15);
    let controller = airframe.controller.as_ref().expect("a controller");
    assert_eq!(controller.kind, "cascaded-pid");
    assert!((controller.gains["attitude_p"] - 6.5).abs() < 1e-15);
    let visual = airframe.visual.as_ref().expect("visual hints");
    assert_eq!(visual.color.as_deref(), Some("#2f6fde"));
    assert_eq!(visual.arm_width_m, Some(0.02));
    assert_eq!(loaded.warnings, vec![]);
}

#[test]
fn bad_values_in_the_other_tables_are_refused() {
    let cases = [
        (
            "[drag]\narea = { value = [0.01, -0.01, 0.03], unit = \"m^2\", source = \"sheet\" }\n",
            "drag.area.value[1]",
            "must not be negative",
        ),
        (
            "[controller]\nkind = \"\"\n",
            "controller.kind",
            "must not be empty",
        ),
        (
            "[controller]\nkind = \"pid\"\ngains.p = { value = 1, unit = \"furlong\", source = \"sheet\" }\n",
            "controller.gains.p.unit",
            "unknown unit `furlong`",
        ),
        (
            "[visual]\ncolor = \"blue\"\n",
            "visual.color",
            "a color is # and six hexadecimal digits; found \"blue\"",
        ),
        (
            "[visual]\ncolor = \"#12345g\"\n",
            "visual.color",
            "a color is # and six hexadecimal digits",
        ),
        (
            "[visual]\nbody_size = { value = [0.1, 0, 0.1], unit = \"m\", source = \"sheet\" }\n",
            "visual.body_size.value[1]",
            "must be positive",
        ),
        (
            "[battery]\ncells = { value = 3, unit = \"1\", source = \"sheet\" }\ncapacity = { value = 3000, unit = \"mA\", source = \"sheet\" }\n",
            "battery.capacity.unit",
            "`mA` is not a charge; use a unit such as mAh or Ah",
        ),
        (
            "[escs.e]\nbraking = \"sometimes\"\n",
            "escs.e.braking",
            "unknown variant `sometimes`",
        ),
    ];
    for (extra, path, reason) in cases {
        let (found_path, found_reason) = refusal(Some(extended(extra))).expect(path);
        assert_eq!(found_path, path);
        assert!(found_reason.contains(reason), "{found_reason}");
    }
}

#[test]
fn parts_no_rotor_uses_are_reported() {
    let text = extended(
        "\n[propellers.spare]\ndiameter = { value = 5, unit = \"in\", source = \"sheet\" }\nblades = { value = 3, unit = \"1\", source = \"sheet\" }\nmax_thrust = { value = 1, unit = \"N\", source = \"sheet\" }\n\n[motors.spare]\ntime_constant = { value = 0.02, unit = \"s\", source = \"sheet\" }\n\n[escs.spare]\nbraking = \"none\"\n",
    );
    let loaded = load(&text).expect("loads");
    let warnings: Vec<String> = loaded.warnings.iter().map(ToString::to_string).collect();
    assert_eq!(
        warnings,
        [
            "propellers.spare: no rotor uses it",
            "motors.spare: no rotor uses it",
            "escs.spare: no rotor uses it",
        ]
    );
    assert_eq!(loaded.airframe.escs["spare"].braking, Braking::None);
}

// ---------------------------------------------------------------- the generated page

#[test]
fn the_published_page_matches_the_example_airframe() {
    let loaded = load_airframe("airframes/example-quadrotor.toml", EXAMPLE).expect("loads");
    assert_eq!(airframe_page(&loaded), EXAMPLE_PAGE);
}

#[test]
fn the_page_lists_every_parameter_with_its_source() {
    let loaded = load(EXAMPLE).expect("loads");
    let page = airframe_page(&loaded);
    assert_eq!(loaded.parameters.len(), 70);
    for record in &loaded.parameters {
        let row = format!(
            "| `{}` | {} | `{}` | {} |",
            record.path, record.value, record.unit, record.si
        );
        assert!(page.contains(&row), "{row}");
    }
    assert!(
        loaded
            .parameters
            .iter()
            .all(|record| record.source.as_deref() == Some("illustrative"))
    );
    assert!(page.contains("| 2 | front left | [0.1591, -0.1591, -0.035] m | [0, 0, -1] | cw | `10x4.5` | `2212-920` | `20a` |"));
    assert!(page.contains("| 7 | front right motor and propeller | cylinder |"));
    assert!(!page.contains("## Warnings"));
}

#[test]
fn the_page_shows_missing_sources_and_warnings() {
    let text = include_str!("fixtures/airframe/missing_source.toml");
    let page = airframe_page(&load(text).expect("loads"));
    assert!(
        page.contains("| `mass.mass` | 1.5 | `kg` | 1.5 kg | — | **none** |"),
        "{page}"
    );
    assert!(
        page.ends_with("\n## Warnings\n\n- `mass.mass`: no source\n"),
        "{page}"
    );
    assert!(
        !page.contains("## Parts"),
        "a direct mass table has no parts"
    );
    assert!(page.contains(
        "| 0 | front right | [0.16, 0.16, -0.03] m | [0, 0, -1] | ccw | `p10` | — | — |"
    ));
}

#[test]
fn table_cells_escape_pipes() {
    let text = edited(&[("name = \"front right\"", "name = \"left | right\"")]).expect("edits");
    let page = airframe_page(&load(&text).expect("loads"));
    assert!(page.contains("| left \\| right |"), "{page}");
}

// ---------------------------------------------------------------- boundaries found by mutation testing

#[test]
fn powers_and_derived_symbols_have_the_right_dimensions() {
    let powered = parse_unit("kg^2 s^-2 A^3 rad^2").map(|unit| unit.dimension);
    assert_eq!(powered, Ok(Dimension([0, 2, -2, 3, 2])));
    let derived = [
        ("Hz", "1/s"),
        ("N", "kg m/s^2"),
        ("J", "N m"),
        ("W", "J/s"),
        ("V", "W/A"),
        ("mV", "W/A"),
        ("ohm", "V/A"),
        ("Ah", "A s"),
        ("rpm", "rad/s"),
    ];
    for (symbol, definition) in derived {
        assert_eq!(
            parse_unit(symbol).map(|unit| unit.dimension),
            parse_unit(definition).map(|unit| unit.dimension),
            "{symbol}"
        );
    }
}

#[test]
fn a_value_that_is_not_a_table_or_a_number_is_refused() {
    assert_refused(
        edited(&[(
            "mass = { value = 1.5, unit = \"kg\", source = \"sheet\" }",
            "mass = \"heavy\"",
        )]),
        "mass.mass",
        "expected a table { value, unit, source, uncertainty }",
    );
}

#[test]
fn a_matrix_uncertainty_is_checked_entry_by_entry() {
    assert_refused(
        edited(&[(
            "unit = \"kg m^2\"",
            "unit = \"kg m^2\"\nuncertainty = [[0, 0, 0], [0, 0, -1e-4], [0, 0, 0]]",
        )]),
        "mass.inertia.uncertainty[1][2]",
        "an uncertainty must be finite and not negative; found -0.0001",
    );
}

#[test]
fn the_page_shows_a_matrix_as_rows() {
    let page = airframe_page(&load(VALID).expect("loads"));
    let row = "| `mass.inertia` | [[0.029, 0, 0.0004], [0, 0.03, 0], [0.0004, 0, 0.055]] | `kg m^2` \
               | [[0.029, 0, 0.0004], [0, 0.03, 0], [0.0004, 0, 0.055]] kg·m² | — | `sheet` |";
    assert!(page.contains(row), "{page}");
}

#[test]
fn a_zero_drag_area_is_allowed() {
    let text =
        extended("[drag]\narea = { value = [0, 0, 0.03], unit = \"m^2\", source = \"sheet\" }\n");
    let loaded = load(&text).expect("loads");
    assert_eq!(loaded.airframe.drag.map(|drag| drag.area_m2[0]), Some(0.0));
}

#[test]
fn every_pair_of_rotors_is_checked() {
    let third = "
[[rotors]]
name = \"rear right\"
position = { value = [-0.16, 0.16, -0.03], unit = \"m\", source = \"sheet\" }
axis = { value = [0, 0, -1], unit = \"1\", source = \"sheet\" }
spin = \"cw\"
propeller = \"p10\"

[[rotors]]
name = \"rear left\"
position = { value = [-0.16, -0.05, -0.03], unit = \"m\", source = \"sheet\" }
axis = { value = [0, 0, -1], unit = \"1\", source = \"sheet\" }
spin = \"ccw\"
propeller = \"p10\"
";
    let text = VALID
        .find("\n[propellers.p10]")
        .map(|at| format!("{}{third}{}", &VALID[..at], &VALID[at..]));
    assert_refused(text, "rotors[2]", "its disk overlaps rotors[1]'s");
}

#[test]
fn disk_planes_exactly_1_mm_apart_are_one_plane() {
    // Hubs at z = 0 and z = -0.001 m: the planes are COPLANAR_M apart to the bit.
    let text = with_second_rotor("[0.16, 0.16, -0.10]", "[0.16, 0.16, -0.001]").and_then(|text| {
        let first = "[0.16, 0.16, -0.03], unit = \"m\"";
        text.contains(first)
            .then(|| text.replacen(first, "[0.16, 0.16, 0], unit = \"m\"", 1))
    });
    assert_refused(text, "rotors[1]", "its disk overlaps rotors[0]'s");
}
