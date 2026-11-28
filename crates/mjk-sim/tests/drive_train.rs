//! From an airframe to motor and ESC models (#19): the electrical tier turns the motor's and
//! propeller's inertia together, the lag tier follows its top speed, the ESC's latency is counted
//! in ticks at the run's physics rate, and a rotor without an ESC gets an ideal one.

use mjk_aero::propulsion::{Braking, EscError, MotorError, MotorModel};
use mjk_sim::config::{DriveError, DriveTrain, LoadedAirframe, drive_train, load_airframe};

const EXAMPLE: &str = include_str!("../../../airframes/example-quadrotor.toml");
const VALID: &str = include_str!("fixtures/airframe/valid.toml");

fn load(text: &str) -> Option<LoadedAirframe> {
    load_airframe("test.toml", text).ok()
}

/// The drive train of the first rotor of `text` at `physics_hz`.
fn first(text: &str, physics_hz: u32) -> Option<Result<Option<DriveTrain>, DriveError>> {
    let loaded = load(text)?;
    let rotor = loaded.airframe.rotors.first()?;
    Some(drive_train(&loaded.airframe, rotor, physics_hz))
}

/// The valid fixture with its rotor driven by motor `m` (whose table is `motor`) and, if given,
/// ESC `e` (whose table is `esc`).
fn with_drive(motor: &str, esc: Option<&str>) -> String {
    let rotor = if esc.is_some() {
        "propeller = \"p10\"\nmotor = \"m\"\nesc = \"e\""
    } else {
        "propeller = \"p10\"\nmotor = \"m\""
    };
    let text = VALID.replacen("propeller = \"p10\"", rotor, 1);
    let esc_table = esc.map_or_else(String::new, |body| format!("\n[escs.e]\n{body}"));
    format!("{text}\n[motors.m]\n{motor}{esc_table}")
}

const LAG: &str = "time_constant = { value = 30, unit = \"ms\", source = \"sheet\" }
max_speed = { value = 24000, unit = \"rpm\", source = \"sheet\" }
max_current = { value = 10, unit = \"A\", source = \"sheet\" }
";

#[test]
fn the_example_drives_its_rotors_with_the_electrical_model() {
    let train = first(EXAMPLE, 1000)
        .expect("loads")
        .expect("builds")
        .expect("a motor");
    let MotorModel::Dc(motor) = train.motor else {
        panic!("the electrical tier");
    };
    assert!((motor.kv_rad_s_v() - 920.0 * 2.0 * core::f64::consts::PI / 60.0).abs() < 1e-12);
    assert!((motor.resistance_ohm() - 0.117).abs() < 1e-15);
    assert!((motor.no_load_current_a() - 0.4).abs() < 1e-15);
    // The motor's 200 g·cm² and the propeller's 645 g·cm² together.
    assert!((motor.inertia_kg_m2() - 8.45e-5).abs() < 1e-18);
    assert_eq!(train.esc.latency_ticks(), 2);
    assert_eq!(train.esc.slew_per_tick(), Some(0.04));
    assert_eq!(train.esc.drive().braking, Braking::Active);
    assert_eq!(
        train.esc.drive().current_limit_a,
        Some(15.0),
        "the smaller of 15 A and 20 A"
    );
}

#[test]
fn latency_is_counted_at_the_runs_physics_rate() {
    let slower = first(EXAMPLE, 400)
        .expect("loads")
        .expect("builds")
        .expect("a motor");
    assert_eq!(
        slower.esc.latency_ticks(),
        1,
        "2 ms at 400 Hz is 0.8 ticks, which rounds to 1"
    );
    assert_eq!(slower.esc.slew_per_tick(), Some(0.1));
}

#[test]
fn the_lag_tier_and_the_ideal_esc() {
    let train = first(&with_drive(LAG, None), 1000)
        .expect("loads")
        .expect("builds")
        .expect("a motor");
    let MotorModel::Lag(lag) = train.motor else {
        panic!("the lag tier");
    };
    assert!((lag.time_constant_s() - 0.03).abs() < 1e-15);
    assert!((lag.max_speed_rad_s() - 24_000.0 * 2.0 * core::f64::consts::PI / 60.0).abs() < 1e-9);
    assert_eq!(train.esc.latency_ticks(), 0);
    assert_eq!(train.esc.slew_per_tick(), None);
    assert_eq!(train.esc.drive().braking, Braking::Active);
    assert_eq!(
        train.esc.drive().current_limit_a,
        Some(10.0),
        "the motor's limit alone"
    );
}

#[test]
fn an_escs_braking_and_limit_carry_over() {
    let esc = "braking = \"none\"\nmax_current = { value = 8, unit = \"A\", source = \"sheet\" }\n";
    let train = first(&with_drive(LAG, Some(esc)), 1000)
        .expect("loads")
        .expect("builds")
        .expect("a motor");
    assert_eq!(train.esc.drive().braking, Braking::None);
    assert_eq!(
        train.esc.drive().current_limit_a,
        Some(8.0),
        "the smaller of 10 A and 8 A"
    );
    let unlimited = "time_constant = { value = 30, unit = \"ms\", source = \"sheet\" }
max_speed = { value = 24000, unit = \"rpm\", source = \"sheet\" }
";
    let free = first(&with_drive(unlimited, Some("braking = \"active\"\n")), 1000)
        .expect("loads")
        .expect("builds")
        .expect("a motor");
    assert_eq!(free.esc.drive().current_limit_a, None);
}

#[test]
fn a_rotor_without_a_motor_has_no_drive_train() {
    assert_eq!(first(VALID, 1000).expect("loads"), Ok(None));
}

#[test]
fn an_esc_that_cannot_be_counted_in_ticks_is_refused() {
    let esc = "braking = \"none\"\nlatency = { value = 1e7, unit = \"s\", source = \"sheet\" }\n";
    let refused = first(&with_drive(LAG, Some(esc)), 1000).expect("loads");
    assert_eq!(
        refused,
        Err(DriveError::Esc {
            key: Some("e".to_owned()),
            error: EscError::Latency { latency_s: 1e7 },
        })
    );
    let ideal = first(&with_drive(LAG, None), 0).expect("loads");
    assert_eq!(
        ideal.map_err(|error| error.to_string()),
        Err("the ideal ESC: the physics rate must be at least 1 Hz".to_owned())
    );
}

#[test]
fn errors_explain_themselves() {
    let motor = DriveError::Motor {
        key: "2212".to_owned(),
        error: MotorError {
            name: "the inertia",
            value: 0.0,
            zero_allowed: false,
        },
    };
    assert_eq!(
        motor.to_string(),
        "motor `2212`: the inertia must be positive and finite; found 0"
    );
    let esc = DriveError::Esc {
        key: Some("20a".to_owned()),
        error: EscError::Rate,
    };
    assert_eq!(
        esc.to_string(),
        "ESC `20a`: the physics rate must be at least 1 Hz"
    );
}
