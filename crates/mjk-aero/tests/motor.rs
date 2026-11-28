//! The motor model (#19): its equations, braking and saturation, its closed forms, and the
//! system the integrators step.

use mjk_aero::propulsion::{
    Braking, ConstantLoad, DcMotor, Drive, DriveInput, LagMotor, LoadTorque, MotorError,
    MotorModel, MotorSpeed, MotorSystem, PropellerLoad,
};
use mjk_dynamics::{Coordinates, LieSystem, System};
use mjk_math::{StateHash, StateHasher};

/// 920 rpm/V in rad/(s·V).
const KV: f64 = 920.0 * 2.0 * core::f64::consts::PI / 60.0;
const R: f64 = 0.117;
const I0: f64 = 0.4;
const J: f64 = 8.5e-5;

const ACTIVE: Drive = Drive {
    braking: Braking::Active,
    current_limit_a: None,
};
const COAST: Drive = Drive {
    braking: Braking::None,
    current_limit_a: None,
};

fn motor() -> Result<DcMotor, MotorError> {
    DcMotor::new(KV, R, I0, J)
}

fn close(a: f64, b: f64, relative: f64) -> bool {
    (a - b).abs() <= relative * b.abs().max(1.0)
}

#[test]
fn parameters_must_be_usable() {
    let cases = [
        (
            DcMotor::new(0.0, R, I0, J),
            "the speed constant",
            0.0,
            false,
        ),
        (DcMotor::new(KV, -R, I0, J), "the resistance", -R, false),
        (
            DcMotor::new(KV, R, -0.1, J),
            "the no-load current",
            -0.1,
            true,
        ),
        (
            DcMotor::new(KV, R, I0, f64::INFINITY),
            "the inertia",
            f64::INFINITY,
            false,
        ),
    ];
    for (result, name, value, zero_allowed) in cases {
        assert_eq!(
            result,
            Err(MotorError {
                name,
                value,
                zero_allowed
            }),
            "{name}"
        );
    }
    assert!(
        DcMotor::new(KV, R, 0.0, J).is_ok(),
        "no friction is allowed"
    );
    assert!(DcMotor::new(f64::NAN, R, I0, J).is_err());
    assert_eq!(
        LagMotor::new(0.03, 0.0).map(|m| m.max_speed_rad_s()),
        Err(MotorError {
            name: "the largest speed",
            value: 0.0,
            zero_allowed: false
        })
    );
    assert!(LagMotor::new(-0.03, 100.0).is_err());
}

#[test]
fn errors_explain_themselves() {
    let negative = DcMotor::new(KV, R, -0.1, J).expect_err("negative current");
    assert_eq!(
        negative.to_string(),
        "the no-load current must be finite and not negative; found -0.1"
    );
    let zero = DcMotor::new(KV, 0.0, I0, J).expect_err("zero resistance");
    assert_eq!(
        zero.to_string(),
        "the resistance must be positive and finite; found 0"
    );
}

#[test]
fn accessors_return_the_parameters() {
    let m = motor().expect("a usable motor");
    assert_eq!(
        (
            m.kv_rad_s_v(),
            m.resistance_ohm(),
            m.no_load_current_a(),
            m.inertia_kg_m2()
        ),
        (KV, R, I0, J)
    );
    let lag = LagMotor::new(0.03, 2500.0).expect("usable");
    assert_eq!(
        (lag.time_constant_s(), lag.max_speed_rad_s()),
        (0.03, 2500.0)
    );
}

#[test]
fn current_follows_the_voltage_less_the_back_emf() {
    let m = motor().expect("a usable motor");
    // Eq. (MOT-1): (V − ω/K_v)/R.
    let current = m.current_a(500.0, 11.1, ACTIVE);
    assert!(close(current, (11.1 - 500.0 / KV) / R, 1e-15));
    assert!(
        m.current_a(1500.0, 11.1, ACTIVE) < 0.0,
        "active braking reverses"
    );
    assert_eq!(
        m.current_a(1500.0, 11.1, COAST).to_bits(),
        0.0_f64.to_bits(),
        "coasting never reverses"
    );
    assert!(m.current_a(500.0, 11.1, COAST) > 0.0);
    assert!(
        m.current_a(f64::NAN, 11.1, ACTIVE).is_nan(),
        "a NaN is seen, not hidden"
    );
}

#[test]
fn current_is_held_within_the_limit() {
    let m = motor().expect("a usable motor");
    let limited = Drive {
        braking: Braking::Active,
        current_limit_a: Some(15.0),
    };
    assert_eq!(
        m.current_a(0.0, 11.1, limited).to_bits(),
        15.0_f64.to_bits()
    );
    assert_eq!(
        m.current_a(3000.0, 0.0, limited).to_bits(),
        (-15.0_f64).to_bits()
    );
    let coast = Drive {
        braking: Braking::None,
        current_limit_a: Some(15.0),
    };
    assert_eq!(m.current_a(0.0, 11.1, coast).to_bits(), 15.0_f64.to_bits());
    assert_eq!(m.current_a(3000.0, 0.0, coast).to_bits(), 0.0_f64.to_bits());
    let free = m.current_a(1000.0, 11.1, limited);
    assert!(close(free, (11.1 - 1000.0 / KV) / R, 1e-15), "{free}");
}

#[test]
fn torque_is_current_over_the_speed_constant_less_friction() {
    let m = motor().expect("a usable motor");
    assert!(close(m.torque_n_m(10.0), (10.0 - I0) / KV, 1e-15));
    assert!(m.torque_n_m(0.0) < 0.0, "friction opposes the spin");
}

#[test]
fn a_motor_at_rest_is_not_driven_backward() {
    let m = motor().expect("a usable motor");
    assert_eq!(
        m.acceleration(0.0, 0.0, 0.01, ACTIVE).to_bits(),
        0.0_f64.to_bits()
    );
    assert_eq!(
        m.acceleration(-1.0, 0.0, 0.01, ACTIVE).to_bits(),
        0.0_f64.to_bits()
    );
    assert!(m.acceleration(0.0, 11.1, 0.0, ACTIVE) > 0.0);
    let spinning = m.acceleration(100.0, 0.0, 0.01, COAST);
    assert!(close(spinning, (-I0 / KV - 0.01) / J, 1e-15), "{spinning}");
}

#[test]
fn the_operating_point_balances_power() {
    let m = motor().expect("a usable motor");
    let input = DriveInput {
        duty: 0.8,
        supply_v: 11.1,
        drive: ACTIVE,
    };
    let speed = 700.0;
    let point = m.operating_point(speed, &input);
    let voltage = 0.8 * 11.1;
    assert!((input.voltage_v() - voltage).abs() < 1e-15);
    assert!(close(point.current_a, (voltage - speed / KV) / R, 1e-15));
    assert!(close(point.torque_n_m, (point.current_a - I0) / KV, 1e-15));
    assert!(close(
        point.electrical_power_w,
        voltage * point.current_a,
        1e-15
    ));
    assert!(close(
        point.mechanical_power_w,
        point.torque_n_m * speed,
        1e-15
    ));
    assert!(close(point.supply_current_a, 0.8 * point.current_a, 1e-15));
    // Electrical power is mechanical power plus copper loss plus friction loss.
    let losses = point.current_a * point.current_a * R + I0 * speed / KV;
    assert!(close(
        point.electrical_power_w,
        point.mechanical_power_w + losses,
        1e-13
    ));
}

#[test]
fn the_steady_speeds_are_where_the_acceleration_vanishes() {
    let m = motor().expect("a usable motor");
    for load in [0.0, 0.02, 0.05] {
        let speed = m.steady_speed(11.1, load);
        assert!(
            m.acceleration(speed, 11.1, load, ACTIVE).abs() < 1e-8,
            "{load}"
        );
    }
    let propeller = PropellerLoad {
        coefficient_n_m_s2: 1.3e-7,
    };
    let speed = m.steady_speed_quadratic(8.0, propeller.coefficient_n_m_s2);
    let rate = m.acceleration(speed, 8.0, propeller.torque_n_m(speed), ACTIVE);
    assert!(rate.abs() < 1e-8, "{rate}");
    let unloaded = m.steady_speed_quadratic(8.0, 0.0);
    assert!(
        close(unloaded, m.steady_speed(8.0, 0.0), 1e-14),
        "{unloaded}"
    );
}

#[test]
fn the_time_constant_is_the_inverse_slope_of_the_acceleration() {
    let m = motor().expect("a usable motor");
    let propeller = PropellerLoad {
        coefficient_n_m_s2: 1.3e-7,
    };
    let speed = m.steady_speed_quadratic(8.0, propeller.coefficient_n_m_s2);
    let acceleration = |w: f64| m.acceleration(w, 8.0, propeller.torque_n_m(w), ACTIVE);
    let dw = 1e-3;
    let slope = (acceleration(speed + dw) - acceleration(speed - dw)) / (2.0 * dw);
    let tau = m.time_constant_s(propeller.slope(speed));
    assert!(
        close(-1.0 / slope, tau, 1e-6),
        "{} against {tau}",
        -1.0 / slope
    );
    assert!(close(m.time_constant_s(0.0), J * R * KV * KV, 1e-14));
}

#[test]
fn the_lag_tier_approaches_its_commanded_speed() {
    let lag = LagMotor::new(0.03, 2500.0).expect("usable");
    assert!(close(
        lag.acceleration(1000.0, 0.6),
        (1500.0 - 1000.0) / 0.03,
        1e-15
    ));
    let input = DriveInput {
        duty: 0.6,
        supply_v: 0.0,
        drive: ACTIVE,
    };
    let model = MotorModel::Lag(lag);
    assert!(close(
        model.acceleration(1000.0, &input, 99.0),
        lag.acceleration(1000.0, 0.6),
        1e-15
    ));
    let dc = MotorModel::Dc(motor().expect("a usable motor"));
    let full = DriveInput {
        duty: 1.0,
        supply_v: 11.1,
        drive: ACTIVE,
    };
    assert!(close(
        dc.acceleration(500.0, &full, 0.02),
        motor()
            .expect("a usable motor")
            .acceleration(500.0, 11.1, 0.02, ACTIVE),
        1e-15
    ));
}

#[test]
fn loads_give_their_torque_and_slope() {
    let constant = ConstantLoad { torque_n_m: 0.03 };
    assert_eq!(
        (constant.torque_n_m(500.0), constant.slope(500.0)),
        (0.03, 0.0)
    );
    let propeller = PropellerLoad {
        coefficient_n_m_s2: 2e-7,
    };
    assert!(close(
        propeller.torque_n_m(500.0),
        2e-7 * 500.0 * 500.0,
        1e-15
    ));
    assert!(close(propeller.slope(500.0), 2.0 * 2e-7 * 500.0, 1e-15));
}

#[test]
fn speeds_are_coordinates_that_never_go_negative() {
    let a = MotorSpeed { speed_rad_s: 3.0 };
    let b = MotorSpeed { speed_rad_s: 4.5 };
    assert_eq!((a + b).speed_rad_s.to_bits(), 7.5_f64.to_bits());
    assert_eq!((a * 2.0).speed_rad_s.to_bits(), 6.0_f64.to_bits());
    assert_eq!(a.positions(), MotorSpeed::default());
    assert_eq!(a.velocities(), a);
    assert_eq!(
        MotorSpeed { speed_rad_s: -0.1 }.project(),
        MotorSpeed::default()
    );
    assert_eq!(
        MotorSpeed { speed_rad_s: -0.0 }
            .project()
            .speed_rad_s
            .to_bits(),
        0.0_f64.to_bits(),
        "a step that lands on −0 is at rest at +0"
    );
    assert_eq!(a.project(), a);
    assert!(
        MotorSpeed {
            speed_rad_s: f64::NAN
        }
        .project()
        .speed_rad_s
        .is_nan()
    );
    let hash = |speed: MotorSpeed| {
        let mut hasher = StateHasher::new();
        speed.hash_state(&mut hasher);
        hasher.finish()
    };
    assert_ne!(hash(a), hash(b));
    assert_eq!(hash(a), hash(MotorSpeed { speed_rad_s: 3.0 }));
}

#[test]
fn the_system_rate_is_the_acceleration_and_its_chart_is_addition() {
    let system = MotorSystem {
        motor: MotorModel::Dc(motor().expect("a usable motor")),
        load: ConstantLoad { torque_n_m: 0.02 },
        input: DriveInput {
            duty: 1.0,
            supply_v: 11.1,
            drive: ACTIVE,
        },
    };
    let y = MotorSpeed { speed_rad_s: 400.0 };
    let rate = system.rate(0.0, &y).speed_rad_s;
    assert!(close(
        rate,
        motor()
            .expect("a usable motor")
            .acceleration(400.0, 11.1, 0.02, ACTIVE),
        1e-15
    ));
    let u = MotorSpeed { speed_rad_s: 50.0 };
    assert_eq!(
        system.retract(&y, &u).speed_rad_s.to_bits(),
        450.0_f64.to_bits()
    );
    let tangent = system.tangent_rate(0.0, &y, &u).speed_rad_s;
    assert!(close(
        tangent,
        system
            .rate(0.0, &MotorSpeed { speed_rad_s: 450.0 })
            .speed_rad_s,
        1e-15
    ));
}
