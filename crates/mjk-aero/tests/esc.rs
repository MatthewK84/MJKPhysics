//! The ESC (#19): latency in whole ticks, rounded to the nearest; a slew limit per tick;
//! commands saturated to 0–1; and parameters that are not usable refused.

use mjk_aero::propulsion::{Braking, Drive, Esc, EscError};

const DRIVE: Drive = Drive {
    braking: Braking::Active,
    current_limit_a: None,
};

fn esc(latency_s: f64, slew: Option<f64>) -> Result<Esc, EscError> {
    Esc::new(latency_s, slew, DRIVE, 1000)
}

#[test]
fn latency_rounds_to_the_nearest_tick_halves_up() {
    let cases = [
        (0.0, 0),
        (0.000_4, 0),
        (0.000_5, 1),
        (0.002, 2),
        (0.002_4, 2),
        (0.002_5, 3),
        (0.002_6, 3),
        (1.0, 1000),
    ];
    for (latency_s, ticks) in cases {
        assert_eq!(
            esc(latency_s, None).expect("a usable ESC").latency_ticks(),
            ticks,
            "{latency_s}"
        );
    }
    let slow = Esc::new(4_294_967_295.0, None, DRIVE, 1).expect("the largest latency");
    assert_eq!(slow.latency_ticks(), u32::MAX);
    let big = Esc::new(1_234_567.0, None, DRIVE, 1).expect("a large latency");
    assert_eq!(big.latency_ticks(), 1_234_567);
}

#[test]
fn unusable_parameters_are_refused() {
    for latency_s in [-0.001, f64::NAN, f64::INFINITY] {
        let refused = Esc::new(latency_s, None, DRIVE, 1000);
        assert!(
            matches!(refused, Err(EscError::Latency { latency_s: found }) if found.to_bits() == latency_s.to_bits()),
            "{latency_s}: {refused:?}"
        );
    }
    assert_eq!(
        Esc::new(4_294_967_296.0, None, DRIVE, 1),
        Err(EscError::Latency {
            latency_s: 4_294_967_296.0
        })
    );
    for slew in [0.0, -1.0, f64::INFINITY] {
        assert_eq!(
            Esc::new(0.0, Some(slew), DRIVE, 1000),
            Err(EscError::SlewRate {
                slew_rate_per_s: slew
            })
        );
    }
    assert!(Esc::new(0.0, Some(f64::NAN), DRIVE, 1000).is_err());
    for limit in [0.0, -5.0, f64::NAN] {
        let drive = Drive {
            braking: Braking::None,
            current_limit_a: Some(limit),
        };
        assert!(Esc::new(0.0, None, drive, 1000).is_err(), "{limit}");
    }
    assert_eq!(Esc::new(0.0, None, DRIVE, 0), Err(EscError::Rate));
}

#[test]
fn errors_explain_themselves() {
    let cases = [
        (
            EscError::Latency { latency_s: -0.001 },
            "a latency must be finite, not negative, and a whole number of ticks that fits in 32 \
             bits; found -0.001 s",
        ),
        (
            EscError::SlewRate {
                slew_rate_per_s: 0.0,
            },
            "a slew rate must be positive and finite; found 0 per second",
        ),
        (
            EscError::CurrentLimit {
                current_limit_a: -5.0,
            },
            "a current limit must be positive and finite; found -5 A",
        ),
        (EscError::Rate, "the physics rate must be at least 1 Hz"),
    ];
    for (error, message) in cases {
        assert_eq!(error.to_string(), message);
    }
}

#[test]
fn accessors_return_what_was_built() {
    let built = Esc::new(0.0, Some(40.0), DRIVE, 1000).expect("usable");
    assert_eq!(built.slew_per_tick(), Some(0.04));
    assert_eq!(built.drive(), DRIVE);
    assert_eq!(esc(0.0, None).expect("a usable ESC").slew_per_tick(), None);
}

#[test]
fn without_latency_or_slew_the_duty_is_the_saturated_command() {
    let mut state = esc(0.0, None).expect("a usable ESC").start(0.0);
    assert_eq!(state.duty().to_bits(), 0.0_f64.to_bits());
    let cases: [(f64, f64); 5] = [
        (0.3, 0.3),
        (1.7, 1.0),
        (-0.2, 0.0),
        (f64::NAN, 0.0),
        (1.0, 1.0),
    ];
    for (command, duty) in cases {
        assert_eq!(state.update(command).to_bits(), duty.to_bits(), "{command}");
        assert_eq!(state.duty().to_bits(), duty.to_bits());
    }
}

#[test]
fn a_command_arrives_after_exactly_the_latency() {
    let mut state = esc(0.003, None).expect("a usable ESC").start(0.2);
    assert_eq!(state.duty().to_bits(), 0.2_f64.to_bits());
    let duties: Vec<f64> = [0.9, 0.9, 0.9, 0.9, 0.5, 0.5, 0.5, 0.5]
        .into_iter()
        .map(|command| state.update(command))
        .collect();
    assert_eq!(duties, [0.2, 0.2, 0.2, 0.9, 0.9, 0.9, 0.9, 0.5]);
    let saturated = esc(0.001, None).expect("a usable ESC").start(5.0);
    assert_eq!(
        saturated.duty().to_bits(),
        1.0_f64.to_bits(),
        "the starting duty is saturated too"
    );
}

#[test]
fn the_slew_limit_moves_the_duty_by_at_most_one_step_a_tick() {
    let mut state = esc(0.0, Some(40.0)).expect("a usable ESC").start(0.0);
    let ticks_up = (1..=100).find(|_| state.update(1.0) >= 1.0);
    assert_eq!(ticks_up, Some(25), "40 per second at 1 kHz is 0.04 a tick");
    assert_eq!(
        state.duty().to_bits(),
        1.0_f64.to_bits(),
        "it lands exactly on the target"
    );
    let first_down = state.update(0.9);
    assert!((first_down - 0.96).abs() < 1e-12, "{first_down}");
    let second_down = state.update(0.9);
    assert!((second_down - 0.92).abs() < 1e-12, "{second_down}");
    assert_eq!(
        state.update(0.9).to_bits(),
        0.9_f64.to_bits(),
        "within one step, it lands on the target"
    );
}

#[test]
fn a_negative_zero_command_reads_as_zero() {
    let mut state = esc(0.0, None).expect("a usable ESC").start(0.5);
    assert_eq!(state.update(-0.0).to_bits(), 0.0_f64.to_bits());
}
