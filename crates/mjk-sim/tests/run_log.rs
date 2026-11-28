//! Recorded runs (#17): a record of the reference scenario holds its inputs, fingerprints and
//! state log; replaying it reproduces every sample to the bit; and any change to the record is
//! found at the first tick it affects, or rejected with the field at fault.

use mjk_core::EngineError;
use mjk_sim::log::{LogError, Sample};
use mjk_sim::run_log::{
    Divergence, FORMAT_VERSION, REFERENCE, ReplayError, ReplayReport, RunLog, hex,
    record_reference, replay_run,
};
use mjk_sim::validation::determinism::{
    CHANNELS, MASTER_SEED, PHYSICS_HZ, SAMPLE_HZ, ScenarioError, TICKS, determinism_study,
};

fn recorded() -> Result<RunLog, ScenarioError> {
    record_reference(MASTER_SEED)
}

/// Records the reference run, applies `change` to the record, and replays it.
fn replay_after(change: impl FnOnce(&mut RunLog)) -> Option<Result<ReplayReport, ReplayError>> {
    let mut log = recorded().ok()?;
    change(&mut log);
    Some(replay_run("run.json", &log))
}

/// Where the changed record's replay first differs from it, if anywhere.
fn divergence_after(change: impl FnOnce(&mut RunLog)) -> Option<Divergence> {
    replay_after(change)?.ok()?.divergence
}

/// The field and message with which the changed record is refused, if it is.
fn refusal_after(change: impl FnOnce(&mut RunLog)) -> Option<(String, String)> {
    match replay_after(change)? {
        Err(ReplayError::Log(EngineError::Config { path, reason, .. })) => {
            Some((path.to_string(), reason))
        }
        _ => None,
    }
}

/// A change to a record.
type Change = fn(&mut RunLog);

fn refusal(field: &str, reason: &str) -> (String, String) {
    (field.to_owned(), reason.to_owned())
}

#[test]
fn a_record_holds_the_header_inputs_fingerprints_and_state_log() {
    let log = recorded().expect("the reference scenario runs");
    assert_eq!(log.format_version, FORMAT_VERSION);
    assert_eq!(log.scenario, REFERENCE);
    assert_eq!(
        (log.seed, log.physics_hz, log.ticks, log.sample_hz),
        (MASTER_SEED, PHYSICS_HZ, TICKS, SAMPLE_HZ)
    );
    assert_eq!(log.inputs.len(), 20);
    assert_eq!(log.inputs[1].tick, 500);
    assert_eq!(log.checkpoints.len(), 11);
    assert_eq!(log.checkpoints[10].tick, TICKS);
    let (study, _) = determinism_study().expect("the study runs");
    assert_eq!(log.final_hash, hex(study.final_hash));
    assert_eq!(log.checkpoints[3].hash, hex(study.checkpoints[3].hash));
    assert_eq!(log.channels.len(), CHANNELS.len());
    for (channel, expected) in log.channels.iter().zip(CHANNELS) {
        assert_eq!(
            (channel.name.as_str(), channel.unit.as_str()),
            (expected.name, expected.unit)
        );
        assert_eq!(channel.values.len(), 1001);
    }
    // Tick 0 is the initial state: at rest 10 m up, spinning.
    let first: Vec<f64> = log.channels.iter().map(|c| c.values[0].0).collect();
    assert_eq!(first[0..4], [0.0, 0.0, 0.0, -10.0]);
    assert_eq!(first[11..14], [0.4, -0.3, 1.5]);
    assert_eq!(log.channels[0].values[1000], Sample(10.0));
}

#[test]
fn fingerprints_are_sixteen_hexadecimal_digits() {
    assert_eq!(hex(0xba), "00000000000000ba");
    assert_eq!(hex(u64::MAX), "ffffffffffffffff");
}

#[test]
fn another_seed_records_another_run() {
    let other = record_reference(MASTER_SEED + 1).expect("runs");
    let reference = recorded().expect("runs");
    assert_eq!(other.seed, MASTER_SEED + 1);
    assert_ne!(other.final_hash, reference.final_hash);
}

#[test]
fn a_replay_reproduces_its_record_to_the_bit() {
    let report = replay_after(|_| {}).expect("records").expect("replays");
    assert_eq!(
        report,
        ReplayReport {
            ticks: TICKS,
            checkpoints: 11,
            samples: 14 * 1001,
            divergence: None,
        }
    );
}

#[test]
fn a_changed_input_is_found_at_the_first_sample_after_it() {
    let divergence = divergence_after(|log| log.inputs[1].thrust_n *= 1.0 + 1e-9);
    assert!(
        matches!(&divergence, Some(Divergence::Sample { tick: 510, channel, .. }) if channel == "north"),
        "{divergence:?}"
    );
}

#[test]
fn a_changed_sample_is_found_at_its_tick() {
    let reference = recorded().expect("runs");
    let divergence = divergence_after(|log| log.channels[3].values[700] = Sample(1.5));
    let expected = Divergence::Sample {
        tick: 7000,
        channel: "down".to_owned(),
        recorded: 1.5,
        replayed: reference.channels[3].values[700].0,
    };
    assert_eq!(divergence, Some(expected));
}

#[test]
fn a_nan_sample_differs_from_a_number() {
    let divergence = divergence_after(|log| log.channels[13].values[2] = Sample(f64::NAN));
    assert_eq!(divergence.map(|d| d.tick()), Some(20));
}

#[test]
fn a_changed_checkpoint_or_final_fingerprint_is_found() {
    let checkpoint = divergence_after(|log| log.checkpoints[3].hash = hex(0x0123_4567_89ab_cdef));
    assert!(
        matches!(
            checkpoint,
            Some(Divergence::Checkpoint {
                tick: 3000,
                recorded: 0x0123_4567_89ab_cdef,
                ..
            })
        ),
        "{checkpoint:?}"
    );
    let last = divergence_after(|log| log.final_hash = hex(7));
    assert!(
        matches!(
            last,
            Some(Divergence::Final {
                tick: TICKS,
                recorded: 7,
                ..
            })
        ),
        "{last:?}"
    );
}

#[test]
fn the_earliest_difference_is_reported_and_a_sample_wins_a_tie() {
    let checkpoint_first = divergence_after(|log| {
        log.channels[1].values[900] = Sample(0.0);
        log.checkpoints[3].hash = hex(1);
    });
    assert!(
        matches!(
            checkpoint_first,
            Some(Divergence::Checkpoint { tick: 3000, .. })
        ),
        "{checkpoint_first:?}"
    );
    let tie = divergence_after(|log| {
        log.channels[5].values[100] = Sample(0.0);
        log.checkpoints[1].hash = hex(1);
    });
    assert!(
        matches!(tie, Some(Divergence::Sample { tick: 1000, .. })),
        "{tie:?}"
    );
    let at_the_end = divergence_after(|log| {
        log.channels[2].values[1000] = Sample(0.0);
        log.final_hash = hex(1);
    });
    assert!(
        matches!(at_the_end, Some(Divergence::Sample { tick: 10_000, .. })),
        "{at_the_end:?}"
    );
}

#[test]
fn divergences_explain_themselves() {
    let sample = Divergence::Sample {
        tick: 510,
        channel: "north".to_owned(),
        recorded: 0.25,
        replayed: f64::NAN,
    };
    assert_eq!(
        sample.to_string(),
        "tick 510: north was 0.25 and is NaN on replay"
    );
    let checkpoint = Divergence::Checkpoint {
        tick: 3000,
        recorded: 1,
        replayed: 0xff,
    };
    assert_eq!(
        checkpoint.to_string(),
        "tick 3000: the checkpoint fingerprint was 0000000000000001 and is 00000000000000ff on \
         replay"
    );
    let last = Divergence::Final {
        tick: 10_000,
        recorded: 2,
        replayed: 3,
    };
    assert_eq!(
        last.to_string(),
        "tick 10000: the final fingerprint was 0000000000000002 and is 0000000000000003 on replay"
    );
    assert_eq!(last.tick(), 10_000);
    assert_eq!(checkpoint.tick(), 3000);
}

#[test]
fn a_run_this_build_cannot_fly_is_refused_with_the_field_at_fault() {
    let cases: [(&str, &str, Change); 5] = [
        ("format_version", "1; found 2", |log| log.format_version = 2),
        ("scenario", "reference; found hover", |log| {
            log.scenario = "hover".to_owned();
        }),
        ("physics_hz", "1000; found 500", |log| log.physics_hz = 500),
        ("ticks", "10000; found 9999", |log| log.ticks = 9999),
        ("sample_hz", "100; found 50", |log| log.sample_hz = 50),
    ];
    for (field, values, change) in cases {
        assert_eq!(
            refusal_after(change),
            Some(refusal(field, &format!("this build replays only {values}")))
        );
    }
}

#[test]
fn other_channels_are_refused() {
    assert_eq!(
        refusal_after(|log| {
            log.channels.pop();
        }),
        Some(refusal(
            "channels",
            "the reference scenario logs 14 channels; found 13"
        ))
    );
    assert_eq!(
        refusal_after(|log| log.channels[4].name = "speed".to_owned()),
        Some(refusal(
            "channels[4]",
            "expected channel `velocity_north` with its unit here"
        ))
    );
    assert_eq!(
        refusal_after(|log| log.channels[0].unit = "ms".to_owned()),
        Some(refusal(
            "channels[0]",
            "expected channel `time` with its unit here"
        ))
    );
}

#[test]
fn a_truncated_record_is_refused_rather_than_called_a_divergence() {
    assert_eq!(
        refusal_after(|log| {
            log.checkpoints.pop();
        }),
        Some(refusal(
            "checkpoints",
            "the run takes 11 checkpoints; found 10"
        ))
    );
    assert_eq!(
        refusal_after(|log| {
            log.channels[6].values.pop();
        }),
        Some(refusal(
            "channels[6].values",
            "the run logs 1001 samples per channel"
        ))
    );
}

#[test]
fn malformed_fingerprints_are_refused() {
    for bad in [
        "abc",
        "0123456789abcdef0",
        "+123456789abcdef",
        "0123456789abcdeg",
    ] {
        assert_eq!(
            refusal_after(|log| log.checkpoints[2].hash = bad.to_owned()),
            Some(refusal(
                "checkpoints[2].hash",
                &format!("a fingerprint is 16 hexadecimal digits; found \"{bad}\"")
            ))
        );
    }
    assert_eq!(
        refusal_after(|log| log.final_hash = "x".to_owned()).map(|(field, _)| field),
        Some("final_hash".to_owned())
    );
    let upper = divergence_after(|log| log.final_hash = log.final_hash.to_uppercase());
    assert_eq!(upper, None, "upper-case digits are the same fingerprint");
}

#[test]
fn inputs_out_of_order_are_refused() {
    let (field, reason) = refusal_after(|log| log.inputs[5].tick = 100).expect("refused");
    assert_eq!(field, "inputs[5].tick");
    assert!(
        reason.contains("tick 100 comes before tick 2000"),
        "{reason}"
    );
}

#[test]
fn replay_errors_explain_themselves() {
    let error = replay_after(|log| log.ticks = 1)
        .expect("records")
        .expect_err("refused");
    assert_eq!(
        error.to_string(),
        "run.json: ticks: this build replays only 10000; found 1"
    );
    let scenario = ReplayError::Scenario(ScenarioError::Log(LogError::Full { capacity: 3 }));
    assert_eq!(
        scenario.to_string(),
        "the scenario's state log is too small: the log is full at 3 rows; size it for the whole \
         run"
    );
}
