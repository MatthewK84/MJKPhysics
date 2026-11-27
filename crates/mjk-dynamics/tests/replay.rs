//! Determinism and replay (#14): identical checkpoint hashes across runs, recorded inputs that
//! replay to the same final hash, and records that fail loudly instead of growing.

use mjk_dynamics::mass::MassProperties;
use mjk_dynamics::{
    Checkpoint, Checkpoints, Clock, InputEvent, InputLog, InputModel, LoadModel, Loads, Model,
    Rate, RecordError, Replay, RigidBodyCoordinates, RigidBodyState, RigidBodySystem, Rkmk4, World,
};
use mjk_math::{
    Body, Mat3, Rotation, StateHash, StateHasher, Vec3, Xoshiro256StarStar, state_hash,
};

const PHYSICS_HZ: u32 = 1000;
const TICKS: u64 = 2000;

/// A body moment set by the pilot and held between inputs.
#[derive(Clone, Copy, Debug)]
struct HeldMoment {
    moment_b: Vec3<Body>,
}

impl LoadModel for HeldMoment {
    fn loads(&self, _: f64, _: &RigidBodyState, _: &MassProperties) -> Loads {
        Loads {
            force_w: Vec3::zero(),
            moment_b: self.moment_b,
        }
    }
}

/// A body flown by pilot moment commands, which counts the commands it has taken.
#[derive(Clone)]
struct Piloted {
    system: RigidBodySystem<HeldMoment>,
    commands: u64,
}

impl Model for Piloted {
    type System = RigidBodySystem<HeldMoment>;
    fn system(&self) -> &Self::System {
        &self.system
    }
    fn update(&mut self, _: &Clock, _: &RigidBodyCoordinates) {}
}

impl InputModel for Piloted {
    type Input = Vec3<Body>;
    fn apply(&mut self, input: &Vec3<Body>) {
        self.system.loads.moment_b = *input;
        self.commands += 1;
    }
}

impl StateHash for Piloted {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.system.loads.moment_b.hash_state(hasher);
        self.commands.hash_state(hasher);
    }
}

type PilotedWorld = World<Piloted, Rkmk4>;

fn world() -> Option<PilotedWorld> {
    let inertia = Mat3::from_rows([[0.02, 0.001, 0.0], [0.001, 0.03, 0.002], [0.0, 0.002, 0.04]]);
    let model = Piloted {
        system: RigidBodySystem {
            props: MassProperties::new(1.5, Vec3::zero(), inertia).ok()?,
            loads: HeldMoment {
                moment_b: Vec3::zero(),
            },
        },
        commands: 0,
    };
    let mut start = RigidBodyState::at_rest(Vec3::zero(), Rotation::identity());
    start.omega_rad_s = Vec3::new(0.3, -1.2, 2.0);
    World::new(model, Rkmk4, RigidBodyCoordinates::from(start), PHYSICS_HZ).ok()
}

/// Ten checkpoints a second over the run, with room for every one.
fn checkpoints() -> Option<Checkpoints> {
    let rate = Rate::new(PHYSICS_HZ, 10).ok()?;
    Some(Checkpoints::new(rate, 21))
}

/// A random pilot command every quarter second, recorded; checkpoints throughout.
fn live_run(seed: u64) -> Option<(InputLog<Vec3<Body>>, Checkpoints, u64)> {
    let mut world = world()?;
    let mut pilot = Xoshiro256StarStar::named(seed, "pilot");
    let mut log = InputLog::with_capacity(8);
    let mut record = checkpoints()?;
    for _ in 0..TICKS {
        if world.clock().tick().is_multiple_of(250) {
            let command = Vec3::new(pilot.next_gaussian(), pilot.next_gaussian(), 0.0) * 0.01;
            world.apply(command, &mut log).ok()?;
        }
        world.checkpoint(&mut record).ok()?;
        world.step();
    }
    world.checkpoint(&mut record).ok()?;
    Some((log, record, world.state_hash()))
}

/// The run that `log` describes, replayed into a new world.
fn replayed_run(log: &InputLog<Vec3<Body>>) -> Option<(Checkpoints, u64, bool)> {
    let mut world = world()?;
    let mut replay = Replay::new(log);
    let mut record = checkpoints()?;
    for _ in 0..TICKS {
        world.replay_due(&mut replay);
        world.checkpoint(&mut record).ok()?;
        world.step();
    }
    world.checkpoint(&mut record).ok()?;
    Some((record, world.state_hash(), replay.is_finished()))
}

// ---------------------------------------------------------------- acceptance criteria

#[test]
fn two_runs_give_identical_hashes_at_every_checkpoint() {
    let (_, first, first_hash) = live_run(11).expect("runs");
    let (_, second, second_hash) = live_run(11).expect("runs");
    assert_eq!(first.records().len(), 21);
    assert_eq!(first.records(), second.records());
    assert_eq!(first_hash, second_hash);
}

#[test]
fn replaying_the_input_log_reproduces_every_checkpoint_and_the_final_hash() {
    let (log, live, live_hash) = live_run(11).expect("runs");
    assert_eq!(log.events().len(), 8);
    let (replayed, replayed_hash, finished) = replayed_run(&log).expect("runs");
    assert!(finished);
    assert_eq!(replayed.records(), live.records());
    assert_eq!(replayed_hash, live_hash);
}

#[test]
fn changing_one_input_changes_every_hash_from_its_tick_on() {
    // One part in 10⁹ of one command. A change of one ulp can be too small to survive: on a
    // 0.01 N·m moment held for 0.25 s it moves ω by about 2e-17 rad/s, below the resolution of
    // an ω near 1 rad/s, so rounding absorbs it once the next command replaces it.
    let (log, live, live_hash) = live_run(11).expect("runs");
    let mut altered = InputLog::with_capacity(8);
    for event in log.events() {
        let mut input = event.input;
        if event.tick == 1000 {
            input = Vec3::new(input.x * (1.0 + 1e-9), input.y, input.z);
        }
        altered.record(event.tick, input).expect("room");
    }
    let (replayed, replayed_hash, _) = replayed_run(&altered).expect("runs");
    assert_ne!(replayed_hash, live_hash);
    for (a, b) in replayed.records().iter().zip(live.records()) {
        assert_eq!(a.tick, b.tick);
        assert_eq!(a.hash == b.hash, a.tick < 1000, "tick {}", a.tick);
    }
}

#[test]
fn a_different_seed_gives_different_hashes() {
    let (_, first, first_hash) = live_run(11).expect("runs");
    let (_, other, other_hash) = live_run(12).expect("runs");
    assert_ne!(first_hash, other_hash);
    // Tick 0 is hashed after the first command is applied, so even it differs.
    for (a, b) in first.records().iter().zip(other.records()) {
        assert_ne!(a.hash, b.hash, "tick {}", a.tick);
    }
}

// ---------------------------------------------------------------- the state hash

#[test]
fn the_world_hash_covers_the_clock_the_state_and_the_model() {
    let mut world = world().expect("physical");
    let start = world.state_hash();
    let mut expected = StateHasher::new();
    world.clock().hash_state(&mut expected);
    world.current().hash_state(&mut expected);
    world.model().hash_state(&mut expected);
    assert_eq!(start, expected.finish());

    world.model_mut().commands = 1;
    assert_ne!(world.state_hash(), start, "model state");
    world.model_mut().commands = 0;
    world.step();
    assert_ne!(world.state_hash(), start, "a step");
    assert_eq!(world.clone().state_hash(), world.state_hash());
}

#[test]
fn the_clock_hashes_its_tick_and_rate() {
    let base = Clock::at(1000, 5).expect("nonzero");
    let mut expected = StateHasher::new();
    5_u64.hash_state(&mut expected);
    1000_u32.hash_state(&mut expected);
    assert_eq!(state_hash(&base), expected.finish());
    assert_ne!(
        state_hash(&Clock::at(1000, 6).expect("nonzero")),
        state_hash(&base)
    );
    assert_ne!(
        state_hash(&Clock::at(500, 5).expect("nonzero")),
        state_hash(&base)
    );
}

#[test]
fn a_state_and_its_coordinates_hash_alike_field_by_field() {
    let mut state = RigidBodyState::at_rest(Vec3::new(1.0, 2.0, 3.0), Rotation::identity());
    state.velocity_m_s = Vec3::new(4.0, 5.0, 6.0);
    state.omega_rad_s = Vec3::new(7.0, 8.0, 9.0);
    let mut expected = StateHasher::new();
    for value in [
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 1.0, 0.0, 0.0, 0.0, 7.0, 8.0, 9.0,
    ] {
        expected.write_f64(value);
    }
    assert_eq!(state_hash(&state), expected.finish());
    assert_eq!(
        state_hash(&RigidBodyCoordinates::from(state)),
        expected.finish()
    );
}

// ---------------------------------------------------------------- records

#[test]
fn checkpoints_are_taken_only_when_due() {
    let mut world = world().expect("physical");
    let mut record = checkpoints().expect("divides 1 kHz");
    assert_eq!(world.checkpoint(&mut record), Ok(true));
    world.step();
    assert_eq!(world.checkpoint(&mut record), Ok(false));
    assert_eq!(
        record.records(),
        [Checkpoint {
            tick: 0,
            hash: record.records().first().map_or(0, |c| c.hash),
        }]
    );
    assert!(record.is_due(100) && !record.is_due(150));
}

#[test]
fn a_checkpoint_must_be_later_than_the_last_and_fit() {
    let rate = Rate::new(PHYSICS_HZ, PHYSICS_HZ).expect("valid");
    let mut record = Checkpoints::new(rate, 2);
    assert_eq!(record.record(3, 30), Ok(()));
    let out_of_order = Err(RecordError::OutOfOrder {
        tick: 3,
        last_tick: 3,
    });
    assert_eq!(record.record(3, 31), out_of_order);
    assert_eq!(record.record(4, 40), Ok(()));
    assert_eq!(record.record(5, 50), Err(RecordError::Full { capacity: 2 }));
    assert_eq!(record.records().len(), 2);
}

#[test]
fn inputs_may_share_a_tick_but_not_go_back_in_time_or_overflow() {
    let mut log = InputLog::with_capacity(3);
    assert_eq!(log.record(2, 'a'), Ok(()));
    assert_eq!(log.record(2, 'b'), Ok(()));
    let back = Err(RecordError::OutOfOrder {
        tick: 1,
        last_tick: 2,
    });
    assert_eq!(log.record(1, 'c'), back);
    assert_eq!(log.record(4, 'd'), Ok(()));
    assert_eq!(log.record(5, 'e'), Err(RecordError::Full { capacity: 3 }));
    let inputs: Vec<char> = log.events().iter().map(|e| e.input).collect();
    assert_eq!(inputs, ['a', 'b', 'd']);
}

#[test]
fn a_replay_hands_back_each_input_once_on_its_tick() {
    let mut log = InputLog::with_capacity(4);
    for (tick, input) in [(0, 'a'), (2, 'b'), (2, 'c'), (5, 'd')] {
        log.record(tick, input).expect("room");
    }
    let mut replay = Replay::new(&log);
    let event = |tick, input| InputEvent { tick, input };
    assert_eq!(replay.due(0), [event(0, 'a')]);
    assert_eq!(replay.due(1), []);
    assert_eq!(replay.due(2), [event(2, 'b'), event(2, 'c')]);
    assert_eq!(replay.due(2), []);
    assert!(!replay.is_finished());
    // A late call still hands back what is overdue, in order.
    assert_eq!(replay.due(9), [event(5, 'd')]);
    assert!(replay.is_finished());
    assert_eq!(replay.due(10), []);
}

#[test]
fn an_input_that_cannot_be_recorded_is_not_applied() {
    let mut world = world().expect("physical");
    let mut log = InputLog::with_capacity(1);
    let command = Vec3::new(0.01, 0.0, 0.0);
    assert_eq!(world.apply(command, &mut log), Ok(()));
    assert_eq!(
        world.apply(command * 2.0, &mut log),
        Err(RecordError::Full { capacity: 1 })
    );
    assert_eq!(world.model().commands, 1);
    assert_eq!(world.model().system.loads.moment_b, command);
}

#[test]
fn replay_applies_the_due_inputs_and_counts_them() {
    let mut world = world().expect("physical");
    let mut log = InputLog::with_capacity(2);
    log.record(0, Vec3::new(0.01, 0.0, 0.0)).expect("room");
    log.record(0, Vec3::new(0.0, 0.02, 0.0)).expect("room");
    let mut replay = Replay::new(&log);
    assert_eq!(world.replay_due(&mut replay), 2);
    assert_eq!(world.model().commands, 2);
    assert_eq!(
        world.model().system.loads.moment_b,
        Vec3::new(0.0, 0.02, 0.0)
    );
    assert_eq!(world.replay_due(&mut replay), 0);
}

#[test]
fn record_errors_explain_themselves() {
    assert_eq!(
        RecordError::Full { capacity: 4 }.to_string(),
        "the record is full at 4 entries; size it for the whole run"
    );
    assert_eq!(
        RecordError::OutOfOrder {
            tick: 3,
            last_tick: 7
        }
        .to_string(),
        "tick 3 comes before tick 7, the last one recorded"
    );
}
