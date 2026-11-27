//! The world loop and its scheduler (#13): rates, order within a tick, zero-order hold, the
//! real-time accumulator, exact time and determinism.

use mjk_dynamics::mass::MassProperties;
use mjk_dynamics::{
    Clock, LoadModel, Loads, Model, Rate, RateError, RigidBodyCoordinates, RigidBodyState,
    RigidBodySystem, Rk4, Rkmk4, World,
};
use mjk_math::{Mat3, Rotation, Vec3};

const PHYSICS_HZ: u32 = 1000;

/// A vertical force set by a controller and held between its updates.
#[derive(Clone, Copy, Debug, Default)]
struct HeldForce {
    force_n: f64,
}

impl LoadModel for HeldForce {
    fn loads(&self, _: f64, _: &RigidBodyState, _: &MassProperties) -> Loads {
        Loads {
            force_w: Vec3::new(0.0, 0.0, self.force_n),
            moment_b: Vec3::zero(),
        }
    }
}

fn system() -> Option<RigidBodySystem<HeldForce>> {
    let inertia = Mat3::from_rows([[0.02, 0.001, 0.0], [0.001, 0.03, 0.002], [0.0, 0.002, 0.04]]);
    let props = MassProperties::new(1.0, Vec3::zero(), inertia).ok()?;
    Some(RigidBodySystem {
        props,
        loads: HeldForce::default(),
    })
}

fn at_rest() -> RigidBodyCoordinates {
    RigidBodyCoordinates::from(RigidBodyState::at_rest(Vec3::zero(), Rotation::identity()))
}

/// `n` as an `f64`, for small tick counts.
fn small(n: u64) -> f64 {
    f64::from(u32::try_from(n).unwrap_or(u32::MAX))
}

// ---------------------------------------------------------------- rates

/// Four subsystems that count their runs and note the first and last tick of each.
struct Counting {
    system: RigidBodySystem<HeldForce>,
    runs: [(Rate, u32, Option<u64>, u64); 4],
}

impl Model for Counting {
    type System = RigidBodySystem<HeldForce>;
    fn system(&self) -> &Self::System {
        &self.system
    }
    fn update(&mut self, clock: &Clock, _: &RigidBodyCoordinates) {
        for (rate, count, first, last) in &mut self.runs {
            if rate.is_due(clock.tick()) {
                *count += 1;
                first.get_or_insert(clock.tick());
                *last = clock.tick();
            }
        }
    }
}

#[test]
fn every_subsystem_runs_exactly_at_its_rate() {
    let rates_hz = [500, 250, 50, 10];
    let runs = rates_hz.map(|hz| {
        let rate = Rate::new(PHYSICS_HZ, hz).expect("divides 1 kHz");
        (rate, 0, None, 0)
    });
    let model = Counting {
        system: system().expect("physical"),
        runs,
    };
    let mut world = World::new(model, Rk4, at_rest(), PHYSICS_HZ).expect("valid rate");
    for _ in 0..10_000 {
        world.step();
    }
    assert_eq!(world.clock().tick(), 10_000);
    for ((rate, count, first, last), hz) in world.model().runs.iter().zip(rates_hz) {
        // 10 s at `hz`: exactly 10·hz runs, starting at tick 0, one every `divisor` ticks.
        assert_eq!(*count, 10 * hz, "{hz} Hz");
        assert_eq!(*first, Some(0), "{hz} Hz");
        assert_eq!(*last, 10_000 - u64::from(rate.divisor()), "{hz} Hz");
    }
}

#[test]
fn rates_must_divide_the_physics_rate() {
    assert_eq!(
        Rate::new(1000, 300),
        Err(RateError::NotADivisor {
            rate_hz: 300,
            physics_hz: 1000
        })
    );
    assert_eq!(
        Rate::new(1000, 2000),
        Err(RateError::FasterThanPhysics {
            rate_hz: 2000,
            physics_hz: 1000
        })
    );
    assert_eq!(Rate::new(1000, 0), Err(RateError::Zero));
    assert_eq!(Rate::new(0, 10), Err(RateError::Zero));
    assert_eq!(Clock::new(0), Err(RateError::Zero));
    assert_eq!(Rate::new(1000, 1000).map(|r| r.divisor()), Ok(1));
    assert_eq!(Rate::new(1000, 1).map(|r| r.divisor()), Ok(1000));
    let messages = [
        (RateError::Zero, "a rate must be at least 1 Hz"),
        (
            RateError::FasterThanPhysics {
                rate_hz: 2000,
                physics_hz: 1000,
            },
            "2000 Hz is faster than the 1000 Hz physics rate",
        ),
        (
            RateError::NotADivisor {
                rate_hz: 300,
                physics_hz: 1000,
            },
            "300 Hz does not divide the 1000 Hz physics rate; subsystems run every whole number \
             of physics steps",
        ),
    ];
    for (error, text) in messages {
        assert_eq!(error.to_string(), text);
    }
}

// ---------------------------------------------------------------- order and hold

/// A sensor and a controller at different rates. The controller commands a force equal to the
/// last sensed tick, and both write to an event log so the order within a tick is visible.
struct Loop {
    system: RigidBodySystem<HeldForce>,
    sensor: Rate,
    controller: Rate,
    sensed_tick: u64,
    log: [u8; 4],
    logged: usize,
    after_ticks: u64,
    stop_on_ground: bool,
}

impl Loop {
    fn new(sensor_hz: u32, controller_hz: u32) -> Option<Self> {
        Some(Self {
            system: system()?,
            sensor: Rate::new(PHYSICS_HZ, sensor_hz).ok()?,
            controller: Rate::new(PHYSICS_HZ, controller_hz).ok()?,
            sensed_tick: 0,
            log: [0; 4],
            logged: 0,
            after_ticks: 0,
            stop_on_ground: false,
        })
    }

    fn note(&mut self, event: u8) {
        if let Some(slot) = self.log.get_mut(self.logged) {
            *slot = event;
            self.logged += 1;
        }
    }
}

impl Model for Loop {
    type System = RigidBodySystem<HeldForce>;
    fn system(&self) -> &Self::System {
        &self.system
    }
    fn update(&mut self, clock: &Clock, _: &RigidBodyCoordinates) {
        if self.sensor.is_due(clock.tick()) {
            self.sensed_tick = clock.tick();
            self.note(b'S');
        }
        if self.controller.is_due(clock.tick()) {
            self.system.loads.force_n = small(self.sensed_tick);
            self.note(b'C');
        }
    }
    fn after_step(&mut self, clock: &Clock, y: &mut RigidBodyCoordinates) {
        self.after_ticks = clock.tick();
        if self.stop_on_ground {
            y.velocity = Vec3::zero();
        }
    }
}

#[test]
fn sensors_run_before_controllers_within_a_tick() {
    let model = Loop::new(500, 250).expect("valid");
    let mut world = World::new(model, Rk4, at_rest(), PHYSICS_HZ).expect("valid rate");
    world.step();
    world.step();
    world.step();
    // Tick 0: sensor then controller. Tick 1: nothing. Tick 2: the sensor only.
    assert_eq!(&world.model().log[..world.model().logged], b"SCS");
}

#[test]
fn commands_are_held_between_controller_updates() {
    // The controller runs at 100 Hz and commands F = (sensed tick) N, so the force is 0, 10 and
    // 20 N over ticks 0–9, 10–19 and 20–29. With m = 1 kg and RK4 exact for constant force,
    // v_z = (0 + 10 + 20)·10·0.001 = 0.3 m/s; refreshing the force every tick would give 0.435.
    let model = Loop::new(1000, 100).expect("valid");
    let mut world = World::new(model, Rk4, at_rest(), PHYSICS_HZ).expect("valid rate");
    for _ in 0..30 {
        world.step();
    }
    assert!(
        (world.current().velocity.z - 0.3).abs() < 1e-14,
        "{:?}",
        world.current()
    );
}

#[test]
fn the_after_step_hook_sees_the_new_tick_and_can_change_the_state() {
    let mut model = Loop::new(1000, 1000).expect("valid");
    model.stop_on_ground = true;
    let mut world = World::new(model, Rk4, at_rest(), PHYSICS_HZ).expect("valid rate");
    for _ in 0..5 {
        world.step();
    }
    assert_eq!(world.model().after_ticks, 5);
    assert_eq!(world.current().velocity, Vec3::zero());
    assert!(
        world.current().position.z > 0.0,
        "the force still moved the body"
    );
}

#[test]
fn the_previous_state_is_one_step_behind() {
    let model = Loop::new(1000, 1000).expect("valid");
    let mut moving = at_rest();
    moving.velocity = Vec3::new(1.0, 0.0, 0.0);
    let mut world = World::new(model, Rk4, moving, PHYSICS_HZ).expect("valid rate");
    assert_eq!(world.previous(), world.current());
    world.step();
    let after_one = *world.current();
    world.step();
    assert_eq!(*world.previous(), after_one);
    assert!(world.current().position.x > after_one.position.x);
    world.model_mut().stop_on_ground = true;
    world.step();
    assert_eq!(world.current().velocity, Vec3::zero());
}

// ---------------------------------------------------------------- real time

fn idle_world() -> Option<World<Loop, Rk4>> {
    let model = Loop::new(1000, 1000)?;
    World::new(model, Rk4, at_rest(), PHYSICS_HZ).ok()
}

#[test]
fn advance_carries_the_remainder_between_calls() {
    let mut world = idle_world().expect("valid");
    let first = world.advance(0.0035);
    assert_eq!(first.steps, 3);
    assert!((first.alpha - 0.5).abs() < 1e-9 && first.dropped_s.abs() < 1e-15);
    let second = world.advance(0.0007);
    assert_eq!(second.steps, 1);
    assert!((second.alpha - 0.2).abs() < 1e-9);
    assert_eq!(world.clock().tick(), 4);
    // Exactly one step of real time, from an empty accumulator, is one step with nothing left.
    let mut fresh = idle_world().expect("valid");
    let exact = fresh.advance(0.001);
    assert_eq!(exact.steps, 1);
    assert!(
        exact.alpha.abs() < 1e-15 && exact.dropped_s.abs() < 1e-15,
        "{exact:?}"
    );
}

#[test]
fn advance_drops_time_it_cannot_catch_up_on() {
    let mut world = idle_world().expect("valid").with_max_steps_per_advance(5);
    let behind = world.advance(1.0);
    assert_eq!(behind.steps, 5);
    // Everything not stepped is dropped, and drawing starts again from the current state.
    assert!((behind.dropped_s - 0.995).abs() < 1e-9, "{behind:?}");
    assert!(behind.alpha.abs() < 1e-15, "{behind:?}");
    let next = world.advance(0.0015);
    assert_eq!(next.steps, 1);
    assert!((next.alpha - 0.5).abs() < 1e-9 && next.dropped_s.abs() < 1e-15);
    assert_eq!(
        idle_world()
            .expect("valid")
            .with_max_steps_per_advance(0)
            .advance(1.0)
            .steps,
        1
    );
    // The default cap is a quarter of a second of steps.
    assert_eq!(idle_world().expect("valid").advance(1.0).steps, 250);
}

#[test]
fn bad_real_time_intervals_take_no_steps() {
    let mut world = idle_world().expect("valid");
    for interval in [-1.0, f64::NAN, f64::INFINITY, 0.0] {
        let advance = world.advance(interval);
        assert_eq!(advance.steps, 0, "{interval}");
        assert!(advance.alpha.abs() < 1e-15 && advance.dropped_s.abs() < 1e-15);
    }
    assert_eq!(world.clock().tick(), 0);
}

// ---------------------------------------------------------------- time and determinism

#[test]
fn time_is_derived_from_the_tick_never_accumulated() {
    let clock = Clock::at(PHYSICS_HZ, 123_456_789).expect("valid rate");
    // 123 456 789 / 1000, correctly rounded, is the double nearest 123456.789.
    assert_eq!(clock.time_s().to_bits(), 123_456.789_f64.to_bits());
    assert_eq!(
        Clock::at(PHYSICS_HZ, 10_000_000)
            .expect("valid")
            .time_s()
            .to_bits(),
        10_000.0_f64.to_bits()
    );
    let late = Clock::at(PHYSICS_HZ, (1 << 52) + 1).expect("valid rate");
    assert_eq!(
        late.time_s().to_bits(),
        ((4_503_599_627_370_497.0_f64) / 1000.0).to_bits()
    );
    assert!((Clock::new(PHYSICS_HZ).expect("valid").dt_s() - 0.001).abs() < 1e-18);
    assert_eq!(Clock::new(500).expect("valid").rate_hz(), 500);
    // Summing 0.001 ten million times instead drifts by about 1e-6 s.
    let summed = (0..10_000_000).fold(0.0_f64, |t, _| t + 0.001);
    assert!((summed - 10_000.0).abs() > 1e-9);
}

#[test]
fn two_runs_are_identical_to_the_bit() {
    let run = || {
        let mut model = Loop::new(250, 50).expect("valid");
        model.system.loads.force_n = 1.0;
        let mut y0 = at_rest();
        y0.angular_velocity = Vec3::new(0.2, 6.0, 0.3);
        let mut world = World::new(model, Rkmk4, y0, PHYSICS_HZ).expect("valid rate");
        for _ in 0..2000 {
            world.step();
        }
        *world.current()
    };
    let (a, b) = (run(), run());
    assert_eq!(
        a.position.to_array().map(f64::to_bits),
        b.position.to_array().map(f64::to_bits)
    );
    assert_eq!(a.attitude.w.to_bits(), b.attitude.w.to_bits());
    assert_eq!(
        a.angular_velocity.to_array().map(f64::to_bits),
        b.angular_velocity.to_array().map(f64::to_bits)
    );
}
