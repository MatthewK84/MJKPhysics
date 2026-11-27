//! A world step allocates nothing (#13): 10⁵ steps with RK4 and with RKMK4, and real-time
//! advances, under a counting global allocator. Nor do checkpoints, recorded inputs and their
//! replay (#14), once their records are sized.
//!
//! This test runs without the test harness (`harness = false`), so no other thread allocates
//! while it measures; `main` panics through an assertion on any allocation.

use mjk_alloc_counter::{CountingAllocator, allocations};
use mjk_dynamics::mass::MassProperties;
use mjk_dynamics::{
    Checkpoints, Clock, InputLog, InputModel, Integrator, LoadModel, Loads, Model, Rate, Replay,
    RigidBodyCoordinates, RigidBodyState, RigidBodySystem, Rk4, Rkmk4, World,
};
use mjk_math::{Mat3, Rotation, StateHash, StateHasher, Vec3};

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const PHYSICS_HZ: u32 = 1000;
const STEPS: u32 = 100_000;

#[derive(Clone, Copy, Debug, Default)]
struct HeldForce {
    force_n: f64,
}

impl LoadModel for HeldForce {
    fn loads(&self, _: f64, _: &RigidBodyState, _: &MassProperties) -> Loads {
        Loads {
            force_w: Vec3::new(0.0, 0.0, self.force_n),
            moment_b: Vec3::new(0.001, 0.0, -0.002),
        }
    }
}

/// A sensor at 250 Hz feeding a controller at 500 Hz, with samples kept in a fixed buffer.
#[derive(Clone)]
struct Vehicle {
    system: RigidBodySystem<HeldForce>,
    sensor: Rate,
    controller: Rate,
    samples: [f64; 16],
    next: usize,
    trim_n: f64,
}

impl Model for Vehicle {
    type System = RigidBodySystem<HeldForce>;
    fn system(&self) -> &Self::System {
        &self.system
    }
    fn update(&mut self, clock: &Clock, y: &RigidBodyCoordinates) {
        if self.sensor.is_due(clock.tick()) {
            if let Some(slot) = self.samples.get_mut(self.next % 16) {
                *slot = y.angular_velocity.z;
            }
            self.next += 1;
        }
        if self.controller.is_due(clock.tick()) {
            let mean = self.samples.iter().sum::<f64>() / 16.0;
            self.system.loads.force_n = self.trim_n - 0.1 * mean;
        }
    }
}

impl InputModel for Vehicle {
    type Input = f64;
    fn apply(&mut self, trim_n: &f64) {
        self.trim_n = *trim_n;
    }
}

impl StateHash for Vehicle {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.system.loads.force_n.hash_state(hasher);
        self.samples.hash_state(hasher);
        u64::try_from(self.next)
            .unwrap_or(u64::MAX)
            .hash_state(hasher);
        self.trim_n.hash_state(hasher);
    }
}

fn vehicle() -> Option<Vehicle> {
    let inertia = Mat3::from_rows([
        [0.021, 0.0004, -0.0006],
        [0.0004, 0.024, 0.0002],
        [-0.0006, 0.0002, 0.040],
    ]);
    Some(Vehicle {
        system: RigidBodySystem {
            props: MassProperties::new(2.0, Vec3::zero(), inertia).ok()?,
            loads: HeldForce::default(),
        },
        sensor: Rate::new(PHYSICS_HZ, 250).ok()?,
        controller: Rate::new(PHYSICS_HZ, 500).ok()?,
        samples: [0.0; 16],
        next: 0,
        trim_n: 0.0,
    })
}

fn start() -> RigidBodyCoordinates {
    let mut state = RigidBodyState::at_rest(Vec3::zero(), Rotation::identity());
    state.omega_rad_s = Vec3::new(0.2, 6.0, 0.3);
    RigidBodyCoordinates::from(state)
}

/// Allocations made while stepping a world built beforehand, and the real-time steps taken.
fn allocations_while_stepping<I: Integrator<RigidBodySystem<HeldForce>>>(
    integrator: I,
) -> Result<(u64, u32), String> {
    let model = vehicle().ok_or_else(|| "the test vehicle is not physical".to_owned())?;
    let mut world =
        World::new(model, integrator, start(), PHYSICS_HZ).map_err(|error| error.to_string())?;
    let before = allocations();
    for _ in 0..STEPS {
        world.step();
    }
    let mut advanced = 0;
    for _ in 0..1000 {
        advanced += world.advance(0.0101).steps;
    }
    Ok((allocations() - before, advanced))
}

/// Allocations made while a run records a checkpoint every 0.1 s and an input every 0.1 s, and
/// a second world replays it; and whether the replay reached the same state.
fn allocations_while_recording_and_replaying() -> Result<(u64, bool), String> {
    let not_physical = || "the test vehicle is not physical".to_owned();
    let mut live = World::new(
        vehicle().ok_or_else(not_physical)?,
        Rkmk4,
        start(),
        PHYSICS_HZ,
    )
    .map_err(|error| error.to_string())?;
    let mut replayed = live.clone();
    let rate = Rate::new(PHYSICS_HZ, 10).map_err(|error| error.to_string())?;
    let mut checkpoints = Checkpoints::new(rate, 1001);
    let mut log = InputLog::with_capacity(1000);
    let before = allocations();
    for tick in 0..u64::from(STEPS) {
        if tick.is_multiple_of(100) {
            let trim_n = if tick.is_multiple_of(200) { 0.5 } else { -0.5 };
            live.apply(trim_n, &mut log)
                .map_err(|error| error.to_string())?;
        }
        live.checkpoint(&mut checkpoints)
            .map_err(|error| error.to_string())?;
        live.step();
    }
    let mut replay = Replay::new(&log);
    for _ in 0..STEPS {
        replayed.replay_due(&mut replay);
        replayed.step();
    }
    let count = allocations() - before;
    Ok((count, replayed.state_hash() == live.state_hash()))
}

fn main() -> Result<(), String> {
    for (name, (count, advanced)) in [
        ("RK4", allocations_while_stepping(Rk4)?),
        ("RKMK4", allocations_while_stepping(Rkmk4)?),
    ] {
        if count != 0 || advanced < 10_000 {
            return Err(format!(
                "{name}: {count} allocations in {STEPS} steps and {advanced} real-time steps"
            ));
        }
    }
    let (count, same) = allocations_while_recording_and_replaying()?;
    if count != 0 || !same {
        return Err(format!(
            "recording and replay: {count} allocations; replay reached the same state: {same}"
        ));
    }
    println!("allocations: 0 in {STEPS} steps and 1000 real-time advances, with RK4 and RKMK4");
    println!("allocations: 0 while recording 1000 checkpoints and 1000 inputs, and replaying them");
    Ok(())
}
