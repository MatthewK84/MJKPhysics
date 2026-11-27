//! The determinism study of [#14](https://github.com/MatthewK84/MJKPhysics/issues/14): a
//! reference scenario whose state fingerprint the book publishes at every checkpoint, beside the
//! fingerprint of a replay of the run's recorded inputs.
//!
//! A tumbling 2 kg body is flown by a rate controller on a noisy gyro, with a noisy
//! accelerometer and random pilot commands: three named random streams, sensors at 500 Hz and
//! 250 Hz, a 250 Hz controller, and RKMK4 at 1 kHz for 10 s. Every operation is bit-exact, so
//! the published hashes are the same on every run and platform, natively and in WebAssembly.

use std::fmt;

use mjk_dynamics::mass::{MassError, MassProperties};
use mjk_dynamics::{
    Checkpoint, Checkpoints, Clock, InputLog, InputModel, LoadModel, Loads, Model, Rate, RateError,
    RecordError, Replay, RigidBodyCoordinates, RigidBodyState, RigidBodySystem, Rkmk4, World,
};
use mjk_math::{Body, Mat3, Rotation, StateHash, StateHasher, Vec3, Xoshiro256StarStar};

/// The master seed of the published run.
pub const MASTER_SEED: u64 = 14;

/// The gyro's random stream.
pub const GYRO_STREAM: &str = "gyro";

/// The accelerometer's random stream.
pub const ACCELEROMETER_STREAM: &str = "accelerometer";

/// The pilot's random stream.
pub const PILOT_STREAM: &str = "pilot";

/// Every random stream in the scenario. Their names must hash differently, or two would share a
/// sequence.
pub const STREAMS: [&str; 3] = [GYRO_STREAM, ACCELEROMETER_STREAM, PILOT_STREAM];

/// The physics rate.
pub const PHYSICS_HZ: u32 = 1000;

/// The length of the run in physics steps: 10 s.
pub const TICKS: u64 = 10_000;

/// Checkpoints per second. With tick 0 and the last tick there are eleven.
pub const CHECKPOINT_HZ: u32 = 1;

/// Checkpoints in a run.
const CHECKPOINTS: usize = 11;

/// Ticks between pilot commands: two a second, twenty in a run.
const PILOT_INTERVAL_TICKS: u64 = 500;

/// Pilot commands in a run.
const PILOT_COMMANDS: usize = 20;

/// Standard gravity.
const GRAVITY_M_S2: f64 = mjk_env::G0_M_S2;

/// Vehicle mass.
const MASS_KG: f64 = 2.0;

/// Rate-loop gain: the commanded moment is `k·J·(ω_cmd − ω_measured)`.
const RATE_GAIN_PER_S: f64 = 8.0;

/// A pilot command: body rates and collective thrust, held until the next command.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PilotCommand {
    /// Commanded body rates.
    pub rate_rad_s: Vec3<Body>,
    /// Commanded thrust along body −z.
    pub thrust_n: f64,
}

/// Why the scenario could not run. None of these happen with the published configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScenarioError {
    /// The vehicle's mass properties are not physical.
    Mass(MassError),
    /// A subsystem rate does not divide the physics rate.
    Rate(RateError),
    /// A checkpoint or input record was too small for the run.
    Record(RecordError),
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Mass(error) => write!(f, "the reference vehicle is not physical: {error}"),
            Self::Rate(error) => write!(f, "a scenario rate is not usable: {error}"),
            Self::Record(error) => write!(f, "a scenario record is too small: {error}"),
        }
    }
}

impl From<MassError> for ScenarioError {
    fn from(error: MassError) -> Self {
        Self::Mass(error)
    }
}

impl From<RateError> for ScenarioError {
    fn from(error: RateError) -> Self {
        Self::Rate(error)
    }
}

impl From<RecordError> for ScenarioError {
    fn from(error: RecordError) -> Self {
        Self::Record(error)
    }
}

/// Thrust along body −z and a body moment, both held, with gravity.
#[derive(Clone, Copy, Debug)]
struct Actuators {
    thrust_n: f64,
    moment_b: Vec3<Body>,
}

impl LoadModel for Actuators {
    fn loads(&self, _: f64, state: &RigidBodyState, props: &MassProperties) -> Loads {
        let thrust_w = state.q_wb.rotate(Vec3::new(0.0, 0.0, -self.thrust_n));
        let weight_w = Vec3::new(0.0, 0.0, props.mass_kg() * GRAVITY_M_S2);
        Loads {
            force_w: thrust_w + weight_w,
            moment_b: self.moment_b,
        }
    }
}

/// A three-axis sensor sampling at a fixed rate, with Gaussian noise from its own stream.
#[derive(Clone, Copy, Debug)]
struct NoisySensor {
    rate: Rate,
    sigma: f64,
    noise: Xoshiro256StarStar,
    reading: Vec3<Body>,
}

impl NoisySensor {
    fn new(seed: u64, name: &str, rate_hz: u32, sigma: f64) -> Result<Self, RateError> {
        Ok(Self {
            rate: Rate::new(PHYSICS_HZ, rate_hz)?,
            sigma,
            noise: Xoshiro256StarStar::named(seed, name),
            reading: Vec3::zero(),
        })
    }

    /// Samples `truth` plus noise on the ticks this sensor runs.
    fn sample(&mut self, tick: u64, truth: Vec3<Body>) {
        if !self.rate.is_due(tick) {
            return;
        }
        let noise = Vec3::new(
            self.noise.next_gaussian(),
            self.noise.next_gaussian(),
            self.noise.next_gaussian(),
        );
        self.reading = truth + noise * self.sigma;
    }
}

impl StateHash for NoisySensor {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.noise.hash_state(hasher);
        self.reading.hash_state(hasher);
    }
}

/// The reference vehicle: a gyro feeding a rate controller, an accelerometer, and the pilot's
/// held command.
#[derive(Clone, Copy, Debug)]
struct ReferenceVehicle {
    system: RigidBodySystem<Actuators>,
    gyro: NoisySensor,
    accelerometer: NoisySensor,
    controller: Rate,
    command: PilotCommand,
}

impl Model for ReferenceVehicle {
    type System = RigidBodySystem<Actuators>;

    fn system(&self) -> &Self::System {
        &self.system
    }

    fn update(&mut self, clock: &Clock, y: &RigidBodyCoordinates) {
        let tick = clock.tick();
        self.gyro.sample(tick, y.angular_velocity);
        // Thrust and gravity only: the specific force is the thrust per unit mass, body −z.
        let thrust_per_kg = self.system.loads.thrust_n / self.system.props.mass_kg();
        self.accelerometer
            .sample(tick, Vec3::new(0.0, 0.0, -thrust_per_kg));
        if self.controller.is_due(tick) {
            let error = self.command.rate_rad_s - self.gyro.reading;
            let inertia = self.system.props.inertia_kg_m2();
            self.system.loads.moment_b = inertia * error * RATE_GAIN_PER_S;
            self.system.loads.thrust_n = self.command.thrust_n;
        }
    }
}

impl InputModel for ReferenceVehicle {
    type Input = PilotCommand;

    fn apply(&mut self, input: &PilotCommand) {
        self.command = *input;
    }
}

impl StateHash for ReferenceVehicle {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.system.loads.thrust_n.hash_state(hasher);
        self.system.loads.moment_b.hash_state(hasher);
        self.gyro.hash_state(hasher);
        self.accelerometer.hash_state(hasher);
        self.command.rate_rad_s.hash_state(hasher);
        self.command.thrust_n.hash_state(hasher);
    }
}

/// The reference vehicle in its world, at tick zero.
type ReferenceWorld = World<ReferenceVehicle, Rkmk4>;

fn reference_world(seed: u64) -> Result<ReferenceWorld, ScenarioError> {
    let inertia = Mat3::from_rows([
        [0.021, 0.0004, -0.0006],
        [0.0004, 0.024, 0.0002],
        [-0.0006, 0.0002, 0.040],
    ]);
    let hover = PilotCommand {
        rate_rad_s: Vec3::zero(),
        thrust_n: MASS_KG * GRAVITY_M_S2,
    };
    let vehicle = ReferenceVehicle {
        system: RigidBodySystem {
            props: MassProperties::new(MASS_KG, Vec3::zero(), inertia)?,
            loads: Actuators {
                thrust_n: hover.thrust_n,
                moment_b: Vec3::zero(),
            },
        },
        gyro: NoisySensor::new(seed, GYRO_STREAM, 500, 0.01)?,
        accelerometer: NoisySensor::new(seed, ACCELEROMETER_STREAM, 250, 0.05)?,
        controller: Rate::new(PHYSICS_HZ, 250)?,
        command: hover,
    };
    let mut start = RigidBodyState::at_rest(Vec3::new(0.0, 0.0, -10.0), Rotation::identity());
    start.omega_rad_s = Vec3::new(0.4, -0.3, 1.5);
    Ok(World::new(
        vehicle,
        Rkmk4,
        RigidBodyCoordinates::from(start),
        PHYSICS_HZ,
    )?)
}

/// A random command: body rates with a standard deviation of 0.5 rad/s, and thrust within a
/// few percent of hover.
fn pilot_command(pilot: &mut Xoshiro256StarStar) -> PilotCommand {
    let rate = Vec3::new(
        pilot.next_gaussian(),
        pilot.next_gaussian(),
        pilot.next_gaussian(),
    );
    PilotCommand {
        rate_rad_s: rate * 0.5,
        thrust_n: MASS_KG * GRAVITY_M_S2 * (1.0 + 0.05 * pilot.next_gaussian()),
    }
}

/// The fingerprints of one run, and where it ended.
#[derive(Clone, Debug, PartialEq)]
pub struct Run {
    /// The state hash at each checkpoint, one a second from tick 0 to the end.
    pub checkpoints: Vec<Checkpoint>,
    /// The state hash at the end of the run.
    pub final_hash: u64,
    /// The vehicle's state at the end of the run.
    pub final_state: RigidBodyCoordinates,
    /// The pilot command held at the end of the run.
    pub final_command: PilotCommand,
}

/// Flies the reference world for the whole run, calling `before_step` before every step and
/// taking a checkpoint whenever one is due.
fn fly<F>(mut world: ReferenceWorld, mut before_step: F) -> Result<Run, ScenarioError>
where
    F: FnMut(&mut ReferenceWorld) -> Result<(), ScenarioError>,
{
    let mut checkpoints = Checkpoints::new(Rate::new(PHYSICS_HZ, CHECKPOINT_HZ)?, CHECKPOINTS);
    for _ in 0..TICKS {
        before_step(&mut world)?;
        world.checkpoint(&mut checkpoints)?;
        world.step();
    }
    world.checkpoint(&mut checkpoints)?;
    Ok(Run {
        checkpoints: checkpoints.records().to_vec(),
        final_hash: world.state_hash(),
        final_state: *world.current(),
        final_command: world.model().command,
    })
}

/// Flies the scenario seeded with `seed`, with a random pilot command from the `pilot` stream
/// every half second, and returns the run's fingerprints and its recorded commands.
///
/// # Errors
///
/// A [`ScenarioError`], which the published configuration never produces.
pub fn record(seed: u64) -> Result<(Run, InputLog<PilotCommand>), ScenarioError> {
    let mut pilot = Xoshiro256StarStar::named(seed, PILOT_STREAM);
    let mut inputs = InputLog::with_capacity(PILOT_COMMANDS);
    let run = fly(reference_world(seed)?, |world| {
        if world.clock().tick().is_multiple_of(PILOT_INTERVAL_TICKS) {
            world.apply(pilot_command(&mut pilot), &mut inputs)?;
        }
        Ok(())
    })?;
    Ok((run, inputs))
}

/// Flies the scenario seeded with `seed` again, taking its pilot commands from `inputs` instead
/// of the `pilot` stream. Inputs recorded after the run's last tick are not applied.
///
/// # Errors
///
/// A [`ScenarioError`], which the published configuration never produces.
pub fn replay(seed: u64, inputs: &InputLog<PilotCommand>) -> Result<Run, ScenarioError> {
    let mut replay = Replay::new(inputs);
    fly(reference_world(seed)?, |world| {
        world.replay_due(&mut replay);
        Ok(())
    })
}

/// The published study: the run seeded with [`MASTER_SEED`], and the replay of its inputs.
///
/// # Errors
///
/// A [`ScenarioError`], which the published configuration never produces.
pub fn determinism_study() -> Result<(Run, Run), ScenarioError> {
    let (recorded, inputs) = record(MASTER_SEED)?;
    let replayed = replay(MASTER_SEED, &inputs)?;
    Ok((recorded, replayed))
}

/// Header of the study table.
const HEADER: &str = "| Time (s) | Tick | Recorded run | Replay of its inputs |\n\
                      |---:|---:|---|---|\n";

/// The study as a Markdown table, one row per checkpoint, hashes in hexadecimal.
#[must_use]
pub fn markdown(recorded: &Run, replayed: &Run) -> String {
    let lines: String = recorded
        .checkpoints
        .iter()
        .zip(&replayed.checkpoints)
        .map(|(a, b)| table_line(a, b))
        .collect();
    format!("{HEADER}{lines}")
}

fn table_line(recorded: &Checkpoint, replayed: &Checkpoint) -> String {
    let seconds = recorded.tick / u64::from(PHYSICS_HZ);
    format!(
        "| {seconds} | {} | `{:016x}` | `{:016x}` |\n",
        recorded.tick, recorded.hash, replayed.hash
    )
}
