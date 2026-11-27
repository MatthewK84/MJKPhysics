//! A world step allocates nothing (#13): 10⁵ steps with RK4 and with RKMK4, and real-time
//! advances, under a counting global allocator.
//!
//! This test runs without the test harness (`harness = false`), so no other thread allocates
//! while it measures; `main` panics through an assertion on any allocation.

use mjk_alloc_counter::{CountingAllocator, allocations};
use mjk_dynamics::mass::MassProperties;
use mjk_dynamics::{
    Clock, Integrator, LoadModel, Loads, Model, Rate, RigidBodyCoordinates, RigidBodyState,
    RigidBodySystem, Rk4, Rkmk4, World,
};
use mjk_math::{Mat3, Rotation, Vec3};

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
struct Vehicle {
    system: RigidBodySystem<HeldForce>,
    sensor: Rate,
    controller: Rate,
    samples: [f64; 16],
    next: usize,
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
            self.system.loads.force_n = -0.1 * mean;
        }
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
    println!("allocations: 0 in {STEPS} steps and 1000 real-time advances, with RK4 and RKMK4");
    Ok(())
}
