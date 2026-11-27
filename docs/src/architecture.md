# Architecture

## Crates and layers

The workspace is split into crates whose dependency graph encodes the engine's layers. A crate may
depend only on the layers beneath it, and `cargo xtask layering` enforces this on every pull
request.

```text
                  mjk-core            engine error model (used by every engine crate)
                     |
                  mjk-math            frame-tagged vectors, quaternions, SO(3)
                 /        \
       mjk-dynamics      mjk-env      rigid body, integrators, world loop | atmosphere, wind
        /    |     \        |
 mjk-contact |  mjk-control |         collision and contact | allocation, controllers
        \    |       |      |
         \  mjk-aero |      |         rotors, propulsion, fixed-wing aerodynamics
          \    |     |      |
           mjk-sensors      |         IMU, GNSS, barometer, magnetometer, range, airspeed
                  \         |
                   mjk-sim  ----------  world composition, airframes, scenarios
                  /       \
            mjk-cli       mjk-wasm    front ends: command line | browser bindings
                             |
                          viewer      TypeScript renderer (reads snapshots only)
```

The exact allowed edges are the rule table in `xtask/src/layering.rs`. Engine crates, everything
from `mjk-core` to `mjk-sim`, may use only `libm`, `serde`, `serde_path_to_error` and `toml` as
external dependencies outside of tests ([ADR 0001](adr/0001-hand-written-math.md),
[ADR 0006](adr/0006-errors-and-configuration.md)). The coding rules every crate follows, and how
CI enforces them, are on the [Engineering standards](engineering-standards.md) page.

## Rules the engine follows

- **No I/O in engine crates.** Files, terminals, sockets and the browser belong to front ends.
  Engine crates take configuration as text and return results as data
  ([Configuration files](configuration.md)).
- **Fixed-step and deterministic.** Physics advances in fixed steps, with subsystems at integer
  rate divisors. The same inputs and seed produce bit-identical state on every platform, and a
  recorded run replays exactly ([ADR 0002](adr/0002-integrator-design.md),
  [Determinism and replay](determinism.md)).
- **No heap allocation per step.** Everything `World::step` needs is allocated when the world is
  built; a counting allocator proves it in CI ([Engineering standards](engineering-standards.md)).
- **No panics.** Fallible operations return `Result<_, EngineError>` from `mjk-core`.
  Configuration errors name the exact field; numerical failures report the simulation time and
  the offending quantity ([ADR 0006](adr/0006-errors-and-configuration.md)).
- **Rendering never feeds back.** The viewer receives state snapshots and pilot inputs flow the
  other way; nothing the renderer does can change a simulated result
  ([ADR 0004](adr/0004-renderer-stack.md)).

## One simulation step

`World::step` (`mjk_dynamics::World`) advances the state in this order, each stage reading only
results from earlier stages:

1. **Discrete subsystems** due on this tick, through `Model::update`: sensors first, then
   controllers, reading the state at the start of the step. The inputs they set are held constant
   for the whole step (zero-order hold).
2. **Continuous dynamics**, through one integrator step
   ([ADR 0002](adr/0002-integrator-design.md)). Inside every integrator stage the system
   evaluates, in order: the environment (atmosphere and wind at each point), actuators (motors,
   ESCs and control surfaces responding to the held commands), and forces and moments (rotors,
   aerodynamics and gravity, summed about the center of mass).
3. **After the step**, through `Model::after_step`, with the clock already at the new tick:
   contact detection and impulse resolution ([ADR 0003](adr/0003-contact-solver.md)), then logging
   into preallocated buffers. Both attach here when they land; until then the hook does nothing.

### Time and rates

Time is an integer tick count at a fixed physics rate, 1 kHz by default; seconds are the tick
divided by the rate, a single correctly rounded division, never a running sum. A subsystem's rate
must divide the physics rate exactly: it runs on tick zero and then every `physics/rate` ticks, so
a 500 Hz controller and a 50 Hz barometer run on every second and every twentieth tick. A rate
that does not divide the physics rate is rejected, never rounded.

### Real time

A real-time front end calls `World::advance(real_dt)`, which takes as many whole steps as fit,
carries the remainder to the next call, and returns `alpha`, how far real time has moved past the
last step as a fraction of one. The viewer draws the state interpolated between the previous and
current steps by `alpha`. If more steps are due than a cap allows (a quarter of a second of steps
by default), the whole backlog is dropped and reported, so a slow frame cannot make the next one
slower (the "spiral of death").
