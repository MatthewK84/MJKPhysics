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
[ADR 0006](adr/0006-errors-and-configuration.md)).

## Rules the engine follows

- **No I/O in engine crates.** Files, terminals, sockets and the browser belong to front ends.
  Engine crates take configuration as text and return results as data
  ([Configuration files](configuration.md)).
- **Fixed-step and deterministic.** Physics advances in fixed steps, with subsystems at integer
  rate divisors. The same inputs and seed produce bit-identical state on every platform
  ([ADR 0002](adr/0002-integrator-design.md)).
- **No heap allocation per step.** Everything `World::step` needs is allocated when the world is
  built.
- **No panics.** Fallible operations return `Result<_, EngineError>` from `mjk-core`.
  Configuration errors name the exact field; numerical failures report the simulation time and
  the offending quantity ([ADR 0006](adr/0006-errors-and-configuration.md)).
- **Rendering never feeds back.** The viewer receives state snapshots and pilot inputs flow the
  other way; nothing the renderer does can change a simulated result
  ([ADR 0004](adr/0004-renderer-stack.md)).

## One simulation step

The world loop ([#13](https://github.com/MatthewK84/MJKPhysics/issues/13)) will advance the state
in this order, each stage reading only results from earlier stages:

1. Environment: atmosphere and wind at each evaluation point.
2. Sensors and controllers that are due on this tick, using sample-and-hold between their updates.
3. Actuators: motors, ESCs and control surfaces respond to commands.
4. Forces and moments: rotors, aerodynamics and gravity, summed about the centre of mass.
5. Integration of the rigid-body state ([ADR 0002](adr/0002-integrator-design.md)).
6. Contact: collision detection and impulse resolution ([ADR 0003](adr/0003-contact-solver.md)).
7. Logging and snapshot, written into preallocated buffers.
