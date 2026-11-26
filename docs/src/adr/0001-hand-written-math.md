# ADR 0001: Hand-written, frame-typed math

- **Status:** Accepted. The external-dependency allowlist is amended by
  [ADR 0006](0006-errors-and-configuration.md).
- **Date:** 2026-11-26
- **Issues:** [#8](https://github.com/MatthewK84/MJKPhysics/issues/8), [#9](https://github.com/MatthewK84/MJKPhysics/issues/9), [#2](https://github.com/MatthewK84/MJKPhysics/issues/2)

## Context

The engine needs 3-vectors, 3×3 matrices, unit quaternions and the SO(3) exponential and logarithm
maps. It also needs small dense solvers later: eigen-decomposition of inertia tensors, and the
eigenvalues used in modal analysis. The project's purpose is to be checkable, and the most common
simulator bugs are frame and convention errors: adding a body-frame vector to a world-frame one,
or applying a quaternion in the wrong direction.

## Decision

Engine crates use hand-written linear algebra in `mjk-math`, with no third-party math or physics
crates.

- Vectors carry their frame in their type: `Vec3<Body>`, `Vec3<World>`. A rotation is typed by
  its source and target frames, so `q_wb.rotate(v_b)` returns `Vec3<World>`, and passing a world
  vector is a compile error.
- All types are `Copy`, sized at compile time, and never allocate.
- Transcendental functions go through `libm` so results are bit-identical on every platform.
- The layering check allows only `libm`, `serde` and `toml` as external engine dependencies.

## Consequences

- Frame mistakes become compile errors instead of silent wrong answers.
- Every operation is short enough to read next to its equation in the
  [conventions](../conventions.md), and has property tests: rotations preserve norms, and
  composition matches matrix products.
- More code to write and test than importing a library. The surface is small, roughly what
  [#8](https://github.com/MatthewK84/MJKPhysics/issues/8) and
  [#9](https://github.com/MatthewK84/MJKPhysics/issues/9) list, so this is bounded.
- No SIMD-specialized kernels. At the engine's scale (a handful of bodies, 1 kHz) this costs
  nothing measurable; the benchmarks ([#82](https://github.com/MatthewK84/MJKPhysics/issues/82))
  will confirm it.

## Alternatives considered

- **nalgebra.** Comprehensive and well tested, but it doesn't tag vectors with frames, its
  generic types slow compilation and obscure error messages, and it doesn't route through `libm`
  by default.
- **glam.** Fast and simple, but `f32`-first, without frame typing, and with its own precision
  and SIMD behavior that would complicate bit-exact determinism.
- **A units-of-measure crate (`uom`).** Compile-time units are attractive, but they add heavy
  generics throughout. Unit suffixes in names and frame types catch most errors at far lower cost.
- **An existing physics engine (Rapier, others).** It would hide exactly the parts this project
  exists to show, and its conventions and integrators are not designed for flight dynamics.
