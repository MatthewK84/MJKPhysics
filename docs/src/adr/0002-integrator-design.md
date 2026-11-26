# ADR 0002: Fixed-step integration with a Lie-group method for attitude

- **Status:** Accepted
- **Date:** 2026-11-26
- **Issues:** [#11](https://github.com/MatthewK84/MJKPhysics/issues/11), [#12](https://github.com/MatthewK84/MJKPhysics/issues/12), [#13](https://github.com/MatthewK84/MJKPhysics/issues/13), [#14](https://github.com/MatthewK84/MJKPhysics/issues/14)

## Context

The state includes an attitude on SO(3), represented as a unit quaternion. Controllers and sensors
run at fixed rates, as they do on real autopilots. Runs must be reproducible bit for bit, natively
and in WebAssembly, so recorded flights replay exactly and native and browser results can be
compared. Forces are dominated by aerodynamics and propulsion, which are dissipative, not
conservative. Contact is the only stiff part of the system.

## Decision

- Physics advances with a **fixed step**, 1 kHz by default. Controllers and sensors run at integer
  divisors of the physics rate, with sample-and-hold between their updates. Real-time front ends
  use an accumulator and interpolate between states for display.
- Integrators implement one trait and are generic over the force model, so they can be swapped and
  compared. Four are provided:
  - explicit Euler and semi-implicit Euler, as references;
  - classical **RK4**, integrating the quaternion and renormalizing;
  - **Runge–Kutta–Munthe-Kaas (RKMK4)** on SO(3) × ℝⁿ, which keeps the attitude on the rotation
    group by construction, with increments applied on the right as in (CONV-10).
- Contact is resolved by an impulse stage after integration, through operator splitting
  ([ADR 0003](0003-contact-solver.md)), rather than inside the integrator.
- The validation suite measures convergence order, energy and angular-momentum drift, and
  unit-norm error for every integrator, and publishes the comparison.

## Consequences

- Controllers see the same sample timing as on hardware, and runs are reproducible.
- The step size must resolve the fastest non-contact dynamics: motor time constants of tens of
  milliseconds and the fastest body modes. 1 kHz gives a wide margin and can be changed per scenario.
- Showing RK4 with renormalization next to RKMK4 makes the cost and benefit of geometric
  integration measurable, not just asserted.

## Alternatives considered

- **Adaptive step size (Dormand–Prince RK45 and similar).** Efficient for smooth problems, but
  controller sampling becomes irregular and runs are no longer reproducible across platforms.
- **Symplectic and variational integrators (for example RATTLE).** Excellent long-term energy
  behavior for conservative systems, but the dominant forces here are dissipative, so the benefit
  is small.
- **Implicit methods.** Only contact is stiff, and the impulse solver handles it. Implicit
  integration of the whole state would add cost and nonlinear solves without a matching benefit.
- **Euler angles as state.** Singular at ±90° pitch; ruled out in the
  [conventions](../conventions.md).
