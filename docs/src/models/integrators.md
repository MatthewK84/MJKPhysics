# Integrators

- **Issues:** [#11](https://github.com/MatthewK84/MJKPhysics/issues/11),
  [#12](https://github.com/MatthewK84/MJKPhysics/issues/12) (the Lie-group method and the study)
- **Code:** `mjk_dynamics::{Integrator, ExplicitEuler, SemiImplicitEuler, Rk4, Rkmk4}`,
  `mjk_dynamics::{System, LieSystem, Coordinates}`,
  `mjk_dynamics::{RigidBodySystem, RigidBodyCoordinates, RigidBodyTangent, LoadModel, Loads}`,
  `mjk_sim::validation::integrators`
- **Equation prefix:** `INT`
- **Decision record:** [ADR 0002](../adr/0002-integrator-design.md)

## Summary

The engine advances in fixed steps ([ADR 0002](../adr/0002-integrator-design.md)). An integrator
takes a system of ordinary differential equations $`\dot{\mathbf{y}} = \mathbf{f}(t, \mathbf{y})`$
and returns $`\mathbf{y}`$ one step $`h`$ later. Four methods are provided: explicit Euler and
semi-implicit Euler as references; the classical fourth-order Runge–Kutta method (RK4), which
integrates the quaternion's components and renormalizes; and the Runge–Kutta–Munthe-Kaas method
(RKMK4), which steps the attitude on the rotation group itself. A study at the end of the page
measures what the group method buys. The [rigid body](rigid-body.md) is the first system; motor and battery states
join it later without changing the integrators.

## Assumptions

1. The step $`h`$ is fixed for a run, and loads are pure functions of time and state during a
   step: inputs such as controller commands change only between steps.
2. The coordinates form a vector space, so stages can be added and scaled. The attitude is
   integrated as four quaternion components, which is a vector space; the unit-norm constraint is
   restored after each step (INT-4).
3. The position-like part of the rate (positions and attitude) depends only on the coordinates:
   it is kinematics. Semi-implicit Euler relies on this.

## Methods

Explicit (forward) Euler evaluates the rate once, at the start of the step:

```math
\mathbf{y}_{n+1} = \mathbf{y}_n + h\,\mathbf{f}(t_n, \mathbf{y}_n)
\qquad\text{(INT-1)}
```

Semi-implicit (symplectic) Euler splits the coordinates into position-like parts $`\mathbf{x}`$
and velocity-like parts $`\mathbf{u}`$. It updates the velocities first, then the positions from
the new velocities:

```math
\mathbf{u}_{n+1} = \mathbf{u}_n + h\,\mathbf{a}(t_n, \mathbf{x}_n, \mathbf{u}_n),
\qquad
\mathbf{x}_{n+1} = \mathbf{x}_n + h\,\mathbf{g}(\mathbf{x}_n, \mathbf{u}_{n+1})
\qquad\text{(INT-2)}
```

Both are first order. Explicit Euler adds energy to an undamped oscillator every step;
semi-implicit Euler keeps the energy error bounded, because it exactly conserves a nearby
modified energy (Hairer, Lubich & Wanner 2006).

The classical RK4 method evaluates the rate four times per step:

```math
\begin{aligned}
\mathbf{k}_1 &= \mathbf{f}(t_n, \mathbf{y}_n), &
\mathbf{k}_2 &= \mathbf{f}\big(t_n + \tfrac h2,\ \mathbf{y}_n + \tfrac h2\,\mathbf{k}_1\big), \\
\mathbf{k}_3 &= \mathbf{f}\big(t_n + \tfrac h2,\ \mathbf{y}_n + \tfrac h2\,\mathbf{k}_2\big), &
\mathbf{k}_4 &= \mathbf{f}\big(t_n + h,\ \mathbf{y}_n + h\,\mathbf{k}_3\big), \\
\mathbf{y}_{n+1} &= \mathbf{y}_n + \tfrac h6\,\big(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\big)
\end{aligned}
\qquad\text{(INT-3)}
```

Its global error is $`O(h^4)`$. For a rate that depends on time only, it reduces to Simpson's rule
and integrates cubics exactly.

## Attitude

The attitude is integrated as the raw quaternion components $`\mathbf{q}`$, with the kinematics of
(CONV-9), and normalized once at the end of each step:

```math
\dot{\mathbf{q}} = \tfrac12\,\mathbf{q}\otimes\begin{bmatrix} 0 \\ \boldsymbol\omega^B \end{bmatrix},
\qquad
\mathbf{q}_{n+1} \leftarrow \frac{\mathbf{q}_{n+1}}{\lVert\mathbf{q}_{n+1}\rVert}
\qquad\text{(INT-4)}
```

Two details keep RK4 at fourth order:

- **No normalization between stages.** A stage such as $`\mathbf{q}_n + \tfrac h2\mathbf{k}_1`$ lies
  slightly off the unit sphere, by $`O(h^2)`$. Normalizing it would scale the next stage's rate by
  $`1 - O(h^2)`$ along the direction of rotation, an $`O(h^3)`$ error per step, and the method would
  drop to second order. The final normalization is a projection onto the exact solution's
  manifold and does not lower the order.
- **The physics sees the normalized attitude.** Loads and the Newton–Euler equations receive
  $`\mathbf{q}/\lVert\mathbf{q}\rVert`$, since rotating a vector with a non-unit quaternion would
  scale it by $`\lVert\mathbf{q}\rVert^2`$. The kinematics (INT-4) uses the raw $`\mathbf{q}`$, which
  keeps the whole right-hand side a smooth function of the coordinates.

Renormalizing is the baseline against which the Lie-group method below, which stays on SO(3) by
construction, is compared.

## Lie-group method (RKMK4)

Near the attitude $`\mathbf{q}_0`$ at the start of a step, write the attitude as a rotation
$`\boldsymbol\theta`$ applied on the right, as in (CONV-10). Differentiating
$`\mathbf{q}_0\otimes\mathrm{Exp}(\boldsymbol\theta)`$ gives
$`\boldsymbol\omega = \mathbf{J}_r(\boldsymbol\theta)\,\dot{\boldsymbol\theta}`$ (SO3-4), so in
this chart the attitude obeys an ordinary differential equation in a vector space:

```math
\mathbf{q}(t) = \mathbf{q}_0 \otimes \mathrm{Exp}\big(\boldsymbol\theta(t)\big),
\qquad
\dot{\boldsymbol\theta} = \mathbf{J}_r^{-1}(\boldsymbol\theta)\,\boldsymbol\omega^B,
\qquad \boldsymbol\theta(0) = \mathbf{0}
\qquad\text{(INT-6)}
```

with $`\mathbf{J}_r^{-1}`$ from (SO3-5). RKMK4 applies classical RK4 to $`\boldsymbol\theta`$ and,
in the same stages, to position, velocity and angular velocity, then maps back with the
exponential:

```math
\begin{aligned}
\mathbf{k}_i &= \mathbf{J}_r^{-1}(\boldsymbol\theta_i)\,\boldsymbol\omega\big(\mathbf{q}_0\otimes\mathrm{Exp}(\boldsymbol\theta_i)\big),
\quad \boldsymbol\theta_1 = \mathbf{0},\ \boldsymbol\theta_2 = \tfrac h2\mathbf{k}_1,\ \boldsymbol\theta_3 = \tfrac h2\mathbf{k}_2,\ \boldsymbol\theta_4 = h\,\mathbf{k}_3, \\
\mathbf{q}_{n+1} &= \mathbf{q}_n \otimes \mathrm{Exp}\Big(\tfrac h6\,\big(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\big)\Big)
\end{aligned}
\qquad\text{(INT-7)}
```

The new attitude is a product of unit quaternions, so in exact arithmetic it is a unit quaternion;
in floating point its norm only wanders by rounding, and nothing is renormalized. Two closed-form
cases check the implementation:

```math
\boldsymbol\omega \parallel \text{a principal axis}
\;\Rightarrow\;
\boldsymbol\theta_i \parallel \boldsymbol\omega,\ \
\mathbf{J}_r^{-1}(\boldsymbol\theta_i)\,\boldsymbol\omega = \boldsymbol\omega
\;\Rightarrow\;
\mathbf{q}_{n+1} = \mathbf{q}_n\otimes\mathrm{Exp}(h\,\boldsymbol\omega)\ \text{exactly}
\qquad\text{(INT-8)}
```

RKMK4 therefore reproduces steady spin about a principal axis to rounding, where RK4 on the
quaternion components has an $`O(h^4)`$ phase error; and with no rotation it reduces to RK4, so it
is exact for constant acceleration.

## Study: RK4 with renormalization against RKMK4

A torque-free body with an X500-class inertia tensor, products of inertia included,
$`\mathbf{J} = [0.021, 0.0004, -0.0006;\ 0.0004, 0.024, 0.0002;\ -0.0006, 0.0002, 0.040]`$ kg·m²,
spins at $`\boldsymbol\omega_0 = (0.2, 6.0, 0.3)`$ rad/s, almost about its intermediate axis, so it
tumbles and every component of $`\boldsymbol\omega`$ changes sign. Each method runs for 100 s at
three step sizes. With no moment, kinetic energy and the angular-momentum vector in world axes
are conserved exactly, so any change is integration error. The table gives the largest relative
change over the run and the largest deviation of $`\lVert\mathbf{q}\rVert`$ from 1. RK4 is also run
without renormalization to show what renormalization corrects.

{{#include ../validation/integrator-study.md}}

*Generated by `mjk validate`; a test fails if this table no longer matches the code.*

What the study shows:

- **The norm.** RKMK4 holds $`\lVert\mathbf{q}\rVert`$ to $`10^{-14}`$ with no renormalization:
  rounding only. RK4 without renormalization drifts off the sphere by up to $`1.7\times10^{-4}`$ at
  $`h = 0.05`$ s; renormalizing removes that, as intended.
- **Energy.** Identical for both methods. Kinetic energy depends only on $`\boldsymbol\omega`$, whose
  equation lives in a vector space where RKMK4 is RK4.
- **Angular momentum.** RKMK4 is slightly better, by about 5 % at the larger steps
  ($`8.9\times10^{-6}`$ against $`9.4\times10^{-6}`$ at $`h = 0.05`$ s): its attitude error is smaller.
- **Cost.** RKMK4 needs four more exponential maps and four inverse Jacobians per step. Measured
  with `cargo run --release -p mjk-cli -- validate --timing` on a shared 2.1 GHz Xeon virtual
  machine, RK4 took 220 to 320 ns per step and RKMK4 590 to 860 ns, 2.1 to 2.8 times as long.

At the default 1 kHz step both methods conserve energy and momentum to $`10^{-12}`$, far below any
modeling error, so the choice is about invariants and cost, not accuracy. RKMK4 is the method that
keeps the attitude a rotation by construction; RK4 with renormalization is cheaper and, for this
problem, as accurate.

## Measuring the order

The order $`p`$ of a method is the slope of the error against the step size on logarithmic axes:

```math
e(h) \approx C\,h^{p}
\quad\Longrightarrow\quad
\ln e = \ln C + p\,\ln h
\qquad\text{(INT-5)}
```

The tests fit $`p`$ by least squares over a geometric sequence of steps, small enough to be in the
asymptotic range and large enough that rounding does not dominate.

## Validity range

All three methods are explicit, so the step must resolve the fastest dynamics: for RK4, stability
on an oscillator requires $`h\,\omega < 2.8`$, and accuracy needs much less. The default 1 kHz rate
resolves motor time constants of tens of milliseconds and every body mode with a wide margin
(ADR 0002). Stiff contact is handled after the step by the impulse solver, not by the integrator.

## Implementation notes

- **One trait, static dispatch.** `Integrator::step` is generic over the `System`, so the
  compiler specializes each combination; no trait objects (Power of Ten rule 9) and no
  allocation.
- **Choosing a method at run time.** `IntegratorKind` names each method (`explicit-euler`,
  `semi-implicit-euler`, `rk4`, `rkmk4`) for scenario files and dispatches with a `match`, so a
  method chosen from a file still needs no trait object. It steps exactly like the method it
  names, to the bit.
- **Coordinates hold their own rates.** The same type is used for $`\mathbf{y}`$ and
  $`\dot{\mathbf{y}}`$. Its fields are named for the quantity (`position`, `velocity`,
  `attitude`, `angular_velocity`) without units, because the units differ between the two uses.
- **Semi-implicit Euler** evaluates the rate twice: once for the velocities, then at the updated
  velocities for the positions.
- **A degenerate attitude** (a zero or non-finite quaternion) makes every rate NaN, so the failure
  shows in the state instead of being hidden by a fallback.

## Validation

| Check | Result | Test |
|---|---|---|
| Constant acceleration: each method against its own closed form (explicit Euler $`h^2 a\,n(n-1)/2`$, semi-implicit $`h^2 a\,n(n+1)/2`$, RK4 exact) | agree to $`10^{-14}`$ | `each_method_has_its_own_closed_form_under_constant_acceleration` |
| RK4 integrates a cubic rate exactly; Euler gives the Riemann sum | agree to $`10^{-15}`$ | `rk4_integrates_a_cubic_rate_exactly` |
| Order on an oscillator, $`x'' = -x`$ to $`t = 10`$ s, $`h = 0.1`$ to $`0.00625`$ s | explicit 1.08, semi-implicit 1.07, RK4 3.99 | `measured_order_on_an_oscillator` |
| Order on a tumbling asymmetric body with products of inertia, to $`t = 2`$ s, against RK4 at $`h = 1/3200`$ s | explicit 1.01, semi-implicit 1.00, RK4 4.00 | `measured_order_on_a_tumbling_body` |
| Semi-implicit Euler energy over $`10^6`$ steps of $`h = 0.01`$ s (about 1600 periods) | bounded: at most 0.50 % | `semi_implicit_euler_keeps_oscillator_energy_bounded` |
| Explicit Euler energy, which grows by exactly $`1 + h^2`$ per step | matches $`(1 + h^2)^{n}`$ to $`10^{-8}`$ | `explicit_euler_gains_energy_on_an_oscillator` |
| Unit attitude after 1000 steps of each method | $`\lvert\lVert\mathbf{q}\rVert - 1\rvert < 10^{-15}`$ | `the_attitude_stays_a_unit_quaternion` |
| RKMK4: exact under constant acceleration, like RK4 | agree to $`10^{-14}`$ | `each_method_has_its_own_closed_form_under_constant_acceleration` |
| RKMK4: steady spin about a principal axis (INT-8) | RKMK4 error below $`10^{-13}`$ rad; RK4 above $`10^{-9}`$ rad | `rkmk4_is_exact_for_steady_spin_about_a_principal_axis` |
| RKMK4: measured order on the oscillator and on the tumbling body | within $`\pm 0.2`$ of 4 | `measured_order_on_an_oscillator`, `measured_order_on_a_tumbling_body` |
| RKMK4: unit norm over $`10^5`$ steps with no renormalization | $`\lvert\lVert\mathbf{q}\rVert - 1\rvert < 10^{-12}`$ | `rkmk4_keeps_a_unit_quaternion_without_renormalizing` |
| A method chosen at run time steps like the method itself; names round-trip | bit-identical after 50 steps, for all four | `a_method_chosen_at_run_time_steps_exactly_like_the_method_itself` |
| The published study table matches the code | identical | `the_published_table_matches_the_code`, `validate_is_byte_identical_and_matches_the_book` |

The acceptance tolerance for every measured order is $`\pm 0.2`$.

## References

- H. Munthe-Kaas, "Runge–Kutta methods on Lie groups," *BIT Numerical Mathematics* 38, 1998.
- A. Iserles, H. Z. Munthe-Kaas, S. P. Nørsett and A. Zanna, "Lie-group methods," *Acta
  Numerica* 9, 2000.
- P. E. Crouch and R. Grossman, "Numerical integration of ordinary differential equations on
  manifolds," *Journal of Nonlinear Science* 3, 1993.
- E. Hairer, C. Lubich and G. Wanner, *Geometric Numerical Integration*, 2nd ed., Springer,
  2006, chapters I and VI.
- E. Hairer, S. P. Nørsett and G. Wanner, *Solving Ordinary Differential Equations I: Nonstiff
  Problems*, 2nd ed., Springer, 1993, section II.1.
- J. Solà, "Quaternion kinematics for the error-state Kalman filter," arXiv:1711.02508, 2017.
