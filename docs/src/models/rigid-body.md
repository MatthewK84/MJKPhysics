# Rigid-body dynamics

- **Issue:** [#10](https://github.com/MatthewK84/MJKPhysics/issues/10)
- **Code:** `mjk_dynamics::{RigidBodyState, StateDerivative, derivative, Wrench}`,
  `mjk_dynamics::mass::{MassProperties, Component, Shape, composite}`
- **Equation prefix:** `RB`

## Summary

Every aircraft in the engine is one rigid body with six degrees of freedom. This page gives the
state, the Newton–Euler equations that advance it, how loads are combined, and how mass
properties are built from simple components. Integrators
([#11](https://github.com/MatthewK84/MJKPhysics/issues/11),
[#12](https://github.com/MatthewK84/MJKPhysics/issues/12)) step these equations in time. Frames,
symbols and quaternion conventions follow the [conventions page](../conventions.md).

## Assumptions

1. The airframe is rigid: no flexing wings or arms, and rotors are modeled as forces and
   moments, not as separate bodies.
2. Mass and inertia are constant during a flight. The aircraft are electric, so no fuel burns;
   a released or slung payload is a separate body (M5).
3. The world frame is flat, non-rotating and inertial (see the conventions page). Neglecting the
   Earth's rotation, $`\Omega_E = 7.29\times10^{-5}`$ rad/s, drops a Coriolis acceleration of at
   most $`2\Omega_E v = 4.4\times10^{-3}`$ m/s² at 30 m/s, under 0.05 % of gravity.
4. The body frame's origin is the center of mass, so translation and rotation decouple.

## State

| Symbol | Meaning | Unit | Code |
|---|---|---|---|
| $`\mathbf{p}^W`$ | Position of the center of mass | m | `position_m: Vec3<World>` |
| $`\mathbf{v}^W`$ | Velocity of the center of mass | m/s | `velocity_m_s: Vec3<World>` |
| $`\mathbf{q}_{WB}`$ | Attitude, body to world | — | `q_wb: Rotation<World, Body>` |
| $`\boldsymbol\omega^B`$ | Angular velocity, body axes | rad/s | `omega_rad_s: Vec3<Body>` |

## Equations of motion

Translation, with $`\mathbf{F}^W`$ the total force including gravity:

```math
\dot{\mathbf{p}}^W = \mathbf{v}^W, \qquad m\,\dot{\mathbf{v}}^W = \mathbf{F}^W
\qquad\text{(RB-1)}
```

Rotation (Euler's equations), with $`\mathbf{M}^B`$ the total moment about the center of mass and
$`\mathbf{J}`$ the full inertia tensor, products of inertia included:

```math
\mathbf{J}\,\dot{\boldsymbol\omega}^B = \mathbf{M}^B - \boldsymbol\omega^B \times \big(\mathbf{J}\,\boldsymbol\omega^B\big)
\qquad\text{(RB-2)}
```

The attitude follows the kinematics of (CONV-9), $`\dot{\mathbf{q}}_{WB} = \tfrac12\,\mathbf{q}_{WB}\otimes(0,\ \boldsymbol\omega^B)`$.
The derivative returns $`\boldsymbol\omega^B`$ itself as the attitude rate: it is the tangent of the
attitude on SO(3), from which an integrator either forms $`\dot{\mathbf{q}}`$ or steps on the group
with the exponential map (CONV-10).

## Loads

Each model reports a wrench: a force and a moment about the center of mass. A force
$`\mathbf{F}`$ applied at a point $`\mathbf{r}`$ from the center of mass, such as a rotor's thrust at
its hub, exerts the moment

```math
\mathbf{M} = \mathbf{r} \times \mathbf{F}
\qquad\text{(RB-3)}
```

Points given from the airframe reference point are first measured from the center of mass:
$`\mathbf{r} = \mathbf{r}_P - \mathbf{c}`$. Wrenches add component by component. Body-axis forces
are rotated into world axes with $`\mathbf{R}_{WB}`$ (CONV-1) before (RB-1).

## Mass properties from components

A body can be described by components: point masses, solid boxes and solid cylinders of uniform
density, each with a mass $`m_i`$, a center $`\mathbf{r}_i`$ from the reference point, and an
orientation $`\mathbf{R}_i`$ from its own part axes to body axes. The whole has

```math
m = \sum_i m_i, \qquad \mathbf{c} = \frac{1}{m}\sum_i m_i\,\mathbf{r}_i
\qquad\text{(RB-4)}
```

and, by the parallel-axis theorem with offsets $`\mathbf{d}_i = \mathbf{r}_i - \mathbf{c}`$ from the
composite center of mass,

```math
\mathbf{J} = \sum_i \Big[\mathbf{R}_i\,\mathbf{J}_i\,\mathbf{R}_i^\top + m_i\big(\lVert\mathbf{d}_i\rVert^2\,\mathbf{I} - \mathbf{d}_i\,\mathbf{d}_i^\top\big)\Big]
\qquad\text{(RB-5)}
```

The central inertia $`\mathbf{J}_i`$ of each shape, in its part axes, is zero for a point mass, and
for a box with edges $`a, b, c`$ along $`x, y, z`$ and a cylinder of radius $`r`$ and length $`L`$
along $`z`$:

```math
\mathbf{J}_{\text{box}} = \frac{m}{12}\,\mathrm{diag}\big(b^2 + c^2,\ a^2 + c^2,\ a^2 + b^2\big)
\qquad\text{(RB-6)}
```

```math
\mathbf{J}_{\text{cyl}} = \mathrm{diag}\Big(\frac{m\,(3r^2 + L^2)}{12},\ \frac{m\,(3r^2 + L^2)}{12},\ \frac{m\,r^2}{2}\Big)
\qquad\text{(RB-7)}
```

## Energy and momentum

```math
T = \tfrac12\,m\,\lVert\mathbf{v}^W\rVert^2 + \tfrac12\,\boldsymbol\omega^{B\top}\mathbf{J}\,\boldsymbol\omega^B,
\qquad
\mathbf{H}^W = \mathbf{R}_{WB}\,\mathbf{J}\,\boldsymbol\omega^B
\qquad\text{(RB-8)}
```

From (RB-1) and (RB-2), $`\dot T = \mathbf{F}\cdot\mathbf{v} + \mathbf{M}\cdot\boldsymbol\omega`$: the
gyroscopic term $`\boldsymbol\omega \times \mathbf{J}\boldsymbol\omega`$ is perpendicular to
$`\boldsymbol\omega`$ and does no work. Without a moment, $`\mathbf{H}^W`$ is constant. Both
identities are tested, and later used to measure integrator drift.

## Validity range

The equations are exact for a rigid body under the assumptions above. Inputs are checked when
mass properties are built:

- the mass must be positive and finite, and the center of mass finite;
- the tensor must be finite, symmetric to a relative $`10^{-9}`$, positive definite, and satisfy
  the triangle inequality on its principal moments, $`J_1 \le J_2 + J_3`$ and cyclically; otherwise
  no mass distribution has it (Wensing, Kim & Slotine 2018);
- components need a positive mass, finite position and non-negative dimensions. A flat plate of
  zero thickness is accepted; point masses on one line are not, since their inertia is singular.

Configuration files report any violation as an `EngineError::Config` naming the field, for
example `mass.inertia_kg_m2`.

## Parameters

| Symbol | Meaning | Unit | Source |
|---|---|---|---|
| $`m`$ | Mass | kg | Airframe file |
| $`\mathbf{c}`$ | Center of mass from the airframe reference point, body axes | m | Airframe file, or (RB-4) |
| $`\mathbf{J}`$ | Inertia tensor about the center of mass, body axes | kg·m² | Airframe file, or (RB-5) |

Each airframe's values and their sources are on its [airframe page](../airframes/README.md).

## Implementation notes

- **No inversion per step.** $`\mathbf{J}^{-1}`$ is computed once, when `MassProperties` is
  built, so `derivative` is a few matrix-vector products. It allocates nothing.
- **Symmetric storage.** The stored tensor is $`\tfrac12(\mathbf{J} + \mathbf{J}^\top)`$, so
  rounding-level asymmetry in published values cannot make (RB-2) non-conservative.
- **Composites about the center of mass.** (RB-5) is accumulated about the composite center of
  mass from each component's offset to it. Accumulating about the reference point and shifting
  afterward would subtract two large terms when offsets are much larger than the components, as
  for a thin battery far from the center.
- **Part axes.** Component orientations are typed `Rotation<Body, Part>`, so a component's
  inertia cannot be added to body-axis quantities before it is rotated.
- Code comments cite (RB-1) to (RB-7) at the lines that implement them.

## Validation

| Check | Tolerance | Test |
|---|---|---|
| A box plus a point mass against a tensor computed by hand, products of inertia included | $`10^{-14}`$ relative | `two_component_body_matches_the_hand_computed_tensor` |
| Box and cylinder against (RB-6) and (RB-7) | $`10^{-15}`$ relative | `box_and_cylinder_match_their_formulas` |
| Cutting any box or cylinder in half leaves mass, center of mass and inertia unchanged | $`10^{-12}`$ relative | `cutting_a_component_in_half_changes_nothing` (512 random cases) |
| Rotating every component rotates the composite: $`\mathbf{c}' = \mathbf{Q}\mathbf{c}`$, $`\mathbf{J}' = \mathbf{Q}\mathbf{J}\mathbf{Q}^\top`$ | $`10^{-12}`$ relative | `turning_every_component_turns_the_whole` (512) |
| Euler's equations in principal axes, against their closed form | $`10^{-12}`$ | `euler_equations_in_principal_axes` |
| Power balance $`\dot T = \mathbf{F}\cdot\mathbf{v} + \mathbf{M}\cdot\boldsymbol\omega`$ for any state, inertia and loads | $`10^{-12}`$ relative | `power_balance` (512) |
| Torque-free motion conserves $`\mathbf{H}^W`$ | $`10^{-12}`$ relative | `torque_free_motion_conserves_angular_momentum` (512) |
| Non-physical inertia in a configuration file is an `EngineError::Config` naming the field | exact | `every_malformed_sample_names_the_exact_field` |

**Limitations.** These checks test the derivative at one instant. Behavior over time, such as the
torque-free precession and the intermediate-axis flip, is validated with the integrators in
[#16](https://github.com/MatthewK84/MJKPhysics/issues/16).

## References

- R. W. Beard and T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton
  University Press, 2012, chapter 3.
- B. L. Stevens, F. L. Lewis and E. N. Johnson, *Aircraft Control and Simulation*, 3rd ed.,
  Wiley, 2016, chapter 1.
- H. Goldstein, C. Poole and J. Safko, *Classical Mechanics*, 3rd ed., Addison-Wesley, 2002,
  chapters 4 and 5.
- P. M. Wensing, S. Kim and J.-J. E. Slotine, "Linear matrix inequalities for physically
  consistent inertial parameter identification: a statistical perspective on the mass
  distribution," *IEEE Robotics and Automation Letters* 3(1), 2018.
