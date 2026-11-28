# Motor and ESC

- **Issue:** [#19](https://github.com/MatthewK84/MJKPhysics/issues/19)
- **Code:** `mjk_aero::propulsion::{DcMotor, LagMotor, MotorModel, Esc, EscState, MotorSystem}`,
  `mjk_sim::config::drive_train`, `mjk_sim::validation::motor`
- **Equation prefix:** `MOT`

## Summary

A brushless motor turns its rotor and propeller, and an electronic speed controller (ESC) sets the
voltage it sees. The model has two tiers:
- **DC-equivalent:** the motor treated as a DC motor with speed constant $`K_v`$, winding
  resistance $`R`$ and no-load current $`i_0`$. It gives speed, current, torque and power.
- **First-order lag:** speed follows the command with one time constant, for airframes where only
  that is published.

The ESC turns a command from 0 to 1 into a duty cycle once per physics step, after a latency
and through a slew limit. It saturates the command, limits the current, and either brakes
actively or lets the motor coast.

Each motor's only state is its rotor's speed, which the integrators step like any other state.
Coupling motor speeds to the rigid body waits for rotor forces
([#21](https://github.com/MatthewK84/MJKPhysics/issues/21)).

## Assumptions

1. **Winding inductance is neglected,** so the current follows the voltage at once. The
   electrical time constant $`L/R`$ is short next to the mechanical one, which is tens of
   milliseconds; the model's speed response leads the real one by about $`L/R`$.
2. **The ESC is lossless and commutation ideal.** The motor sees the average voltage
   $`d\,V_s`$; switching ripple and commutation delay are not modeled.
3. **Parameters are constant.** $`K_v`$, $`R`$ and $`i_0`$ do not change with temperature or
   load. Friction is the constant torque $`i_0/K_v`$ against the spin, not viscous drag.
4. **A rotor turns one way.** Its speed is a non-negative magnitude, and the airframe's `spin`
   gives the direction. A rotor at rest is held there rather than driven backward.
5. **The ESC updates once per physics step.** Its latency is a whole number of steps, and its
   duty cycle is held for the step.
6. **The lag tier's speed follows the duty cycle times a top speed,** with one time constant
   that already includes its load and supply voltage.

## Equations

With $`V`$ the average voltage at the motor and $`\omega \ge 0`$ the rotor's speed, the free
current is

```math
i = \frac{V - \omega/K_v}{R}
\qquad\text{(MOT-1)}
```

and the ESC holds it within its limits. With active braking it may reverse; without, it cannot:

```math
i \leftarrow \min\!\left(\max\left(i,\ i_{\min}\right),\ i_{\max}\right), \qquad
i_{\min} = \begin{cases} -i_{\max} & \text{active braking} \\ 0 & \text{no braking} \end{cases}
\qquad\text{(MOT-2)}
```

In SI units the torque constant is $`K_t = 1/K_v`$, so the torque on the rotor, less friction, is

```math
\tau_m = \frac{i - i_0}{K_v}
\qquad\text{(MOT-3)}
```

The rotor, of inertia $`J`$ (motor and propeller together), turns against its load
$`\tau_L(\omega)`$:

```math
J\,\dot\omega = \tau_m - \tau_L(\omega), \qquad \dot\omega \ge 0 \ \text{when}\ \omega = 0
\qquad\text{(MOT-4)}
```

**Steady speed, constant load.** Setting $`\dot\omega = 0`$ with the current unsaturated:

```math
\omega_{ss} = K_v\left(V - R\left(i_0 + K_v\,\tau_L\right)\right)
\qquad\text{(MOT-5)}
```

**Small-signal time constant.** Linearizing (MOT-4) about an operating point, back-EMF opposes a
change of speed with $`1/(RK_v^2)`$ N·m per rad/s, and the load adds its slope:

```math
\tau = \frac{J}{1/(R K_v^2) + d\tau_L/d\omega}
\qquad\text{(MOT-6)}
```

**Steady speed, propeller load** $`\tau_L = k\,\omega^2`$. The positive root of
$`k\omega^2 + a\omega - c = 0`$, written without cancellation:

```math
\omega_{ss} = \frac{2c}{a + \sqrt{a^2 + 4kc}}, \qquad a = \frac{1}{R K_v^2}, \qquad
c = \frac{V/R - i_0}{K_v}
\qquad\text{(MOT-7)}
```

**Lag tier.** With duty cycle $`d`$, top speed $`\omega_{\max}`$ and time constant $`\tau_m`$:

```math
\tau_m\,\dot\omega = d\,\omega_{\max} - \omega
\qquad\text{(MOT-8)}
```

**ESC.**
- **Saturation:** the command $`u_k`$ at tick $`k`$ is held to $`[0, 1]`$; a command that is
  not a number reads as 0.
- **Latency:** delayed by $`L = \operatorname{round}(t_{\text{latency}}/h)`$ ticks, halves
  rounded up.
- **Slew limit:** moved toward by at most $`s\,h`$ a tick, where $`s`$ is the slew rate in full
  scale per second.

```math
d_k = \min\!\left(\max\left(u_{k-L},\ d_{k-1} - s\,h\right),\ d_{k-1} + s\,h\right), \qquad
V = d_k\,V_s
\qquad\text{(MOT-9)}
```

**Power.** Electrical power in, mechanical power out, and the current drawn from the supply
through a lossless ESC; the difference is copper and friction loss:

```math
P_e = V i, \qquad P_m = \tau_m\,\omega, \qquad i_s = d\,i, \qquad
V i = \tau_m\,\omega + i^2 R + i_0\,\omega/K_v
\qquad\text{(MOT-10)}
```

### Saturation and braking

These closed forms follow from (MOT-2) to (MOT-4) under a constant load $`\tau_L`$:

- **Coasting without braking.** When the voltage falls below the back-EMF $`\omega/K_v`$, the
  current stops at zero instead of reversing. Only friction and the load slow the rotor, at a
  constant rate:

  ```math
  \omega(t) = \omega_0 - \left(\frac{i_0}{K_v} + \tau_L\right)\frac{t}{J}
  \qquad\text{(MOT-11)}
  ```

- **Active braking.** At zero voltage the current reverses, $`i = -\omega/(RK_v)`$, and the
  back-EMF brakes the rotor exponentially toward a negative speed it never reaches. It stops at
  zero:

  ```math
  \omega(t) = (\omega_0 + b\tau)\,e^{-t/\tau} - b\tau, \qquad \tau = J R K_v^2, \qquad
  b = \frac{i_0/K_v + \tau_L}{J}
  \qquad\text{(MOT-12)}
  ```

- **Current limit.** While the free current exceeds the limit, the current is the limit, and the
  rotor accelerates at a constant rate:

  ```math
  \omega(t) = \omega_0 + \left(\frac{i_{\max} - i_0}{K_v} - \tau_L\right)\frac{t}{J}
  \qquad\text{(MOT-13)}
  ```

The duty cycle saturates at 0 and 1, so the voltage never exceeds the supply.

## Validity range

- **Speeds** from rest up; the model does not turn a rotor backward.
- **Step size:** the physics step must resolve the mechanical time constant. RK4's error per
  step is about $`(h/\tau)^5/120`$ of the change, below 10⁻¹¹ at 1 kHz for the 76 ms of the test
  motor. Time constants of a few milliseconds still integrate stably at 1 kHz, but less
  accurately.
- **Electrical tier:** needs $`J > 0`$. An airframe whose rotor has an electrical motor must give
  the motor's inertia, the propeller's, or both.
- **Latency** must be a whole number of ticks that fits in 32 bits at the run's physics rate. A
  longer one is refused when the drive train is built.

## Parameters

| Symbol | Meaning | Unit | Airframe field | Source |
|---|---|---|---|---|
| $`K_v`$ | Speed constant | rad/(s·V) | `motors.<key>.kv` (`rpm/V`) | Airframe file |
| $`R`$ | Winding resistance | Ω | `motors.<key>.resistance` | Airframe file |
| $`i_0`$ | No-load current | A | `motors.<key>.no_load_current` | Airframe file |
| $`J`$ | Inertia turned | kg·m² | `motors.<key>.inertia` plus `propellers.<key>.inertia` | Airframe file |
| $`i_{\max}`$ | Current limit | A | the smaller of `motors.<key>.max_current` and `escs.<key>.max_current` | Airframe file |
| $`\tau_m`$ | Lag tier's time constant | s | `motors.<key>.time_constant` | Airframe file |
| $`\omega_{\max}`$ | Lag tier's top speed | rad/s | `motors.<key>.max_speed` | Airframe file |
| $`L`$ | ESC latency | ticks | `escs.<key>.latency`, rounded at the run's rate | Airframe file |
| $`s`$ | ESC slew rate | 1/s | `escs.<key>.slew_rate` | Airframe file |
| — | Braking | `active` or `none` | `escs.<key>.braking` | Airframe file |
| $`V_s`$ | Supply voltage | V | from the battery ([#20](https://github.com/MatthewK84/MJKPhysics/issues/20)) | Battery model |

A rotor without an ESC gets an ideal one: no latency, no slew limit and active braking, limited
only by the motor's own `max_current`.

## Implementation notes

- **The ESC is discrete.** `EscState::update` runs before each step with the tick's command, and
  its duty cycle is held for the step. That keeps the motor's equations smooth inside the
  integrator's stages, and real ESCs update discretely too.
  - **Delay line:** latency is a ring buffer sized when the ESC starts. A test makes 10⁵ updates
    under a counting allocator and finds no allocation.
  - **Slew limit:** it clamps the target with `min` and `max`, so the duty cycle lands exactly
    on the target once it is within one step.
- **The motor is a system.** `MotorSystem` implements `System` and `LieSystem`. The chart RKMK4
  steps in is plain addition, so every integrator treats the motor alike.
  - **Projection:** after each step it holds the speed at zero or above.
  - **At rest:** a rotor at rest under a net torque against its spin stays at rest.
- **A NaN is never hidden.** The current limit passes a NaN through instead of replacing it with
  a limit, and the speed's projection does the same.
- **Latency is converted without an `as` cast,** which the workspace forbids. The rounded number
  of ticks is decoded from the double's bits.

## Validation

`mjk validate motor` runs the models through the simulation loop and compares each with its
closed form. That means `World` with RKMK4 at 1 kHz, the ESC updated every tick.
- **The test motor** is an illustrative 2212-class outrunner: 920 rpm/V, 0.117 Ω and 0.4 A,
  turning 8.5 × 10⁻⁵ kg·m² from an 11.1 V supply.
- **Each tolerance** is ten times an error estimate computed before the run, rounded up to a
  power of ten, as in the [analytic suite](../validation/README.md#analytic-checks).

{{#include ../validation/motor.md}}

*Generated by `mjk validate motor`. A test fails if this table no longer matches the code, and
CI requires the WebAssembly build to print the same bytes.*

**The acceptance criteria are rows 1–3 and 5.**
- **Steady speed under a constant load** matches (MOT-5) to 10⁻¹¹ rad/s.
- **A 1 % step at the propeller operating point** settles with a time constant
  5.3 × 10⁻⁴ from the linearized (MOT-6), relative. That difference is the quadratic load's second-order term,
  which the estimate predicts at 8 × 10⁻⁴.

**Every error is at or below its estimate.** Rows 10 and 12 have a constant acceleration, so RK4
integrates them exactly and only rounding remains. The ESC's timing is exact to the tick.

Unit tests in `mjk-aero` check each equation, both braking modes, the current limit and the ESC's
rounding at every boundary. Mutation testing catches every viable mutant of the propulsion code.
Two are excluded as strictly equivalent, with their reasons in `.cargo/mutants.toml`.

## Limitations

- **No inductance:** the current is quasi-static, as Assumption 1 says.
- **No motor dynamics beyond the DC equivalent:** no torque ripple, cogging, demagnetization or
  heating.
- **No reverse rotation:** reversing motors, as in 3D flight modes, are not modeled.
- **The lag tier ignores supply voltage and load.** Its time constant and top speed are what was
  measured for one propeller and one battery.
- **The ESC's latency is quantized to whole physics steps.** At 1 kHz that is up to half a
  millisecond of difference.

## References

- M. Drela, "First-Order DC Electric Motor Model," MIT Aero & Astro, February 2007. The
  $`K_v`$, $`R`$, $`i_0`$ motor model this page uses.
