# Models

Each physical model gets one page that follows the [model page template](../templates/model-page.md):
equations with IDs, assumptions, validity range, parameters, implementation notes, validation and
references.

Equation IDs use a prefix unique to their page. The registry below reserves prefixes for the
planned pages so IDs never collide.

| Model | Equation prefix | Issue |
|---|---|---|
| Conventions and notation | `CONV` | [#5](https://github.com/MatthewK84/MJKPhysics/issues/5), [page](../conventions.md) |
| SO(3) maps | `SO3` | [#9](https://github.com/MatthewK84/MJKPhysics/issues/9), [page](../so3.md) |
| Rigid-body dynamics | `RB` | [#10](https://github.com/MatthewK84/MJKPhysics/issues/10), [page](rigid-body.md) |
| Integrators, including the Lie-group method | `INT` | [#11](https://github.com/MatthewK84/MJKPhysics/issues/11), [#12](https://github.com/MatthewK84/MJKPhysics/issues/12) |
| Atmosphere and gravity | `ATM` | [#15](https://github.com/MatthewK84/MJKPhysics/issues/15) |
| Brushless motor and ESC | `MOT` | [#19](https://github.com/MatthewK84/MJKPhysics/issues/19) |
| Battery | `BAT` | [#20](https://github.com/MatthewK84/MJKPhysics/issues/20) |
| Blade element momentum theory | `BEMT` | [#21](https://github.com/MatthewK84/MJKPhysics/issues/21) |
| Rotor in edgewise and oblique flow | `ROTF` | [#22](https://github.com/MatthewK84/MJKPhysics/issues/22) |
| Rotor fidelity tiers | `ROTT` | [#23](https://github.com/MatthewK84/MJKPhysics/issues/23) |
| Multirotor body aerodynamics and ground effect | `BODY` | [#24](https://github.com/MatthewK84/MJKPhysics/issues/24) |
| Control allocation | `ALLOC` | [#25](https://github.com/MatthewK84/MJKPhysics/issues/25) |
| Multirotor controllers | `MRC` | [#26](https://github.com/MatthewK84/MJKPhysics/issues/26), [#27](https://github.com/MatthewK84/MJKPhysics/issues/27) |
| Trajectories | `TRAJ` | [#28](https://github.com/MatthewK84/MJKPhysics/issues/28) |
| Fixed-wing aerodynamics | `FWA` | [#44](https://github.com/MatthewK84/MJKPhysics/issues/44) |
| Control-surface actuators | `ACT` | [#45](https://github.com/MatthewK84/MJKPhysics/issues/45) |
| Fixed-wing propulsion | `FWP` | [#46](https://github.com/MatthewK84/MJKPhysics/issues/46) |
| Trim | `TRIM` | [#47](https://github.com/MatthewK84/MJKPhysics/issues/47) |
| Linearization and modes | `LIN` | [#48](https://github.com/MatthewK84/MJKPhysics/issues/48) |
| Fixed-wing autopilot | `FWC` | [#49](https://github.com/MatthewK84/MJKPhysics/issues/49) |
| Wind and turbulence | `WIND` | [#53](https://github.com/MatthewK84/MJKPhysics/issues/53) |
| Collision detection | `COL` | [#55](https://github.com/MatthewK84/MJKPhysics/issues/55) |
| Contact solver | `CON` | [#57](https://github.com/MatthewK84/MJKPhysics/issues/57) |
| Joints and slung payloads | `JNT` | [#59](https://github.com/MatthewK84/MJKPhysics/issues/59) |
| Inertial sensors | `IMU` | [#72](https://github.com/MatthewK84/MJKPhysics/issues/72) |
| Other sensors | `SNS` | [#73](https://github.com/MatthewK84/MJKPhysics/issues/73) |

A new model reserves its prefix here in the same pull request that adds its page.
