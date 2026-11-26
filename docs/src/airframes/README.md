# Airframes

Four aircraft are modeled, chosen for the quality of their public data. Each airframe file gives
every parameter a unit, a source and, where known, an uncertainty. Parameter pages will be
generated from those files ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)), so the
book can never disagree with what the simulator loads.

| Airframe | Class | Primary sources | Issue |
|---|---|---|---|
| Bitcraze Crazyflie 2.x | Nano quadrotor | Förster (2015); Eschmann, Albani & Loianno (2024) | [#29](https://github.com/MatthewK84/MJKPhysics/issues/29) |
| Holybro X500 V2 | 500 mm quadrotor | PX4 Gazebo `x500` model; Holybro specifications | [#30](https://github.com/MatthewK84/MJKPhysics/issues/30) |
| Aerosonde | Small fixed-wing | Beard & McLain (2012); BYU `mavsim_public` | [#50](https://github.com/MatthewK84/MJKPhysics/issues/50) |
| Skywalker X8 | Flying wing | Gryte et al. (2018); Reinhardt et al. (2022); Løw-Hansen et al. (2025) | [#51](https://github.com/MatthewK84/MJKPhysics/issues/51) |

Where sources disagree, the airframe page lists every value with its source, and states which one
the simulator uses and why.
