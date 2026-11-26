# Conventions and notation

This page is the single authority for reference frames, rotation conventions, signs, units and
symbols in MJKPhysics. Every model page links here and uses these symbols; code follows the same
names. If a source paper uses a different convention, its model page states the mapping
explicitly instead of silently adopting the paper's convention.

Equations carry stable IDs such as (CONV-3) so code comments can cite them.

## 1. Notation

| Kind | Typeface | Example |
|---|---|---|
| Scalar | italic | $m$, $V_a$, $\alpha$ |
| Vector | bold lowercase | $\mathbf{v}$, $\boldsymbol{\omega}$ |
| Matrix | bold uppercase | $\mathbf{R}$, $\mathbf{J}$ |
| Quaternion | bold lowercase $\mathbf{q}$ with components $(q_0, q_1, q_2, q_3)$, scalar first | $\mathbf{q}_{WB}$ |
| Frame | upright capital | $W$, $B$, $S$, $A$ |

- A superscript names the frame a vector is **resolved in**: $\mathbf{v}^B$ is $\mathbf{v}$
  expressed in body axes.
- A rotation or quaternion with two subscripts maps coordinates from the right frame to the left
  frame: $\mathbf{x}^W = \mathbf{R}_{WB} \mathbf{x}^B$. Subscripts cancel inward under
  composition: $\mathbf{R}_{WS} = \mathbf{R}_{WB} \mathbf{R}_{BS}$.
- $[\mathbf{a}]_\times$ is the skew-symmetric matrix with $[\mathbf{a}]_\times\mathbf{b} = \mathbf{a}\times\mathbf{b}$.
- $\otimes$ is the quaternion product and $\mathbf{q}^{\ast}$ the conjugate.
- A dot is a time derivative taken in the world frame unless stated otherwise.

In code, `q_wb` is $\mathbf{q}_{WB}$ ("world from body"), and vectors carry their frame in their
type, as in `Vec3<Body>` and `Vec3<World>` ([#8](https://github.com/MatthewK84/MJKPhysics/issues/8)).

## 2. Reference frames

| Frame | Symbol | Code marker | Origin | Axes |
|---|---|---|---|---|
| World | $W$ | `World` | Scenario reference point on the ground | **North, East, Down** (local level) |
| Body | $B$ | `Body` | Vehicle centre of mass | **Forward, Right, Down** |
| Stability | $S$ | `Stability` | Centre of mass | $\mathbf{x}_S$ along the relative wind projected onto the body $x$–$z$ plane; $\mathbf{y}_S = \mathbf{y}_B$ |
| Wind (air-relative) | $A$ | `Wind` | Centre of mass | $\mathbf{x}_A$ along the relative wind; $\mathbf{z}_A$ in the body plane of symmetry |
| Render | $\mathcal{R}$ | viewer only | Same as $W$ | East, Up, South (Y-up, right-handed) |

The wind axes are written $A$, not $W$, so they cannot be confused with the world frame. All
frames are right-handed.

### 2.1 World frame and the flat-earth assumption

The world frame is a local-level North-East-Down frame fixed to the ground at a reference point
with WGS-84 geodetic coordinates $(\text{lat}_0, \text{lon}_0, h_0)$. The equations of motion treat
it as inertial and flat: no earth rotation, no curvature, gravity along $+\mathbf{z}_W$.

This is appropriate for small unmanned aircraft operating within about 10 km of the reference
point. At that range:

| Neglected effect | Size at 10 km or 40 m/s |
|---|---|
| Earth curvature: surface drop $d^2/2R_E$ | 7.85 m |
| Change in gravity direction $d/R_E$ | 0.090° |
| Coriolis acceleration $2 \Omega_E V$ | $5.8\times10^{-3}$ m/s² (0.06 % of $g$) |

Centripetal acceleration from earth rotation is not neglected: it is already included in normal
gravity when the WGS-84 gravity model is selected
([#15](https://github.com/MatthewK84/MJKPhysics/issues/15)). Geodetic coordinates appear only at
the edges: GNSS output ([#73](https://github.com/MatthewK84/MJKPhysics/issues/73)) and terrain
placement ([#62](https://github.com/MatthewK84/MJKPhysics/issues/62)).

Altitude above the reference ellipsoid is $h = h_0 - p_D$, where $p_D$ is the Down component of
the position.

### 2.2 Body frame

The body frame is fixed to the airframe with its origin at the centre of mass: $\mathbf{x}_B$ out
of the nose, $\mathbf{y}_B$ out of the right side, $\mathbf{z}_B$ down. For a multirotor, "nose" is
the forward direction declared in the airframe file; for the quad-X layout it bisects motors 1
and 3 (section 6).

The rotation from body to world is defined by its columns, which are the body axes expressed in
world coordinates:

```math
\mathbf{R}_{WB} = \begin{bmatrix} \mathbf{x}_B^W & \mathbf{y}_B^W & \mathbf{z}_B^W \end{bmatrix},
\qquad \mathbf{x}^W = \mathbf{R}_{WB}\,\mathbf{x}^B,
\qquad \mathbf{R}_{BW} = \mathbf{R}_{WB}^{\top}
\tag{CONV-1}
```

### 2.3 Stability and wind axes

With the air-relative velocity resolved in body axes as $\mathbf{v}_a^B = (u_a, v_a, w_a)$ and
airspeed $V_a = \lVert \mathbf{v}_a \rVert$:

```math
\alpha = \operatorname{atan2}(w_a,\, u_a), \qquad \beta = \arcsin\!\left(\frac{v_a}{V_a}\right)
\tag{CONV-2}
```

```math
\mathbf{R}_{BS} = \mathbf{R}_y(-\alpha), \qquad \mathbf{R}_{SA} = \mathbf{R}_z(\beta), \qquad
\mathbf{v}_a^B = \mathbf{R}_{BS}\,\mathbf{R}_{SA} \begin{bmatrix} V_a \\ 0 \\ 0 \end{bmatrix}
\tag{CONV-3}
```

Positive $\alpha$ means the relative wind meets the aircraft from below. Positive $\beta$ means it
meets the aircraft from the right. Both angles are undefined as $V_a \to 0$; models that use them
must define their behaviour below a stated airspeed
([#44](https://github.com/MatthewK84/MJKPhysics/issues/44)).

### 2.4 Render frame

The viewer uses a right-handed Y-up frame. One module, and only that module, converts from world
coordinates:

```math
\mathbf{C} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 0 & -1 \\ -1 & 0 & 0 \end{bmatrix},
\qquad \mathbf{p}^{\mathcal{R}} = \mathbf{C}\,\mathbf{p}^W = (p_E,\, -p_D,\, -p_N)
\tag{CONV-4}
```

Because $\mathbf{C}$ is a proper rotation ($\det\mathbf{C} = +1$), a world-frame attitude maps to
$\mathbf{C} \mathbf{R} \mathbf{C}^{\top}$, which in quaternion form is

```math
\mathbf{q}^{\mathcal{R}} = (q_0,\; q_2,\; -q_3,\; -q_1)
\tag{CONV-5}
```

Meshes authored in body FRD coordinates go through the same $\mathbf{C}$, so positions and
orientations always convert the same way ([#34](https://github.com/MatthewK84/MJKPhysics/issues/34)).

## 3. Rotations and quaternions

### 3.1 Quaternion convention

MJKPhysics uses **Hamilton** quaternions ($ij = k$), stored **scalar first**:

```math
\mathbf{q} = \begin{bmatrix} q_0 \\ \mathbf{q}_v \end{bmatrix}
= \begin{bmatrix} \cos(\theta/2) \\ \mathbf{u}\,\sin(\theta/2) \end{bmatrix},
\qquad \lVert \mathbf{q} \rVert = 1
\tag{CONV-6}
```

This is a right-handed rotation by $\theta$ about the unit axis $\mathbf{u}$. The product is

```math
\mathbf{p}\otimes\mathbf{q} =
\begin{bmatrix}
p_0 q_0 - \mathbf{p}_v\cdot\mathbf{q}_v \\
p_0\,\mathbf{q}_v + q_0\,\mathbf{p}_v + \mathbf{p}_v\times\mathbf{q}_v
\end{bmatrix}
\tag{CONV-7}
```

The attitude state is the unit quaternion $\mathbf{q}_{WB}$. It acts on vectors as
$\mathbf{x}^W = \mathbf{q}_{WB}\otimes\mathbf{x}^B\otimes\mathbf{q}_{WB}^{\ast}$, using pure
quaternions $(0, \mathbf{x})$, and corresponds to

```math
\mathbf{R}_{WB} =
\begin{bmatrix}
1 - 2(q_2^2 + q_3^2) & 2(q_1 q_2 - q_0 q_3) & 2(q_1 q_3 + q_0 q_2) \\
2(q_1 q_2 + q_0 q_3) & 1 - 2(q_1^2 + q_3^2) & 2(q_2 q_3 - q_0 q_1) \\
2(q_1 q_3 - q_0 q_2) & 2(q_2 q_3 + q_0 q_1) & 1 - 2(q_1^2 + q_2^2)
\end{bmatrix}
\tag{CONV-8}
```

Composition follows the matrix order: $\mathbf{q}_{WS} = \mathbf{q}_{WB}\otimes\mathbf{q}_{BS}$.
$\mathbf{q}$ and $-\mathbf{q}$ are the same rotation. The state is never sign-flipped, to keep it
continuous in time, and attitude errors are compared with the rotation angle
$2\arccos\lvert(\mathbf{p}^{\ast}\otimes\mathbf{q})_0\rvert$, which is independent of sign.

**Why Hamilton, scalar first.** It matches PX4 and MAVLink and the Lie-group literature this
project builds on (Solà 2017; Solà, Deray & Atchuthan 2018), and its composition order equals the
matrix order. The JPL convention ($ij = -k$, usually scalar last)
reverses composition order and is common in older spacecraft-navigation papers; any source
using it is converted at the model boundary and the conversion is tested.

### 3.2 Attitude kinematics

The angular velocity of the body relative to the world, resolved in body axes, is
$\boldsymbol{\omega}^B = (p, q, r)$: roll, pitch and yaw rates. The italic scalar $q$ (pitch
rate) is unrelated to the bold quaternion $\mathbf{q}$.

```math
\dot{\mathbf{q}}_{WB} = \tfrac{1}{2}\,\mathbf{q}_{WB}\otimes\begin{bmatrix} 0 \\ \boldsymbol{\omega}^B \end{bmatrix},
\qquad
\dot{\mathbf{R}}_{WB} = \mathbf{R}_{WB}\,[\boldsymbol{\omega}^B]_\times
\tag{CONV-9}
```

The exponential map is $\operatorname{Exp}(\boldsymbol{\theta}) = (\cos\tfrac{\theta}{2},\,
\tfrac{\boldsymbol{\theta}}{\theta}\sin\tfrac{\theta}{2})$ with $\theta = \lVert\boldsymbol{\theta}\rVert$.
Increments and perturbations are applied **on the right**, in body axes:

```math
\mathbf{q}_{WB}(t + \Delta t) = \mathbf{q}_{WB}(t)\otimes\operatorname{Exp}(\boldsymbol{\omega}^B\,\Delta t)
\quad\text{for constant } \boldsymbol{\omega}^B
\tag{CONV-10}
```

Small-angle and $\theta\to 0$ handling of these maps is specified in
[#9](https://github.com/MatthewK84/MJKPhysics/issues/9).

### 3.3 Euler angles (input and output only)

Euler angles never appear in the state and their rates are never integrated. They exist for
human-facing input and output: scenario files, logs, the HUD. The sequence is **3-2-1**: yaw
$\psi$, then pitch $\theta$, then roll $\phi$, using elementary rotations that act on vectors:

```math
\mathbf{R}_{WB} = \mathbf{R}_z(\psi)\,\mathbf{R}_y(\theta)\,\mathbf{R}_x(\phi)
\tag{CONV-11}
```

```math
\mathbf{R}_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix},\;
\mathbf{R}_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix},\;
\mathbf{R}_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}
```

Ranges are $\psi\in(-\pi,\pi]$, $\theta\in[-\tfrac{\pi}{2},\tfrac{\pi}{2}]$, $\phi\in(-\pi,\pi]$.
Extraction, with $R_{ij}$ the entries of $\mathbf{R}_{WB}$:

```math
\theta = -\arcsin R_{31}, \qquad \phi = \operatorname{atan2}(R_{32}, R_{33}), \qquad \psi = \operatorname{atan2}(R_{21}, R_{11})
\tag{CONV-12}
```

At $\lvert\theta\rvert = \tfrac{\pi}{2}$ only $\psi \mp \phi$ is defined. The convention there
is $\phi = 0$ and $\psi = \operatorname{atan2}(-R_{12}, R_{22})$, which holds for both signs of
$\theta$.

### 3.4 Positive rotations and moments

By the right-hand rule about the body axes:

| Rotation or moment | Axis | Positive sense |
|---|---|---|
| Roll $\phi$, rate $p$, moment $M_x$ | $+\mathbf{x}_B$ | right wing down |
| Pitch $\theta$, rate $q$, moment $M_y$ | $+\mathbf{y}_B$ | nose up |
| Yaw $\psi$, rate $r$, moment $M_z$ | $+\mathbf{z}_B$ | nose right (clockwise seen from above) |

Moments are written $(M_x, M_y, M_z)$ rather than $(\ell, m, n)$ so they cannot be confused with
mass $m$. The aerodynamic coefficients keep their standard names $C_l$, $C_m$, $C_n$.

## 4. Forces, aerodynamics and controls

### 4.1 Aerodynamic forces

Lift and drag are defined in stability axes and side force in body axes, as in Beard & McLain
(2012, ch. 4):

- **Drag** $D$ acts along $-\mathbf{x}_S$.
- **Lift** $L$ acts along $-\mathbf{z}_S$ (upward in upright flight).
- **Side force** $Y$ acts along $+\mathbf{y}_B$.

Stevens, Lewis & Johnson define drag along the wind axis $-\mathbf{x}_A$. The two definitions
differ at order $\beta$, and each model page states which one its source uses.

Coefficients are nondimensionalised by the dynamic pressure $\bar{q} = \tfrac12\rho V_a^2$, the
reference area $S$, the span $b$ (roll and yaw) and the mean aerodynamic chord $\bar{c}$ (pitch).
Rates enter as $\tfrac{pb}{2V_a}$, $\tfrac{q\bar{c}}{2V_a}$ and $\tfrac{rb}{2V_a}$.

### 4.2 Control-surface deflections

Virtual controls are defined by the moment they produce. This matches the Beard & McLain
parameter set, so the Aerosonde derivatives are used without sign changes; every airframe's
validation checks the sign of each control derivative
([#50](https://github.com/MatthewK84/MJKPhysics/issues/50),
[#51](https://github.com/MatthewK84/MJKPhysics/issues/51)).

| Control | Symbol | Positive deflection | Primary effect |
|---|---|---|---|
| Elevator | $\delta_e$ | trailing edge down | nose-down pitching moment ($C_{m_{\delta_e}} < 0$) |
| Aileron | $\delta_a$ | left trailing edge down, right trailing edge up | right-wing-down rolling moment ($C_{l_{\delta_a}} > 0$) |
| Rudder | $\delta_r$ | trailing edge left | nose-left yawing moment ($C_{n_{\delta_r}} < 0$) |
| Throttle | $\delta_t$ | $0 \le \delta_t \le 1$ | — |

A **physical** surface is positive when its trailing edge moves down, or left for a vertical
surface. Mixed surfaces such as elevons and ruddervators are declared in the airframe file as a
linear map from $(\delta_e, \delta_a, \delta_r)$ to physical deflections
([#45](https://github.com/MatthewK84/MJKPhysics/issues/45)). With that definition,
$\delta_a = \tfrac12(\delta_{\text{left}} - \delta_{\text{right}})$ for conventional ailerons.

## 5. Gravity and atmosphere

Gravity acts along $+\mathbf{z}_W$ with magnitude $g_0 = 9.80665$ m/s² (standard gravity), or
WGS-84 normal gravity as a function of latitude and altitude when that model is selected.
Atmospheric quantities follow the U.S. Standard Atmosphere 1976: temperature $T$ in kelvin,
pressure $P$ in pascals, density $\rho$ in kg/m³, speed of sound $a$, dynamic viscosity $\mu$
([#15](https://github.com/MatthewK84/MJKPhysics/issues/15)).

## 6. Rotors and propellers

- **Geometry.** Each rotor $i$ has a hub position $\mathbf{r}_i^B$ relative to the centre of mass
  and a unit thrust axis $\mathbf{n}_i^B$; for a flat multirotor, $\mathbf{n}_i^B = -\mathbf{z}_B$.
- **Speed.** $\omega_i \ge 0$ is the rotor's speed magnitude in rad/s. RPM appears only in input
  and output (`_rpm` suffix).
- **Spin direction.** $s_i = +1$ for counter-clockwise and $s_i = -1$ for clockwise, both as seen
  from above (looking along $+\mathbf{z}_B$). The rotor's angular velocity is
  $s_i \omega_i \mathbf{n}_i$.
- **Yaw reaction.** The motor's reaction on the airframe opposes the torque that spins the rotor.
  A counter-clockwise rotor therefore produces a **positive** (nose-right) yaw moment:

  ```math
  \mathbf{M}_{\text{reaction},i}^B = -s_i\,Q_i\,\mathbf{n}_i^B
  \qquad\left(= +s_i\,Q_i\,\mathbf{z}_B \text{ for } \mathbf{n}_i = -\mathbf{z}_B\right)
  \tag{CONV-13}
  ```

  Here $Q_i \ge 0$ is the aerodynamic torque magnitude. To yaw right, speed up the
  counter-clockwise rotors.
- **Quad-X numbering** follows PX4: motor 1 front-right counter-clockwise, motor 2 rear-left
  counter-clockwise, motor 3 front-left clockwise, motor 4 rear-right clockwise. Airframe files
  list rotors with explicit IDs, so other layouts need no special cases.

### 6.1 Propeller and rotor coefficients

Two nondimensionalisations are in common use, and they differ by large constant factors. Propeller
data (UIUC Propeller Data Site; Brandt & Selig) uses revolutions per second $n$ and diameter $D$.
Rotorcraft theory (Leishman) uses $\Omega$ and radius $R$:

```math
C_{T,\text{prop}} = \frac{T}{\rho n^2 D^4},\quad C_{P,\text{prop}} = \frac{P}{\rho n^3 D^5},\quad J = \frac{V}{nD}
\qquad\qquad
C_{T,\text{rotor}} = \frac{T}{\rho A (\Omega R)^2},\quad C_{P,\text{rotor}} = \frac{P}{\rho A (\Omega R)^3}
\tag{CONV-14}
```

```math
C_{T,\text{rotor}} = \frac{4}{\pi^3}\,C_{T,\text{prop}}, \qquad C_{P,\text{rotor}} = \frac{4}{\pi^4}\,C_{P,\text{prop}}
\tag{CONV-15}
```

Code names them `ct_prop` and `ct_rotor` (likewise `cp_prop` and `cp_rotor`) and never uses a
bare `ct`.

## 7. Units, numbers and time

- **SI throughout**, with angles in radians. Degrees, RPM and other display units appear only in
  input and output.
- **Unit suffixes** on every physical field and variable whose unit is not obvious:

  | Suffix | Unit | Suffix | Unit |
  |---|---|---|---|
  | `_m` | metre | `_n` | newton |
  | `_m_s` | m/s | `_n_m` | newton-metre |
  | `_m_s2` | m/s² | `_pa` | pascal |
  | `_rad` | radian | `_k` | kelvin |
  | `_rad_s` | rad/s | `_v` | volt |
  | `_kg` | kilogram | `_a` | ampere |
  | `_kg_m2` | kg·m² | `_ohm` | ohm |
  | `_s` | second | `_deg`, `_rpm` | input/output only |

- **Floating point.** The engine computes in `f64`. Transcendental functions go through `libm` so
  native and WebAssembly builds agree bit for bit
  ([#14](https://github.com/MatthewK84/MJKPhysics/issues/14)).
- **Time.** Simulation time is an integer tick count; seconds are derived as $k \Delta t$ and
  never accumulated by repeated addition.

## 8. Interoperability

Conversions to external conventions happen only at the boundary: bridges, file import and export,
and the renderer.

| System | World / body frames | Quaternion | Conversion |
|---|---|---|---|
| MJKPhysics | NED / FRD | Hamilton, scalar first, body → world | — |
| PX4, MAVLink | NED / FRD | Hamilton, scalar first, body → NED | none |
| ArduPilot | NED / FRD | — | none for frames |
| ROS (REP 103) | ENU / FLU | Hamilton, stored $(x, y, z, w)$ | ENU→NED: $(N, E, D) = (y, x, -z)$; FLU→FRD: $(x, -y, -z)$ |
| three.js viewer | East-Up-South | Hamilton, $(x, y, z, w)$ | (CONV-4), (CONV-5) |

## 9. Symbol table

| Symbol | Meaning | Unit | Code |
|---|---|---|---|
| $\mathbf{p}^W = (p_N, p_E, p_D)$ | Position of the centre of mass | m | `position_m: Vec3<World>` |
| $\mathbf{v}^W$ | Velocity of the centre of mass relative to $W$ | m/s | `velocity_m_s: Vec3<World>` |
| $\mathbf{q}_{WB}$ | Attitude, body to world | — | `q_wb: UnitQuat` |
| $\boldsymbol{\omega}^B = (p, q, r)$ | Body angular velocity relative to $W$, in body axes | rad/s | `omega_rad_s: Vec3<Body>` |
| $\mathbf{v}_{\text{wind}}^W$ | Wind velocity | m/s | `wind_m_s: Vec3<World>` |
| $\mathbf{v}_a = \mathbf{v} - \mathbf{v}_{\text{wind}}$, $(u_a, v_a, w_a)$ | Air-relative velocity and its body components | m/s | `v_air_m_s: Vec3<Body>` |
| $V_a$ | Airspeed | m/s | `airspeed_m_s` |
| $\alpha$, $\beta$ | Angle of attack, sideslip (CONV-2) | rad | `alpha_rad`, `beta_rad` |
| $\psi, \theta, \phi$ | Yaw, pitch, roll (I/O only) | rad | `yaw_rad`, `pitch_rad`, `roll_rad` |
| $m$ | Mass | kg | `mass_kg` |
| $\mathbf{J}^B$ | Inertia tensor about the centre of mass, body axes | kg·m² | `inertia_kg_m2: Mat3` |
| $\mathbf{F}$, $\mathbf{M} = (M_x, M_y, M_z)$ | Force; moment about the centre of mass | N, N·m | `Wrench<F>` |
| $L$, $D$, $Y$ | Lift, drag, side force | N | `lift_n`, `drag_n`, `side_force_n` |
| $C_L, C_D, C_Y, C_l, C_m, C_n$ | Force and moment coefficients | — | `c_lift`, `c_drag`, … |
| $\bar{q} = \tfrac12\rho V_a^2$ | Dynamic pressure | Pa | `dynamic_pressure_pa` |
| $S$, $b$, $\bar{c}$ | Reference area, span, mean aerodynamic chord | m², m, m | `area_m2`, `span_m`, `chord_m` |
| $\delta_e, \delta_a, \delta_r, \delta_t$ | Elevator, aileron, rudder, throttle | rad, rad, rad, — | `elevator_rad`, … |
| $\omega_i$, $s_i$ | Rotor speed, spin direction | rad/s, — | `omega_rad_s`, `spin` |
| $T_i$, $Q_i$ | Rotor thrust, rotor torque magnitude | N, N·m | `thrust_n`, `torque_n_m` |
| $\rho$, $T$, $P$, $a$, $\mu$ | Density, temperature, pressure, speed of sound, viscosity | kg/m³, K, Pa, m/s, Pa·s | `density_kg_m3`, … |
| $g_0$ | Standard gravity, 9.80665 | m/s² | `G0_M_S2` |
| $\Delta t$, $k$ | Physics step, tick index | s, — | `dt_s`, `tick` |

## References

- J. Solà, "Quaternion kinematics for the error-state Kalman filter," arXiv:1711.02508, 2017.
- J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in robotics,"
  arXiv:1812.01537, 2018.
- B. L. Stevens, F. L. Lewis, E. N. Johnson, *Aircraft Control and Simulation*, 3rd ed., Wiley, 2015.
- R. W. Beard, T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University
  Press, 2012.
- J. G. Leishman, *Principles of Helicopter Aerodynamics*, 2nd ed., Cambridge University Press, 2006.
- PX4 documentation: coordinate frames, and multicopter geometry and motor numbering.
- ROS REP 103, "Standard Units of Measure and Coordinate Conventions."
