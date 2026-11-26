# SO(3) maps

- **Issue:** [#9](https://github.com/MatthewK84/MJKPhysics/issues/9)
- **Code:** `mjk_math::so3`, `mjk_math::EulerAngles`, `Rotation::{exp, log, plus, minus, slerp, from_matrix, to_euler}`
- **Equation prefix:** `SO3`

## Summary

The exponential and logarithm maps connect rotation vectors, three numbers that can be added and
scaled, to rotations, which cannot. Integrators (#11, #12), interpolation and attitude errors are
all built on them. This page also covers the SO(3) Jacobians, the $`\oplus`$ and $`\ominus`$
operators, spherical interpolation, conversion from rotation matrices, and Euler-angle extraction.
Notation and conventions follow the [conventions page](conventions.md); Hamilton quaternions,
scalar first.

## Assumptions

1. Rotation vectors are $`\boldsymbol\theta = \theta\,\mathbf{u}`$ with unit axis $`\mathbf{u}`$ and
   angle $`\theta`$ in radians, right-handed.
2. Increments are applied on the right, in the local frame (CONV-10).
3. Every transcendental function comes from `libm`, so results are bit-identical on every
   platform.

## Exponential and logarithm

```math
\mathrm{Exp}(\boldsymbol\theta) = \Big(\cos\tfrac{\theta}{2},\ \tfrac{\sin(\theta/2)}{\theta}\,\boldsymbol\theta\Big)
\qquad\text{(SO3-1)}
```

```math
\mathrm{Log}(\mathbf{q}) = \frac{2\,\mathrm{atan2}(\lVert\mathbf{q}_v\rVert,\ q_0)}{\lVert\mathbf{q}_v\rVert}\,\mathbf{q}_v,
\qquad \mathbf{q} \leftarrow -\mathbf{q}\ \text{if}\ q_0 < 0
\qquad\text{(SO3-2)}
```

The sign flip picks the shortest rotation, so the angle of $`\mathrm{Log}(\mathbf{q})`$ is in
$`[0, \pi]`$, and $`\mathbf{q}`$ and $`-\mathbf{q}`$ give the same answer. The `atan2` form stays
accurate near $`\pi`$, where $`\arccos q_0`$ would lose half its digits.

## Small angles

Each ratio is evaluated in the form that keeps full precision.

```math
\frac{\sin(\theta/2)}{\theta} \approx \tfrac12 - \tfrac{\theta^2}{48} + \tfrac{\theta^4}{3840},
\qquad
\frac{\theta - \sin\theta}{\theta^3} = \sum_{k\ge0}\frac{(-1)^k\,\theta^{2k}}{(2k+3)!},
\qquad
\frac{1}{\theta^2} - \frac{\cot(\theta/2)}{2\theta} = \sum_{n\ge1}\frac{(-1)^{n+1}B_{2n}\,\theta^{2n-2}}{(2n)!}
\qquad\text{(SO3-3)}
```

| Quantity | Problem with the closed form | What the code does |
|---|---|---|
| $`\sin(\theta/2)/\theta`$ | 0/0 at $`\theta = 0`$ only | Series below $`10^{-3}`$ rad; closed form above |
| $`2\,\mathrm{atan2}(n, q_0)/n`$ | 0/0 at $`n = 0`$ only | Series below $`n = 10^{-6}`$ |
| $`(1-\cos\theta)/\theta^2`$ | Cancellation: loses about $`2\varepsilon/\theta^2`$ | Computed as $`2\,(\sin(\theta/2)/\theta)^2`$, with no cancellation anywhere |
| $`(\theta-\sin\theta)/\theta^3`$ | Cancellation: loses about $`6\varepsilon/\theta^2`$, about 9 digits at $`\theta = 10^{-3}`$ | 8-term series below 1 rad |
| $`1/\theta^2 - \cot(\theta/2)/(2\theta)`$ | Cancellation: loses about $`12\varepsilon/\theta^2`$ | 11-term series below 1 rad ($`B_{2n}`$ are the Bernoulli numbers) |

The series coefficients were generated exactly in rational arithmetic. A unit test checks all
three coefficients against 40-digit references at eight angles from $`10^{-6}`$ to 3 rad, to a
relative error below $`10^{-15}`$. The obvious closed forms fail that test by nine orders of
magnitude at the $`\theta \approx 10^{-3}`$ of a typical integrator step.

## Jacobians

```math
\mathbf{J}_r(\boldsymbol\theta) = \mathbf{I} - \frac{1-\cos\theta}{\theta^2}[\boldsymbol\theta]_\times + \frac{\theta-\sin\theta}{\theta^3}[\boldsymbol\theta]_\times^2
\qquad\text{(SO3-4)}
```

```math
\mathbf{J}_r^{-1}(\boldsymbol\theta) = \mathbf{I} + \tfrac12[\boldsymbol\theta]_\times + \Big(\frac{1}{\theta^2} - \frac{\cot(\theta/2)}{2\theta}\Big)[\boldsymbol\theta]_\times^2
\qquad\text{(SO3-5)}
```

```math
\mathbf{J}_l(\boldsymbol\theta) = \mathbf{J}_r(-\boldsymbol\theta) = \mathbf{J}_r(\boldsymbol\theta)^\top,
\qquad
\mathbf{J}_l^{-1}(\boldsymbol\theta) = \mathbf{J}_r^{-1}(-\boldsymbol\theta)
\qquad\text{(SO3-6)}
```

They are defined by how a small change in the rotation vector moves the rotation:

```math
\mathrm{Exp}(\boldsymbol\theta + \delta\boldsymbol\theta) \approx \mathrm{Exp}(\boldsymbol\theta)\,\mathrm{Exp}\big(\mathbf{J}_r(\boldsymbol\theta)\,\delta\boldsymbol\theta\big),
\qquad
\mathrm{Log}\big(\mathrm{Exp}(\boldsymbol\theta)\,\mathrm{Exp}(\delta\boldsymbol\theta)\big) \approx \boldsymbol\theta + \mathbf{J}_r^{-1}(\boldsymbol\theta)\,\delta\boldsymbol\theta
\qquad\text{(SO3-7)}
```

The Lie-group integrator (#12) uses $`\mathbf{J}_r^{-1}`$. Writing the inverse coefficient with
$`\cot(\theta/2)`$ keeps it finite at $`\theta = \pi`$. The Jacobians are valid for
$`\theta < 2\pi`$.

## Composition operators and interpolation

```math
\mathbf{R}\oplus\boldsymbol\tau = \mathbf{R}\,\mathrm{Exp}(\boldsymbol\tau),
\qquad
\mathbf{S}\ominus\mathbf{R} = \mathrm{Log}(\mathbf{R}^{-1}\mathbf{S}),
\qquad
\mathbf{R}\oplus(\mathbf{S}\ominus\mathbf{R}) = \mathbf{S}
\qquad\text{(SO3-8)}
```

```math
\mathrm{slerp}(\mathbf{R}_0, \mathbf{R}_1, t) = \mathbf{R}_0 \oplus t\,(\mathbf{R}_1 \ominus \mathbf{R}_0)
\qquad\text{(SO3-9)}
```

The vector $`\boldsymbol\tau`$ is in the rotation's `From` frame, so in code
`q_wb.plus(tau_b)` type-checks only with a body-frame increment. Interpolation follows the
shortest arc at a constant angular rate.

## From a rotation matrix

Shepperd's method takes the square root of the largest of
$`4q_0^2 = 1 + \mathrm{tr}\,\mathbf{R}`$, $`4q_1^2 = 1 + 2R_{11} - \mathrm{tr}\,\mathbf{R}`$,
$`4q_2^2 = 1 + 2R_{22} - \mathrm{tr}\,\mathbf{R}`$ and $`4q_3^2 = 1 + 2R_{33} - \mathrm{tr}\,\mathbf{R}`$.
It then recovers the other three components from sums and differences of off-diagonal entries.
For example, when $`q_0`$ is the largest:

```math
q_0 = \tfrac12\sqrt{1+\mathrm{tr}\,\mathbf{R}},\quad
q_1 = \frac{R_{32}-R_{23}}{4q_0},\quad
q_2 = \frac{R_{13}-R_{31}}{4q_0},\quad
q_3 = \frac{R_{21}-R_{12}}{4q_0}
\qquad\text{(SO3-10)}
```

Choosing the largest component avoids dividing by a small number. The input must be a proper
rotation, orthonormal to $`10^{-9}`$ and with determinant $`+1`$; anything else is rejected.

## Euler angles

The definition and the gimbal-lock convention are in the [conventions](conventions.md#33-euler-angles-input-and-output-only)
(CONV-11, CONV-12). The extraction is arranged so that the rebuilt rotation is exact, even close to
±90° of pitch:

```math
\begin{aligned}
\psi &= \mathrm{atan2}(R_{21}, R_{11}), \qquad \theta = \mathrm{atan2}\big(-R_{31},\ \sqrt{R_{11}^2+R_{21}^2}\big), \\
\phi &= \mathrm{atan2}(\sin\psi\,R_{13} - \cos\psi\,R_{23},\ \cos\psi\,R_{22} - \sin\psi\,R_{12})
\end{aligned}
\qquad\text{(SO3-11)}
```

The textbook roll, $`\mathrm{atan2}(R_{32}, R_{33})`$, divides two entries that both shrink with
$`\cos\theta`$. Near gimbal lock, yaw and roll then pick up independent errors of about
$`\varepsilon/\cos\theta`$. The property tests caught this: at $`\cos\theta \approx 2\times10^{-6}`$
the rebuilt rotation was off by $`5\times10^{-11}`$. Computing roll from the yaw already found
uses entries that don't shrink, so roll absorbs any error in yaw, and the round trip stays at
rounding level, about $`10^{-16}`$.

Gimbal lock is declared only when $`\cos\theta < 10^{-15}`$, just above rounding noise, which
catches pitch at exactly ±90° in floating point. There, roll is zero and yaw is
$`\mathrm{atan2}(-R_{12}, R_{22})`$. A wider threshold would only discard information: with
$`10^{-10}`$, the round-trip error inside the threshold grew to $`10^{-10}`$.

## Validation

| Check | Tolerance | Result |
|---|---|---|
| $`\mathrm{Log}(\mathrm{Exp}(\boldsymbol\theta)) = \boldsymbol\theta`$ for $`\lvert\boldsymbol\theta\rvert`$ log-uniform in $`[10^{-12}, \pi - 10^{-6}]`$ | $`10^{-12}`$ absolute and relative | Pass |
| $`\mathrm{Exp}(\mathrm{Log}(\mathbf{q})) = \mathbf{q}`$; $`\mathrm{Log}`$ takes the short way; $`\mathrm{Log}(-\mathbf{q}) = \mathrm{Log}(\mathbf{q})`$ | $`8\varepsilon`$ | Pass |
| $`\mathbf{J}_r\mathbf{J}_r^{-1} = \mathbf{I}`$, $`\mathbf{J}_l = \mathbf{J}_r^\top = \mathbf{R}\,\mathbf{J}_r`$ | $`10^{-13}`$ | Pass |
| SO3-7 by finite differences, $`\lvert\delta\boldsymbol\theta\rvert = 10^{-5}`$ | $`10^{-9}`$ (second-order) | Pass |
| $`\oplus`$ and $`\ominus`$ invert each other | $`10^{-12}`$ | Pass |
| slerp: endpoints, constant rate, short arc | $`10^{-13}`$ | Pass |
| Euler round trip, pitch at least $`10^{-3}`$ rad from ±90° | $`10^{-11}`$ per angle | Pass |
| Euler-to-rotation round trip for all pitch, including exactly ±90° | $`10^{-14}`$ | Pass |
| Euler matches $`\mathbf{R}_z\mathbf{R}_y\mathbf{R}_x`$ | $`8\varepsilon`$ | Pass |
| Shepperd round trip; scaled and reflected matrices rejected | $`8\varepsilon`$ | Pass |
| Jacobian coefficients against 40-digit references | $`10^{-15}`$ relative | Pass |

The property tests run 512 random cases per property on every CI run. Every one of eight
deliberate mutants is caught: sign errors in Exp, Log, $`\mathbf{J}_r`$ and Shepperd's method,
a dropped factor, a perturbed series coefficient, the textbook roll formula, and the old lock
threshold.

## References

- J. Solà, J. Deray, D. Atchuthan, "A micro Lie theory for state estimation in robotics,"
  arXiv:1812.01537, 2018.
- T. D. Barfoot, *State Estimation for Robotics*, Cambridge University Press, 2017.
- S. W. Shepperd, "Quaternion from rotation matrix," *Journal of Guidance and Control*, 1978.
- J. Solà, "Quaternion kinematics for the error-state Kalman filter," arXiv:1711.02508, 2017.
