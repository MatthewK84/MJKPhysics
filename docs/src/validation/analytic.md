| # | Check | Closed form | Simulated | Error | Tolerance | Result |
|---:|---|---:|---:|---:|---:|---|
| 1 | Free fall: distance fallen in 10 s | 490.3325000 | 490.3325000 | 5.3e-11 m | 1e-7 m | Pass |
| 2 | Free fall: speed after 10 s | 98.06650000 | 98.06650000 | 1.3e-11 m/s | 1e-8 m/s | Pass |
| 3 | Linear drag: position after 5 s | 70.49587964 | 70.49587964 | 2.4e-13 m | 1e-9 m | Pass |
| 4 | Linear drag: velocity after 5 s | 24.37355023 | 24.37355023 | 1.4e-14 m/s | 1e-9 m/s | Pass |
| 5 | Axisymmetric body: precession rate in body axes | 9.000000000 | 9.000000000 | 4.9e-10 rad/s | 1e-8 rad/s | Pass |
| 6 | Axisymmetric body: precession rate in space | 21.02379604 | 21.02379604 | 4.9e-10 rad/s | 1e-7 rad/s | Pass |
| 7 | Axisymmetric body: attitude after 10 s | R(t), Eq. (VAL-6) | angle from it | 2.4e-10 rad | 1e-6 rad | Pass |
| 8 | Intermediate axis: time between flips | 5.926982582 | 5.926982582 | 7.1e-13 (relative) | 1e-6 | Pass |
| 9 | Intermediate axis: angular momentum over 60 s | constant | largest change | 3.4e-14 (relative) | 1e-8 | Pass |
| 10 | Intermediate axis: kinetic energy over 60 s | constant | largest change | 7.0e-14 (relative) | 1e-8 | Pass |
| 11 | Convergence order: explicit Euler, translation (linear drag) | 1 | 0.99 | 0.01 | 0.2 | Pass |
| 12 | Convergence order: explicit Euler, rotation (precession) | 1 | 0.99 | 0.01 | 0.2 | Pass |
| 13 | Convergence order: semi-implicit Euler, translation (linear drag) | 1 | 1.00 | 0.00 | 0.2 | Pass |
| 14 | Convergence order: semi-implicit Euler, rotation (precession) | 1 | 0.99 | 0.01 | 0.2 | Pass |
| 15 | Convergence order: RK4, translation (linear drag) | 4 | 4.02 | 0.02 | 0.2 | Pass |
| 16 | Convergence order: RK4, rotation (precession) | 4 | 4.00 | 0.00 | 0.2 | Pass |
| 17 | Convergence order: RKMK4, translation (linear drag) | 4 | 4.02 | 0.02 | 0.2 | Pass |
| 18 | Convergence order: RKMK4, rotation (precession) | 4 | 4.00 | 0.00 | 0.2 | Pass |

*Vector checks show magnitudes; their error is the distance between the vectors.*

Why each tolerance:

- **1.** RKMK4 integrates translation as RK4 does, and RK4's stages integrate a constant acceleration exactly, so only rounding remains: at most N·ε·z = 1.1e-9 over N = 10000 steps.
- **2.** RKMK4 integrates translation as RK4 does, and RK4's stages integrate a constant acceleration exactly, so only rounding remains: at most N·ε·v = 2.2e-10 over N = 10000 steps.
- **3–4.** RKMK4 integrates translation as RK4 does. Its truncation, (T/τ)(h/τ)⁴/120 = 4e-17 of the 231 m transient, is far below rounding, N·ε·|x| = 7.8e-11 m over N = 5000 steps.
- **5.** RK4-type phase error over the run, N·(h·ω)⁵/120 divided by T = 10 s, with ω = 9.00 rad/s: 4.9e-10 rad/s.
- **6.** RK4-type phase error over the run, N·(h·ω)⁵/120 divided by T = 10 s, with ω = 12.04 rad/s: 2.2e-9 rad/s.
- **7.** RK4-type phase error, N·(h·ω)⁵/120 with ω = 12.04 rad/s, plus rounding: 2.2e-8 rad.
- **8.** Near the separatrix (k′² = 5.6e-4) an invariant error δ moves the interval by about δ/(2K·k′²) = 175δ; with δ = 1.3e-10, 2.4e-8.
- **9–10.** RK4-type truncation N·(h·ω)⁵/120 plus rounding N·ε, with ω = 3.00 rad/s: 1.3e-10.
- **11–18.** Slope of ln(error) against ln(h) over h = 1/16 to 1/128 s. At the largest step h·ω = 0.11 and h/τ = 0.0625: the next term of the error expansion shifts the slope by about that much, and ±0.2 is about twice it.
