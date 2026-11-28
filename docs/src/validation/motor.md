| # | Check | Closed form | Simulated | Error | Tolerance | Result |
|---:|---|---:|---:|---:|---:|---|
| 1 | Steady speed, constant load 0 N·m | 1064.8893255 | 1064.8893255 | 1.0e-11 rad/s | 1e-8 rad/s | Pass |
| 2 | Steady speed, constant load 0.02 N·m | 1043.1698793 | 1043.1698793 | 1.0e-11 rad/s | 1e-8 rad/s | Pass |
| 3 | Steady speed, constant load 0.05 N·m | 1010.5907099 | 1010.5907099 | 8.2e-12 rad/s | 1e-8 rad/s | Pass |
| 4 | Steady speed, propeller load, 80 % duty | 767.7867127 | 767.7867127 | 4.2e-12 rad/s | 1e-8 rad/s | Pass |
| 5 | Small-signal time constant, 1 % step at 80 % duty | 0.0757383443 | 0.0757784664 | 5.3e-4 (relative) | 1e-2 | Pass |
| 6 | Lag tier time constant, step to half speed | 0.0300000000 | 0.0300000003 | 1.1e-8 (relative) | 1e-2 | Pass |
| 7 | ESC latency 2 ms at 1 kHz, in ticks | 2 | 2 | 0 ticks | 0 ticks | Pass |
| 8 | ESC latency 2.5 ms at 1 kHz, in ticks | 3 | 3 | 0 ticks | 0 ticks | Pass |
| 9 | ESC slew 40 /s, ticks from 0 to full duty | 25 | 25 | 0 ticks | 0 ticks | Pass |
| 10 | Coast-down without braking, 0.5 s | 857.9301878 | 857.9301878 | 2.0e-11 rad/s | 1e-8 rad/s | Pass |
| 11 | Active braking at zero duty, 0.1 s | 321.1143374 | 321.1143375 | 4.4e-8 rad/s | 1e-5 rad/s | Pass |
| 12 | Current-limited start at 15 A, 50 ms | 89.14304997 | 89.14304997 | 5.7e-14 rad/s | 1e-11 rad/s | Pass |
| 13 | Power balance at the steady 0.05 N·m point | 57.90990696 | 57.90990696 | 1.2e-16 (relative) | 1e-13 | Pass |

Why each tolerance:

- **1.** The transient left after T = 3 s, ω·e^(−T/τ) = 8.2e-12 with τ = J·R·K_v² = 0.0923 s, plus rounding N·ε·ω = 7.1e-10 over N = 3000 steps: 7.2e-10 rad/s.
- **2.** The transient left after T = 3 s, ω·e^(−T/τ) = 8.0e-12 with τ = J·R·K_v² = 0.0923 s, plus rounding N·ε·ω = 6.9e-10 over N = 3000 steps: 7.0e-10 rad/s.
- **3.** The transient left after T = 3 s, ω·e^(−T/τ) = 7.8e-12 with τ = J·R·K_v² = 0.0923 s, plus rounding N·ε·ω = 6.7e-10 over N = 3000 steps: 6.8e-10 rad/s.
- **4.** The transient left after T = 3 s with the linearized τ = 0.0759 s, Eq. (MOT-6), ω·e^(−T/τ) = 5.1e-15, plus rounding N·ε·ω = 5.1e-10: 5.1e-10 rad/s. The transient decays faster than e^(−T/τ) from below.
- **5.** Relative: the quadratic load's second-order term, k·Δω·τ/J = 8.1e-4 for a step of Δω = 7.03 rad/s, plus linear interpolation of the crossing, h²/(8τ²) = 2.2e-5: 8.4e-4.
- **6.** Relative: linear interpolation of the crossing, h²/(8τ²) = 1.4e-4 with τ = 30 ms. RK4's truncation of e^(−h/τ), (h/τ)⁴/120 = 1.0e-8, is four orders smaller and left out.
- **7–9.** Exact: a latency is rounded to the nearest whole tick, halves up, and the delay line holds that many commands; a slew limit moves the duty cycle by at most slew/rate a tick and lands on the target.
- **10.** With no braking the current stays at zero, so the deceleration is constant and RK4 integrates it exactly: only rounding, N·ε·ω₀ = 1.1e-10 rad/s.
- **11.** RK4's truncation of e^(−h/τ), N·(h/τ)⁵/120 of the 1026 rad/s decaying part: 1.3e-7 rad/s. Rounding, N·ε·ω₀ = 2.2e-11, is four orders smaller and left out.
- **12.** The current stays at its limit, so the acceleration is constant and RK4 integrates it exactly: only rounding, N·ε·ω = 9.9e-13 rad/s.
- **13.** Relative: V·i = τ·ω + i²R + i₀·ω/K_v holds exactly in real numbers; a few roundings, 10ε = 2.2e-15.
