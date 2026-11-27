# Atmosphere and gravity

- **Issue:** [#15](https://github.com/MatthewK84/MJKPhysics/issues/15)
- **Code:** `mjk_env::{Atmosphere, Air, Gravity, NormalGravity, Environment}`,
  `mjk_env::atmosphere`, `mjk_env::gravity::wgs84`, `mjk_env::reference`,
  `mjk_sim::validation::atmosphere`
- **Equation prefix:** `ATM`

## Summary

The air and gravity around the vehicle:

- **Air:** temperature, pressure, density, speed of sound and dynamic viscosity from the
  *U.S. Standard Atmosphere, 1976*, from −5 km to 20 km. Viscosity gives the Reynolds numbers
  the rotor models need.
- **Temperature offset:** shifts the whole profile for a hot or cold day ("ISA + 15") while
  keeping the air in hydrostatic balance.
- **Gravity:** constant standard gravity, or WGS-84 normal gravity, which varies with latitude
  and altitude.
- **Field elevation:** places the world origin above mean sea level, so a position's Down
  coordinate gives its altitude.

Every quantity matches the standard's printed tables to four significant figures, and the
gravity model matches the exact field of the WGS-84 ellipsoid to $`1.5\times10^{-6}`$ m/s².

## Assumptions

1. **One altitude for both models.** Altitude is geometric height above mean sea level. Gravity
   uses it as height above the WGS-84 ellipsoid: the two differ by the geoid height, at most
   about 100 m, which changes gravity by about $`3\times10^{-5}`$ of its value.
2. **Air is a still, dry ideal gas** of constant composition below 20 km, as the standard
   defines it. Humidity and weather are not modeled; wind arrives with
   [#53](https://github.com/MatthewK84/MJKPhysics/issues/53).
3. **A temperature offset shifts every layer equally.** Sea-level pressure stays 101 325 Pa,
   and pressure follows from hydrostatic balance with the warmer or colder air.
4. **Gravity points along** $`+\mathbf{z}_W`$, on the flat earth of the
   [conventions](../conventions.md#21-world-frame-and-the-flat-earth-assumption): its direction
   changes by only 0.09° over 10 km.

## Equations

Altitude comes from the field elevation $`h_0`$ and the Down coordinate $`p_D`$:

```math
h = h_0 - p_D
\qquad\text{(ATM-1)}
```

The standard works in geopotential altitude $`H`$: the height in a uniform field of
$`g_0 = 9.80665`$ m/s² with the same potential energy. With its earth radius
$`r_0 = 6\,356\,766`$ m,

```math
H = \frac{r_0\,h}{r_0 + h}
\qquad\text{(ATM-2)}
```

Below 20 km there are two layers: the troposphere, cooling at $`L = -6.5`$ K/km to the
tropopause at $`H_{11} = 11`$ km, and an isothermal layer above it. With a temperature offset
$`\Delta T`$ and sea-level temperature $`T_0 = 288.15`$ K,

```math
T(H) = \begin{cases} T_0 + \Delta T + L\,H & H \le H_{11} \\ T_0 + \Delta T + L\,H_{11} & H > H_{11} \end{cases}
\qquad\text{(ATM-3)}
```

Hydrostatic balance, $`dP = -\rho\,g_0\,dH`$, with the gas law gives pressure in closed form,
starting from $`P_0 = 101\,325`$ Pa:

```math
P = P_0 \left(\frac{T}{T_0 + \Delta T}\right)^{-g_0 M_0 / (R^* L)}, \quad H \le H_{11}
\qquad\text{(ATM-4)}
```

```math
P = P_{11} \exp\!\left(-\frac{g_0 M_0 (H - H_{11})}{R^*\,T_{11}}\right), \quad H > H_{11}
\qquad\text{(ATM-5)}
```

where $`T_{11}`$ and $`P_{11}`$ are the values at the tropopause, $`M_0 = 0.0289644`$ kg/mol is
the mean molar mass of air, and $`R^* = 8.31432`$ J/(mol·K) is the gas constant as the 1976
standard defines it. The other quantities follow from $`T`$ and $`P`$:

```math
\rho = \frac{P\,M_0}{R^*\,T}
\qquad\text{(ATM-6)}
```

```math
a = \sqrt{\frac{\gamma\,R^*\,T}{M_0}}, \quad \gamma = 1.4
\qquad\text{(ATM-7)}
```

```math
\mu = \frac{\beta\,T^{3/2}}{T + S}, \quad \beta = 1.458\times10^{-6}\ \text{kg/(m·s·K}^{1/2}),\ S = 110.4\ \text{K}
\qquad\text{(ATM-8)}
```

WGS-84 normal gravity at geodetic latitude $`\varphi`$ follows Somigliana's closed form on the
ellipsoid,

```math
\gamma(\varphi) = \gamma_e\,\frac{1 + k\sin^2\varphi}{\sqrt{1 - e^2\sin^2\varphi}}
\qquad\text{(ATM-9)}
```

and its second-order expansion in height above it, as NGA.STND.0036 gives them (eqs. 4-1 and 4-3),

```math
\gamma_h = \gamma(\varphi)\left[1 - \frac{2}{a}\left(1 + f + m - 2f\sin^2\varphi\right)h + \frac{3h^2}{a^2}\right]
\qquad\text{(ATM-10)}
```

with semi-major axis $`a`$, flattening $`f`$, $`m = \omega^2 a^2 b / GM`$, equatorial gravity
$`\gamma_e`$, eccentricity $`e`$ and $`k = b\gamma_p / (a\gamma_e) - 1`$.

## Validity range

| | Range | Outside it |
|---|---|---|
| Altitude | −5 km to 20 km geometric. 20 km lies inside the isothermal layer, which ends at 20 km geopotential (20 063 m geometric) | `AltitudeOutOfRange`: the model never extrapolates or clamps |
| Temperature offset | Any finite value that keeps the coldest layer above 0 K | `InvalidTemperatureOffset` |
| Latitude | ±90° | `LatitudeOutOfRange` |
| Normal gravity | −5 km to 20 km: the expansion's error grows with height, to $`1.5\times10^{-6}`$ m/s² at 20 km | Usable higher, with a larger truncation error |

A non-finite input is an error, never a NaN in the results. A run that leaves the range finds
out when a model asks for the air there; the world step has no error path yet, so the models that
use the atmosphere (M2) will decide how such a run fails.

## Parameters

| Symbol | Meaning | Unit | Value | Source |
|---|---|---|---|---|
| $`g_0`$ | Standard gravity | m/s² | 9.80665 | USSA-1976 |
| $`R^*`$ | Gas constant (1976 value, not CODATA's 8.314 462 618) | J/(mol·K) | 8.31432 | USSA-1976 |
| $`M_0`$ | Mean molar mass of air | kg/mol | 0.0289644 | USSA-1976 |
| $`r_0`$ | Earth radius for geopotential altitude | m | 6 356 766 | USSA-1976 |
| $`a`$, $`1/f`$ | WGS-84 semi-major axis, inverse flattening | m, — | 6 378 137, 298.257223563 | NGA.STND.0036 Table 3.1 |
| $`GM`$, $`\omega`$ | Earth's gravitational constant, angular velocity | m³/s², rad/s | $`3.986004418\times10^{14}`$, $`7.292115\times10^{-5}`$ | NGA.STND.0036 Table 3.1 |
| $`\gamma_e`$, $`\gamma_p`$ | Normal gravity at the equator and the poles | m/s² | 9.780325335904, 9.832184937863 | Derived, below |
| $`k`$, $`e^2`$, $`m`$ | Somigliana's constant, eccentricity squared, gravity ratio | — | 0.001931852652458, 0.006694379990141, 0.003449786506841 | Derived, below |

The four WGS-84 defining parameters are those of NGA.STND.0036, Table 3.1. The derived constants
are computed from them with the closed formulas of the level ellipsoid (Heiskanen and Moritz,
1967) at 40 significant digits, and agree with the standard's printed values to the last digit:

| Constant | NGA.STND.0036 | Derived here |
|---|---|---|
| $`f`$ (Table 3.5) | $`3.3528106647475\times10^{-3}`$ | $`3.3528106647474805\times10^{-3}`$ |
| $`b`$ (Table 3.5) | 6 356 752.3142 m | 6 356 752.314245 m |
| $`e^2`$ (Table 3.5) | $`6.694379990141\times10^{-3}`$ | $`6.694379990141317\times10^{-3}`$ |
| $`\gamma_e`$ (Table 3.6) | 9.7803253359 m/s² | 9.78032533590389 m/s² |
| $`\gamma_p`$ (Table 3.6) | 9.8321849379 m/s² | 9.83218493786340 m/s² |
| $`k`$ (Table 3.6) | $`1.931852652458\times10^{-3}`$ | $`1.931852652458274\times10^{-3}`$ |
| $`m`$ (Table 3.6) | $`3.449786506841\times10^{-3}`$ | $`3.449786506840845\times10^{-3}`$ |

**Precision matters in the last digits.** Computing $`q_0`$ directly in double precision loses about
$`3\times10^{-13}`$ of its value to cancellation, which gives $`k = 1.93185265241\times10^{-3}`$:
44 units off in the standard's last printed digit. That value is quoted in some secondary
sources. The code stores the 40-digit constants, which match the standard, and a test fails if a
double-precision recomputation replaces them. The difference is about $`10^{-10}`$ m/s² in
gravity, so nothing physical depends on it; what depends on it is whether the constants can be
traced to the standard.

## Implementation notes

- **Closed forms, no tables.** Each quantity is a handful of operations, with `libm` for the
  power, exponential and sine, so results are identical on every platform. Nothing allocates.
- **The tropopause pressure** is computed once, when the atmosphere is built.
- **Reference data** from the standard's tables lives in `mjk_env::reference`, with the
  document's page numbers, so the tests and the study compare against the same printed values.

## Validation

How well the model agrees with the published tables, generated by `mjk validate atmosphere`:

{{#include ../validation/atmosphere.md}}

*Each altitude has two rows: the model, then the 1976 table as printed. Gap is the largest
difference at that altitude, in units of the table value's fourth significant figure: four-figure
agreement means at most 0.5. A test fails if this table no longer matches the code, and CI requires the WebAssembly
build to print it byte for byte.*

| Check | Criterion | Test |
|---|---|---|
| The 1976 tables at 0, 1, 2, 5, 11 and 20 km: T, P, ρ, a, μ | within half a unit of the fourth significant figure; in fact within $`5\times10^{-5}`$ relative | `matches_the_1976_tables_to_four_significant_figures` |
| Geopotential altitude against the tables' H column | within 0.5 m | `geopotential_altitude_matches_the_tables` |
| Continuity at the tropopause | T and P continuous to $`10^{-9}`$ | `temperature_and_pressure_are_continuous_at_the_tropopause` |
| Hydrostatic balance, $`dP/dh = -\rho\,g_0\,(r_0/(r_0+h))^2`$, with ΔT = 0, +15 and −20 K | within $`10^{-6}`$ relative | `the_air_is_in_hydrostatic_balance_with_and_without_an_offset` |
| A +15 K offset | T up 15 K everywhere; sea-level pressure unchanged; pressure higher at every altitude above sea level | `an_offset_shifts_temperature_and_keeps_sea_level_pressure` |
| The model's range and invalid inputs | errors at −5 km − 1 mm and 20 km + 1 mm, and for non-finite values | `the_model_covers_exactly_its_range`, `an_offset_that_is_not_physical_is_rejected` |
| WGS-84 derived constants | recomputed from a, f, GM, ω | `the_derived_constants_follow_from_the_defining_parameters` |
| WGS-84 derived constants against NGA.STND.0036, Tables 3.5 and 3.6 | f, b, e², γe, γp, k and m within half a unit of the last printed digit; a double-precision derivation of k would miss by 44 units | `the_derived_constants_agree_with_the_standard_to_the_last_printed_digit`, `a_double_precision_derivation_would_miss_the_published_k` |
| Normal gravity against the exact field of the level ellipsoid, at 0°, 45° and 90° from −5 to 20 km | within $`1.5\times10^{-6}`$ m/s²; on the ellipsoid within $`10^{-9}`$ | `normal_gravity_matches_the_exact_field_from_minus_5_to_20_km` |
| Altitude from the field elevation, and air and gravity there | equal to evaluating the models directly | `the_environment_measures_altitude_from_the_field` |

**A warm day does not simply thin the air.** With ΔT = +15 K the air near the ground is thinner:
4 % less dense than standard at 2 km. But a warm column holds more air aloft, so pressure is
higher at every altitude, and above about 9 km the air is *denser* than standard: 12 % at
20 km. A model that kept the standard pressure at each altitude and only raised the
temperature would get the sign of that effect wrong up high.

## References

- NOAA, NASA, USAF, *U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562, 1976. NASA scan:
  NTRS 19770009539. Table I, pp. 53–61; Table III, pp. 101–109.
- NGA, *Department of Defense World Geodetic System 1984: Its Definition and Relationships with
  Local Geodetic Systems*, NGA.STND.0036_1.0.0_WGS84, version 1.0.0, 2014-07-08: Table 3.1
  (p. 3-4), Table 3.5 (p. 3-9), Table 3.6 (p. 3-10), eqs. (4-1) and (4-3) (pp. 4-1, 4-3).
- NIMA, *Department of Defense World Geodetic System 1984: Its Definition and Relationships with
  Local Geodetic Systems*, TR8350.2, 3rd ed., Amendment 1, 2000: the standard's predecessor.
- W. A. Heiskanen, H. Moritz, *Physical Geodesy*, W. H. Freeman, 1967: the normal gravity field
  of the level ellipsoid (ch. 2).
- Department of Defense, *Performance Specification: World Magnetic Model (WMM)*,
  MIL-PRF-89500B, 2019, Appendix A, eq. (8): the same $`a`$ and $`1/f`$.
