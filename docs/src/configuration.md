# Configuration files

Airframes, and later scenarios, are written as TOML documents. This page covers the rules every
document follows, the airframe format, units and sources, and what error messages look like.
[ADR 0006](adr/0006-errors-and-configuration.md) records why it works this way.

## Who reads the file

Engine crates do no I/O. A front end, such as the command-line tool or the browser viewer, reads
the file and passes its text to a loader together with a name for error messages, usually the file
path:

```rust,ignore
let loaded = mjk_sim::config::load_airframe("airframes/example-quadrotor.toml", &text)?;
```

Loading checks, in order:
1. TOML syntax.
2. The header: `schema_version`, then `kind`.
3. The shape of the document: required fields present, no unknown fields, values of the right
   type and length, and a unit on every number.
4. Each parameter: its unit is known and of the right quantity, its value and uncertainty are
   finite, and its source is in the file's bibliography. Then each value is converted to SI.
5. Physical plausibility of each value: a positive mass, a realizable inertia, a nonzero rotor
   axis, and so on.
6. The whole airframe: it can lift itself, and its rotor disks do not overlap.

The first error found is returned. If none is found, the result holds:
- the airframe, in SI units;
- every parameter as written, for the airframe's generated page;
- the warnings, such as a number without a source.

## Rules

- **Every document starts with a header:**

  ```toml
  schema_version = 1
  kind = "airframe"
  ```

- **Unknown fields are errors.** A misspelled key such as `mas` is reported, never silently
  ignored.
- **Every number is a parameter**, a table `{ value, unit, source, uncertainty }`
  ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)):

  ```toml
  mass = { value = 350, unit = "g", source = "datasheet", uncertainty = 5 }
  ```

  - `unit` is required. A bare number, or a table without `unit`, is an error naming the field.
  - `source` is the key of an entry in the file's `[sources]`. A number without a source loads,
    with a warning.
  - `uncertainty` is optional, in the same unit as the value, and never negative.
- **Vectors and tensors are arrays** in body axes (FRD). Matrices are arrays of rows:
  `inertia.value[row][col]`.
- **Machine output is JSON**, written by front ends: run logs, for example
  ([Command-line tool](cli.md#run-logs)). Configuration types implement serde's `Serialize` for
  that purpose.

Units in Rust identifiers still follow the [conventions](conventions.md#7-units-numbers-and-time):
`mass_kg`, `kv_rad_s_v`. In a file, the unit is written next to the number instead, so a value
can be copied from a datasheet exactly as printed.

## Units

A unit is a product of symbols, each with an optional integer power, divided by others:

- `kg m^2` and `kg·m²` are the same unit;
- `rpm/V`, `N m s/rad` and `1/s` show the division;
- `1` is a pure number.

Symbols are separated by spaces, `·` or `*`. Powers are written `^2`, `^-1`, `²` or `³`. Each
field states the quantity it needs. A unit of another dimension is an error that names the
quantity and suggests units.

Only these symbols are known. There is no general prefix rule, so `min` is always a minute:

| Quantity | Symbols |
|---|---|
| Length | `m`, `cm`, `mm`, `km`, `in`, `ft` |
| Mass | `kg`, `g`, `lb`, `oz` |
| Time | `s`, `ms`, `us` or `µs`, `min`, `h` |
| Current | `A`, `mA` |
| Angle | `rad`, `deg`, `rev` |
| Angular rate, frequency | `rpm`; `Hz` |
| Force | `N`, `gf`, `kgf`, `lbf` |
| Energy, power | `J`; `W` |
| Voltage | `V`, `mV` |
| Resistance | `ohm` or `Ω`, `mohm` |
| Charge | `Ah`, `mAh` |

**Each symbol's factor is its definition.**
- **Exact factors:** the inch (0.0254 m), the pound (0.453 592 37 kg) and standard gravity behind
  the gram-force (9.806 65 m/s²) are exact.
- **Rounded factors:** those involving π round once to double precision.

**Angle is its own dimension**, although SI counts it as dimensionless. That keeps an `rpm` from
being mistaken for a `Hz`, and it means a speed constant must be given per volt as an angular
rate (`rpm/V` or `rad/s/V`).

## Sources

`[sources.<key>]` tables form the file's bibliography, and parameters cite them by key:

```toml
[sources.datasheet]
title = "2212 920 KV motor datasheet"
author = "Example Motors"
year = 2024
url = "https://example.com/2212-datasheet.pdf"
note = "Table 2, at 11.1 V."
```

| Field | Rule |
|---|---|
| `title` | Required, not empty. |
| `author`, `url`, `doi`, `note` | Optional text. |
| `year` | Optional whole number. |

A parameter that cites a key with no entry is an error. An entry that no parameter cites gets a
warning.

## The airframe format

[`airframes/example-quadrotor.toml`](https://github.com/MatthewK84/MJKPhysics/blob/main/airframes/example-quadrotor.toml)
uses every table below, and [its page](airframes/example-quadrotor.md) is what
`mjk airframe` generates from it. Required fields are in bold.

**Top level:**
- **`schema_version`**, **`kind`**: the header.
- **`name`**: not empty.
- **`mass`**: the mass properties.
- **`rotors`**: at least one.
- **The optional tables:** `sources`, `propellers`, `motors`, `escs`, `battery`, `drag`,
  `controller` and `visual`.

### Mass properties

Give either **`mass`**, **`cg`** and **`inertia`** directly, or a list of `[[mass.components]]`
built up by the parallel-axis theorem. Giving both is an error.

| Field | Quantity | Rule |
|---|---|---|
| `mass` | mass | Positive and finite. |
| `cg` | length, vector | Center of mass from the airframe's reference point. Finite. |
| `inertia` | moment of inertia, 3×3 | About the center of mass. Finite; symmetric to a relative tolerance of $`10^{-9}`$; positive definite; physically realizable. |

Each component:

| Field | Quantity | Rule |
|---|---|---|
| **`name`** | text | Not empty. |
| **`mass`** | mass | Positive. |
| **`position`** | length, vector | Its center from the reference point. |
| **`shape`** | `point`, `box` or `cylinder` | A box needs `size`; a cylinder needs `radius` and `length`, along its own z axis. A shape with fields of another shape is an error. |
| `size`, `radius`, `length` | length | Not negative. |
| `orientation` | angle, vector | Roll, pitch and yaw of the part's axes from body axes. |

"Physically realizable" means some mass distribution actually has this inertia. For a symmetric
tensor that holds exactly when the second-moment matrix $`\tfrac12\mathrm{tr}(\mathbf{J})\,\mathbf{I} - \mathbf{J}`$
is positive semidefinite. This is the matrix form of the triangle inequality on the principal
moments, $`J_1 + J_2 \ge J_3`$ (Wensing, Kim & Slotine, 2018). Both conditions are tested with
principal minors, so no eigen-decomposition is needed. A thin plate, with
$`J_3 = J_1 + J_2`$ exactly, is accepted. A rod with a zero principal moment is rejected, because it
makes the rotational dynamics singular.

### Rotors and propellers

Each `[[rotors]]` entry:

| Field | Quantity | Rule |
|---|---|---|
| **`name`** | text | Not empty. |
| **`position`** | length, vector | The hub, from the reference point. |
| **`axis`** | pure number, vector | The direction of thrust: up is `[0, 0, -1]`. Not zero; normalized when read. |
| **`spin`** | `cw` or `ccw` | Seen from the side the thrust points to. A `ccw` rotor's angular velocity points along `axis`. |
| **`propeller`** | key | An entry in `[propellers]`. |
| `motor`, `esc` | key | Entries in `[motors]` and `[escs]`. |

Each `[propellers.<key>]` entry:

| Field | Quantity | Rule |
|---|---|---|
| **`diameter`** | length | Positive. |
| `pitch` | length | Positive. |
| **`blades`** | count | Unit `1`; at least 1. |
| **`max_thrust`** | force | Largest static thrust with its motor. Positive. |
| `mass` | mass | Positive. |
| `inertia` | moment of inertia | About the spin axis. Positive. |
| `data` | text | Where its measured performance is, such as a UIUC propeller database entry. |

Two checks cover the whole airframe:
- **Thrust-to-weight.** The rotors' summed `max_thrust`, divided by the weight at standard
  gravity, must exceed 1.
- **Rotor disks.** Two rotors whose hubs are at least the sum of their radii apart are clear.
  Closer than that:
  - **Parallel rotors** whose disk planes are more than 1 mm apart are stacked, as in a coaxial
    pair, and pass. In the same plane, their disks overlap, which is an error.
  - **Tilted rotors** are an error, because this version cannot verify their clearance.

### Motors, ESCs and the battery

A motor is described by its electrical constants, or only by its time constant for airframes
where only that is published. The models that use these arrive with
[#19](https://github.com/MatthewK84/MJKPhysics/issues/19) and
[#20](https://github.com/MatthewK84/MJKPhysics/issues/20).

| Table | Field | Quantity | Rule |
|---|---|---|---|
| `[motors.<key>]` | `kv` | speed constant, such as `rpm/V` | `kv`, `resistance` and `no_load_current` together, or `time_constant`. All positive. |
| | `resistance` | resistance | |
| | `no_load_current` | current | |
| | `time_constant` | time | |
| | `max_current` | current | Positive. |
| | `inertia` | moment of inertia | Rotor, about the spin axis. Positive. |
| | `mass` | mass | Positive. |
| `[escs.<key>]` | **`braking`** | `active` or `none` | What the motor does when the command drops. |
| | `latency` | time | Positive. |
| | `slew_rate` | rate, `1/s` | Fastest change, in full scale per second. Positive. |
| | `max_current` | current | Positive. |
| `[battery]` | **`cells`** | count | Cells in series. |
| | **`capacity`** | charge, such as `mAh` | Positive. |
| | `internal_resistance` | resistance | Of the whole pack. Positive. |

### Drag, controller and visual hints

These tables are stored and their units checked. Body aerodynamics
([#24](https://github.com/MatthewK84/MJKPhysics/issues/24)), the flight controllers
([#26](https://github.com/MatthewK84/MJKPhysics/issues/26),
[#27](https://github.com/MatthewK84/MJKPhysics/issues/27)) and the renderer (M3) define how they
are used.

| Table | Field | Quantity | Rule |
|---|---|---|---|
| `[drag]` | **`area`** | area, vector | Drag coefficient times reference area along each body axis. Not negative. |
| `[controller]` | **`kind`** | text | Which controller, such as `cascaded-pid`. Not empty. |
| | `gains.<name>` | any | Each gain in its own unit, such as `N m s/rad` or `1/s`. |
| `[visual]` | `body_size` | length, vector | Positive. |
| | `arm_width` | length | Positive. |
| | `color` | text | `#` and six hexadecimal digits. |

## Warnings

A file with warnings still loads. `mjk airframe` prints each warning to standard error, and the
generated page lists them. A warning is given for:
- a number without a source;
- a source that no parameter cites;
- a propeller, motor or ESC that no rotor uses.

## Error messages

Every error reads `document: field: reason` and names the exact field, using the same path
syntax as the file. A key that is not a bare TOML key is quoted, as in
`propellers."10x4.5".diameter`. These are real messages from the test fixtures in
`crates/mjk-sim/tests/fixtures/airframe/`:

```text
example.toml: (document): invalid TOML: invalid basic string, expected `"` (line 4, column 26)
example.toml: schema_version: missing; documents start with `schema_version = 1`
example.toml: schema_version: this file uses schema version 2, but this build reads version 1; update MJKPhysics
example.toml: kind: expected "airframe" for this loader; found "scenario"
example.toml: mass.mas: unknown field `mas`, expected one of `mass`, `cg`, `inertia`, `components` (line 10, column 1)
example.toml: mass: missing `inertia`: give mass, cg and inertia, or [[mass.components]]
example.toml: mass.mass: missing `unit`: every number in an airframe file carries one, as in { value = 1.2, unit = "kg", source = "maker" } (line 10, column 8)
example.toml: mass.mass.unit: `m` is not a mass; use a unit such as kg or g
example.toml: mass.mass.unit: unknown unit `stone`
example.toml: mass.mass.source: no entry `manual` in [sources]
example.toml: mass.mass.value: invalid type: string "heavy", expected f64 (line 10, column 18)
example.toml: mass.inertia.value[2]: invalid length 2, expected an array of length 3 (line 17, column 3)
example.toml: mass.mass.value: must be a positive, finite mass in kilograms; found -1.5
example.toml: mass.cg.value[1]: must be finite; found NaN
example.toml: mass.inertia.value[0][1]: not symmetric: [0][1] = 0.001 but [1][0] = 0
example.toml: mass.inertia.value: principal moments violate the triangle inequality, so no rigid body has this inertia (½·tr(J)·I − J has a principal minor of -5.500e-3)
example.toml: rotors: thrust-to-weight is 0.666667: the rotors' 9.80665 N of maximum thrust cannot lift 14.71 N
example.toml: rotors[1]: its disk overlaps rotors[0]'s: their hubs are 0.21 m apart, less than the 0.254 m their radii add up to
```

And the warning for a number without a source:

```text
warning: example.toml: mass.mass: no source
```

## Versions and migration

`schema_version` is the format version of the document. A file newer than the build is refused
with a request to update MJKPhysics. A file older than the oldest supported version is refused
with a pointer to this section.

Version 1 is the only version so far, and no migrations exist yet. Until v0.1.0
([#85](https://github.com/MatthewK84/MJKPhysics/issues/85)), version 1 may change as the formats
grow, and files that no longer fit fail with a clear error.
- **#18 used that latitude:** it replaced the first slice's bare SI fields, such as `mass_kg`,
  with parameter tables carrying units and sources.
- **From v0.1.0 on,** any change that breaks existing files bumps the version and adds a migration
  note here.

## References

- P. M. Wensing, S. Kim, J.-J. E. Slotine, "Linear matrix inequalities for physically consistent
  inertial parameter identification: a statistical perspective on the mass distribution,"
  *IEEE Robotics and Automation Letters*, 2018.
