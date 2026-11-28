# Configuration files

Airframes, and later scenarios, are written as TOML documents. This page covers the rules every
document follows, how versions work, and what error messages look like.
[ADR 0006](adr/0006-errors-and-configuration.md) records why it works this way.

## Who reads the file

Engine crates do no I/O. A front end, such as the command-line tool or the browser viewer, reads
the file and passes its text to a loader together with a name for error messages, usually the file
path:

```rust,ignore
let airframe = mjk_sim::config::load_airframe("airframes/example.toml", &text)?;
```

Loading checks, in order:
1. TOML syntax.
2. The header: `schema_version`, then `kind`.
3. The shape of the document: required fields present, no unknown fields, values of the right
   type and length.
4. Physical plausibility.

The first problem found is returned.

## Rules

- **Every document starts with a header:**

  ```toml
  schema_version = 1
  kind = "airframe"
  ```

- **Unknown fields are errors.** A misspelled key such as `mas_kg` is reported, never silently
  ignored.
- **Units are in the field names**, using the suffixes from the
  [conventions](conventions.md#7-units-numbers-and-time): `mass_kg`, `cg_m`, `inertia_kg_m2`.
  Values are SI; degrees and RPM appear only where a field name says so.
- **Vectors and tensors are arrays** in body axes (FRD). Matrices are arrays of rows:
  `inertia_kg_m2[row][col]`.
- **Comments are encouraged**, especially to explain where a number came from. Structured source
  citations for every parameter arrive with the full airframe format
  ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)).
- **Machine output is JSON**, written by front ends: run logs, for example
  ([Command-line tool](cli.md#run-logs)). Configuration types implement serde's `Serialize` for
  that purpose.

## A minimal airframe

The airframe format currently covers identity and mass properties. Rotors, propulsion,
aerodynamics and parameter sources arrive with
[#18](https://github.com/MatthewK84/MJKPhysics/issues/18) and
[#43](https://github.com/MatthewK84/MJKPhysics/issues/43).

```toml
schema_version = 1
kind = "airframe"
name = "Example quadrotor"

[mass]
mass_kg = 1.5
cg_m = [0.0, 0.0, 0.01]           # center of mass from the reference point, body axes
inertia_kg_m2 = [                 # about the center of mass, body axes
  [0.0290, 0.0000, 0.0004],
  [0.0000, 0.0300, 0.0000],
  [0.0004, 0.0000, 0.0550],
]
```

### What is checked

| Field | Rule |
|---|---|
| `name` | Not empty. |
| `mass.mass_kg` | Positive and finite. |
| `mass.cg_m` | Finite. |
| `mass.inertia_kg_m2` | Finite; symmetric to a relative tolerance of $`10^{-9}`$; positive definite; physically realizable. |

"Physically realizable" means some mass distribution actually has this inertia. For a symmetric
tensor that holds exactly when the second-moment matrix $`\tfrac12\mathrm{tr}(\mathbf{J})\,\mathbf{I} - \mathbf{J}`$
is positive semidefinite. This is the matrix form of the triangle inequality on the principal
moments, $`J_1 + J_2 \ge J_3`$ (Wensing, Kim & Slotine, 2018). Both conditions are tested with
principal minors, so no eigen-decomposition is needed. A thin plate, with
$`J_3 = J_1 + J_2`$ exactly, is accepted. A rod with a zero principal moment is rejected, because it
makes the rotational dynamics singular.

## Error messages

Every error reads `document: field: reason` and names the exact field, using the same path
syntax as the file. These are real messages from the test fixtures in
`crates/mjk-sim/tests/fixtures/airframe/`:

```text
example.toml: (document): invalid TOML: invalid basic string, expected `"` (line 3, column 26)
example.toml: schema_version: missing; documents start with `schema_version = 1`
example.toml: schema_version: this file uses schema version 2, but this build reads version 1; update MJKPhysics
example.toml: kind: expected "airframe" for this loader; found "scenario"
example.toml: mass.mas_kg: unknown field `mas_kg`, expected one of `mass_kg`, `cg_m`, `inertia_kg_m2` (line 6, column 1)
example.toml: mass: missing field `mass_kg` (line 5, column 1)
example.toml: mass.mass_kg: invalid type: string "1.5", expected f64 (line 6, column 11)
example.toml: mass.inertia_kg_m2[1][0]: invalid type: string "zero", expected f64 (line 10, column 4)
example.toml: mass.inertia_kg_m2[2]: invalid length 2, expected an array of length 3 (line 11, column 3)
example.toml: mass.mass_kg: must be a positive, finite mass in kilograms; found -1.5
example.toml: mass.inertia_kg_m2[0][1]: not symmetric: [0][1] = 0.001 but [1][0] = 0
example.toml: mass.inertia_kg_m2: principal moments violate the triangle inequality, so no rigid body has this inertia (½·tr(J)·I − J has a principal minor of -5.500e-3)
```

## Versions and migration

`schema_version` is the format version of the document. A file newer than the build is refused
with a request to update MJKPhysics. A file older than the oldest supported version is refused
with a pointer to this section.

Version 1 is the only version so far, and no migrations exist yet. Until v0.1.0
([#85](https://github.com/MatthewK84/MJKPhysics/issues/85)), version 1 may gain required fields as
the formats grow; files that miss them fail with a clear error. From v0.1.0 on, any change that
breaks existing files bumps the version and adds a migration note here.

## References

- P. M. Wensing, S. Kim, J.-J. E. Slotine, "Linear matrix inequalities for physically consistent
  inertial parameter identification: a statistical perspective on the mass distribution,"
  *IEEE Robotics and Automation Letters*, 2018.
