# ADR 0006: One engine error type, and configuration parsed from TOML text

- **Status:** Accepted
- **Date:** 2026-11-26
- **Issues:** [#7](https://github.com/MatthewK84/MJKPhysics/issues/7)
- **Amends:** [ADR 0001](0001-hand-written-math.md) (external-dependency allowlist)

## Context

Errors arise in every layer of the engine:
- configuration problems when airframes and scenarios are loaded;
- non-finite values in the dynamics;
- iterative solvers that fail to converge, such as rotor inflow and trim.

Callers, including the command-line tool, the browser bindings and the Python bindings, need one
type to handle. They also need messages that point to the exact cause: for a configuration file,
the exact field.

The architecture already says engine crates do no I/O ([Architecture](../architecture.md)), and
[#7](https://github.com/MatthewK84/MJKPhysics/issues/7) asks for configuration in TOML with
versioned headers, rejection of unknown fields, field-path messages and plausibility checks.

## Decision

- **One error type, `EngineError`, in a new bottom-layer crate `mjk-core`.** Every engine crate may
  depend on `mjk-core`, and it depends on nothing. The enum is `#[non_exhaustive]` and has three
  variants:
  - `Config { document, path, reason }`;
  - `Numerical { sim_time_s, quantity, value }`;
  - `Convergence { solver, iterations, residual }`.

  The numerical and convergence variants use `&'static str` names, so creating them on the
  simulation path doesn't allocate.
- **No I/O variant.** Engine loaders take the document's text and a name for messages. Front ends
  read files and wrap their own I/O errors alongside `EngineError`.
- **TOML for human-written configuration**, parsed with serde. Every document starts with
  `schema_version` and `kind`, which are checked before the rest of the document, so version
  problems are reported first. Configuration structs use `deny_unknown_fields`.
- **Exact field paths.** Paths such as `mass.inertia_kg_m2[1][0]` come from
  `serde_path_to_error`, and syntax errors report line and column. Plausibility checks build paths
  with `FieldPath`.
- **Physical checks live with the physics.** For example, the inertia consistency check is
  `mjk_dynamics::mass::check_inertia`. The configuration layer maps its result to a field path.
- **Engine dependency allowlist:** `libm`, `serde`, `serde_path_to_error` and `toml`.
  `serde_path_to_error` is added. serde's derive becomes the one procedural macro allowed in
  engine crates.

## Consequences

- Front ends handle one engine error type, and its messages read `document: field: reason`.
- Adding a variant is not a breaking change for callers, because the enum is non-exhaustive.
- Fixtures for every class of malformed file, with exact expected paths, are the acceptance tests
  for the format (`crates/mjk-sim/tests/airframe_config.rs`).
- Loading parses the text twice, once for the header and once for the typed document. Documents
  are small and loaded once, so this costs nothing that matters.

## Alternatives considered

- **Per-crate error enums.** More precise types, but callers would juggle many types, and the
  information they need (where and why) is the same everywhere.
- **`thiserror` or `anyhow`.** `thiserror` adds another procedural macro for little benefit over a
  hand-written `Display`. `anyhow` erases the type, which the bindings need to inspect.
- **Putting `EngineError` in `mjk-math`.** It avoids a crate, but mixes unrelated concerns into the
  math layer.
- **YAML, JSON or RON for configuration.** YAML's implicit typing and significant indentation
  cause silent errors. JSON has no comments, and comments are where parameter provenance gets
  explained. RON is less familiar. TOML is explicit, supports comments, and is already familiar
  from Cargo.
- **Recovering paths from TOML error spans by walking the parsed table.** Possible without a
  dependency, but it means more code tied to TOML's spanned-value API than a small, widely used
  crate written for exactly this job.
- **Hand-written `Deserialize` implementations instead of serde's derive.** Far more code for
  each format change, with more room for error.
