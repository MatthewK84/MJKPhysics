# Contributing to MJKPhysics

Thank you for helping. This project's value rests on being checkable, so the rules below are
enforced by tools wherever a tool can enforce them, and by review where none can.

## Workflow

1. Pick an issue. Each has a scope, acceptance criteria, references and the issues it depends on.
   The project principles and the shared definition of done are in
   [#1](https://github.com/MatthewK84/MJKPhysics/issues/1).
2. Branch from `main`, one issue per branch.
3. Open a pull request that says `Closes #N`, ticks the issue's acceptance criteria and explains
   how each was verified.
4. CI must be green before review. Squash or rebase onto `main`; keep history linear.

Commit subjects are imperative and under about 72 characters ("Add RK4 integrator", not "Added").

## Running the checks locally

Rust (toolchain pinned in `rust-toolchain.toml`):

```bash
rustup toolchain install
```

```bash
cargo fmt --all --check && cargo clippy --workspace --all-targets --locked -- -D warnings
```

```bash
cargo test --workspace --locked && cargo xtask layering
```

Viewer (Node 22 or newer):

```bash
cd viewer && npm ci && npm run check
```

## Continuous integration

`.github/workflows/ci.yml` runs on every pull request and every push to `main`. Each check is its
own job, so each can be required before merging:

| Status check | What it runs |
|---|---|
| `Rust format` | `cargo fmt --all --check` |
| `Clippy` | `cargo clippy --workspace --all-targets --locked -- -D warnings` |
| `Tests (Linux)` | `cargo test --workspace --locked` |
| `Crate layering` | `cargo xtask layering` |
| `API docs` | `cargo doc --workspace --no-deps --locked` with `RUSTDOCFLAGS=-D warnings` |
| `WebAssembly build` | release build of `mjk-wasm` for `wasm32-unknown-unknown`; raw and gzip sizes go to the job summary |
| `Viewer` | `npm ci`, then typecheck, lint and format check in `viewer/` |

All Rust jobs build with `RUSTFLAGS=-D warnings` and `--locked`, and use the toolchain pinned in
`rust-toolchain.toml`. Third-party actions are pinned to full commit SHAs, and checkouts don't
keep credentials.

`.github/workflows/cross-platform.yml` runs the test suite on macOS and Windows every Monday and
on demand. Scheduled workflows only run from the repository's default branch.

**Branch protection.** To make CI block merges, a repository admin enables a rule for `main`
(Settings → Rules → Rulesets, or Settings → Branches): require a pull request, require status
checks to pass, and select the seven checks in the table above.

## Rust coding standard

The workspace lint table in `Cargo.toml` applies to every crate; `clippy.toml` relaxes the
panic-related lints inside tests only, where a panic is simply a failing test.

| # | Rule | How it is enforced |
|---|---|---|
| 1 | **Simple control flow.** Early returns, `let … else`, iterator adaptors; avoid deep nesting. | clippy `pedantic`; review |
| 2 | **No global mutable state.** Module-level items are `const` or immutable `static`; state is passed explicitly. | `unsafe_code = "forbid"` rules out `static mut`; review |
| 3 | **No panicking shortcuts or unchecked features.** No `unsafe`, `unwrap`, `expect`, `panic!`, `todo!`, `unimplemented!`, `unreachable!`, slice indexing that can panic, silent `as` casts, or exact float comparison. | `forbid`/`deny` lints: `unsafe_code`, `unwrap_used`, `expect_used`, `panic`, `todo`, `unimplemented`, `unreachable`, `indexing_slicing`, `as_conversions`, `float_cmp` |
| 4 | **Tight scoping.** Declare at first use, immutable by default, smallest visibility that works. | compiler; review |
| 5 | **Static analysis.** Formatting, lints and the layering check run on every PR. | `rustfmt`, `clippy -D warnings`, `cargo xtask layering`, CI |
| 6 | **No recursion.** Use loops and explicit fixed-capacity stacks (for example in BVH traversal). | review: clippy has no lint for this |
| 7 | **Small, focused functions.** At most 60 lines; prefer pure functions. | clippy `too_many_lines` (threshold in `clippy.toml`) |
| 8 | **Declare all data.** Public items are documented; physical quantities carry units in their names (`mass_kg`, `omega_rad_s`). | `missing_docs`; review |
| 9 | **One error model.** Fallible functions return `Result` with the crate's error type; numerical failures report sim time and quantity. Errors are never swallowed. | lints above; review |
| 10 | **Macros stay simple.** `macro_rules!` only for trivial repetition; no procedural-macro dependencies in engine crates. | engine external-dependency allowlist in `xtask/src/layering.rs` |

Engine-specific rules:

- **No heap allocation in `World::step`.** Allocate at construction; verified by an
  allocation-counting test once the world loop exists
  ([#13](https://github.com/MatthewK84/MJKPhysics/issues/13)).
- **Transcendental functions go through `libm`**, so native and WebAssembly builds produce
  identical bits ([#14](https://github.com/MatthewK84/MJKPhysics/issues/14)).
- **Conventions are fixed**: NED world frame, FRD body frame, Hamilton scalar-first quaternions,
  SI units ([#5](https://github.com/MatthewK84/MJKPhysics/issues/5)).

## Crate layering and dependencies

Crates form layers (`mjk-math` → … → `mjk-sim` → front ends). `cargo xtask layering` rejects any
dependency outside a crate's allowed set, and engine crates may use only `libm`, `serde` and `toml`
as external dependencies outside of tests. Adding a crate or an engine dependency means updating
the rule table in `xtask/src/layering.rs` and recording the decision in an architecture decision
record ([#6](https://github.com/MatthewK84/MJKPhysics/issues/6)).

## TypeScript coding standard (viewer)

`viewer/tsconfig.json` enables `strict` plus `noUncheckedIndexedAccess`,
`exactOptionalPropertyTypes` and related flags. `viewer/eslint.config.js` builds on
`typescript-eslint`'s strict type-checked rules.

| # | Rule | How it is enforced |
|---|---|---|
| 1 | Early returns and shallow control flow | `max-depth` 3, `complexity` 10 |
| 2 | No global state; don't mutate inputs | `no-restricted-syntax` (assigning to `window`/`globalThis`), `no-param-reassign` |
| 3 | No legacy or unsafe features: `eval`, `document.write`, `any`, non-null `!` | `no-eval`, `no-new-func`, `no-restricted-properties`, `no-explicit-any`, `no-non-null-assertion` |
| 4 | Block scoping: `const` by default, never `var` | `no-var`, `prefer-const` |
| 5 | Static analysis | ESLint, Prettier, `tsc --noEmit` in CI |
| 6 | Avoid deep recursion | review |
| 7 | Small, pure functions | `max-lines-per-function` 60 |
| 8 | Explicit data shapes and signatures | `explicit-function-return-type`, `explicit-module-boundary-types`; `interface`/`type` for shapes |
| 9 | Consistent async error handling; no floating promises | `no-floating-promises`, `no-misused-promises` |
| 10 | Never modify built-in prototypes | `no-extend-native` |

## Physical models and documentation

Each physical model gets a documentation page with numbered equations, assumptions, validity range,
parameters and references, and code comments cite those equation numbers. Every model needs at least
one test against an analytic result or published data. Parameters in airframe files carry a unit, a
source and, where known, an uncertainty.

## License of contributions

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed under the MIT and
Apache-2.0 licenses, without any additional terms or conditions.
