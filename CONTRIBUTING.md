# Contributing to MJKPhysics

Thank you for helping. This project's value rests on being checkable, so the rules below are
enforced by tools wherever a tool can enforce them, and by review where none can.

## Workflow

1. Pick an issue. Each has a scope, acceptance criteria, references and the issues it depends on.
   The project principles and the shared definition of done are in
   [#1](https://github.com/MatthewK84/MJKPhysics/issues/1).
2. Branch from `main`, one issue per branch. Nothing is pushed to `main` directly: it is
   protected (below).
3. Open a pull request that says `Closes #N`, ticks the issue's acceptance criteria and explains
   how each was verified.
4. Turn on auto-merge. The pull request is rebased onto `main` once all fourteen required checks
   pass, and GitHub then deletes its branch. History stays linear.

Commit subjects are imperative and under about 72 characters ("Add RK4 integrator", not "Added").

Write American English everywhere: code, comments, docs, commit messages and issues. CI runs
`typos` with the US locale, which rejects British spellings, including inside identifiers. Text
between `typos: off` and `typos: on` markers is not checked; use them only for quoted material.
`typos` cannot reject a British spelling that is also an American word: *disc* passes (a compact
disc), so reviewers check that the rotor *disk* and disk loading are spelled with a *k*.

## Running the checks locally

Rust (toolchain pinned in `rust-toolchain.toml`):

```bash
rustup toolchain install
```

```bash
cargo fmt --all --check && cargo clippy --workspace --all-targets --locked -- -D warnings
```

```bash
cargo test --workspace --locked && cargo xtask layering && cargo xtask equations && cargo xtask standards
```

Supply chain, coverage and mutation testing need three cargo tools
(`cargo install --locked cargo-deny cargo-llvm-cov cargo-mutants`) and `rustup component add llvm-tools`:

```bash
cargo deny --locked check
```

```bash
cargo llvm-cov --workspace --exclude xtask --exclude mjk-cli --exclude mjk-wasm --exclude mjk-alloc-counter --fail-under-lines 90
```

```bash
git diff main > /tmp/change.diff && cargo mutants --workspace --in-diff /tmp/change.diff
```

Viewer, README hero and book scripts (Node 22 or newer):

```bash
cd viewer && npm ci && npm run check
```

## Continuous integration

`.github/workflows/ci.yml` and `.github/workflows/docs.yml` run on every pull request and every
push to `main`; `ci.yml` also runs daily, so a newly published advisory against a locked crate
fails within a day. Each check is its own job, so each can be required before merging:

| Status check | What it runs |
|---|---|
| `Rust format` | `cargo fmt --all --check` |
| `Clippy` | `cargo clippy --workspace --all-targets --locked -- -D warnings` |
| `Tests (Linux)` | `cargo test --workspace --locked` |
| `Crate layering` | `cargo xtask layering` |
| `Power of Ten` | `cargo xtask standards`: recursion, loop bounds, macros, conditional compilation, pointers, assertion density; the allocator crate's lint table; the branch protection ruleset against the workflows |
| `Supply chain` | `cargo deny --locked check` against `deny.toml` |
| `Spelling` | `typos` with the US locale (`typos.toml`): typos and British spellings, in prose and identifiers |
| `Coverage` | engine line coverage of at least 90 %, reported in the job summary |
| `Mutation testing (changed lines)` | `cargo mutants --in-diff` on the lines the change touches; any surviving mutant fails |
| `API docs` | `cargo doc --workspace --no-deps --locked` with `RUSTDOCFLAGS=-D warnings` |
| `WebAssembly build` | release build of `mjk-wasm` for `wasm32-unknown-unknown`; raw and gzip sizes go to the job summary |
| `Native and WebAssembly agree` | builds `mjk` natively and for `wasm32-wasip1`, runs both (the WASI build under Node, `viewer/scripts/run-wasi.mjs`), and requires each study's output to match the other and the book's table byte for byte |
| `Viewer` | `npm audit`; typecheck, lint and format check of `viewer/`, the README hero and the book scripts; `npm run lint:ui` |
| `Docs site` | `cargo xtask equations`, type-check of `docs/theme/math.js`, `mdbook build docs` (workflow `docs.yml`) |

All Rust jobs build with `RUSTFLAGS=-D warnings` (except mutation testing, where a mutant that only
adds a warning must still be tested) and `--locked`, and use the toolchain pinned in
`rust-toolchain.toml`. Third-party actions are pinned to full commit SHAs, and checkouts don't
keep credentials.

On `main`, `docs.yml` also publishes the book to GitHub Pages; that requires Settings → Pages →
Source: *GitHub Actions*, and until then the publish job skips with a warning.

Scheduled workflows, which only run from the default branch:

- `cross-platform.yml` runs the test suite on macOS and Windows every Monday.
- `deep-analysis.yml` mutates the whole engine every Sunday, in four shards, and runs PractRand
  on eight interleaved random streams of one run to 32 GB. PractRand is built from source in the
  job, from an archive pinned by its SHA-256.
- Dependabot (`.github/dependabot.yml`) opens grouped updates for actions, cargo and npm weekly.

**Branch protection.** `main` is protected by the ruleset in
[`.github/rulesets/main.json`](.github/rulesets/main.json), versioned here so it can be reviewed:

- **No direct pushes:** every change arrives through a pull request, merged by rebase. No
  approvals are required, because GitHub does not let authors approve their own pull requests.
- **Checks first:** the fourteen checks in the table above must pass on a branch that is up to
  date with `main`.
- **History is fixed:** `main` cannot be force-pushed or deleted, and history stays linear.
- **No bypass:** the rule applies to admins too. In an emergency an admin switches the ruleset off
  in Settings, which the audit log records.

`cargo xtask standards` fails if the ruleset and the workflows disagree: a required check that no
pull-request job reports, which would make every pull request wait forever, or a pull-request job
the ruleset leaves out. It also fails if the file gains a bypass actor or stops being enforced.
Jobs with their own `if:`, such as the Pages publish, and scheduled workflows are not required.

GitHub does not apply the file by itself; a repository admin imports it (Settings → Rules →
Rulesets → New ruleset → Import a ruleset) and, under Settings → General, enables *Allow
auto-merge* and *Automatically delete head branches*. A change to the file is applied the same
way.

## Engineering standards

Every rule below is enforced by a tool where a tool can check it; the rest are review items, and
the table says why. The book's [Engineering standards](https://matthewk84.github.io/MJKPhysics/engineering-standards.html)
page explains the reasoning.

### Rust: the Power of Ten

The workspace lint table in `Cargo.toml` applies to every crate; `clippy.toml` sets thresholds and
relaxes the panic-related lints inside tests only, where a panic is simply a failing test. Rule
numbers are the Power of Ten's, as `cargo xtask standards` reports them.

| # | Rule | How it is enforced |
|---|---|---|
| 1 | **Simple control flow; no recursion.** Early returns, `let … else`, iterator adaptors. | clippy `excessive_nesting` (threshold 4: an `impl` block counts as a level, so methods get two levels and free functions three) and `pedantic`. Recursion: `cargo xtask standards` builds every workspace target's LLVM IR and reports any call-graph cycle through our code, direct or mutual, including calls resolved through generics, traits and closures. |
| 2 | **Every loop has a fixed upper bound.** | `for` loops run over finite ranges and collections. Every `loop` and `while` in the workspace carries a `// bound:` comment naming its bound (`cargo xtask standards`); clippy `infinite_loop` is denied. |
| 3 | **No heap allocation after initialization.** | `crates/mjk-dynamics/tests/allocations.rs`: a counting global allocator proves `World::step` and `World::advance` allocate nothing over 10⁵ steps. New models and subsystems join that test as they land. |
| 4 | **Short functions.** At most 50 lines. | clippy `too_many_lines` (threshold in `clippy.toml`) |
| 5 | **Assertions state preconditions.** Engine functions that take physical quantities check them with `debug_assert!`. | Review: a per-function quota produces assertions that check nothing. `cargo xtask standards` reports density per crate so a drop is visible. |
| 6 | **Smallest possible scope.** Declare at first use, immutable by default, smallest visibility. | compiler; clippy `shadow_unrelated` |
| 7 | **Check every return value; validate parameters.** No `unwrap`, `expect`, `panic!`, `todo!`, `unimplemented!`, `unreachable!`, panicking indexing, silent `as` casts or exact float comparison. | `unused_must_use` under `-D warnings`; clippy `must_use_candidate`, `let_underscore_must_use`, `let_underscore_untyped`, `unwrap_used`, `expect_used`, `panic`, `todo`, `unimplemented`, `unreachable`, `indexing_slicing`, `as_conversions`, `float_cmp` |
| 8 | **Limited preprocessor.** No `macro_rules!` and no conditional compilation other than `#[cfg(test)]` outside tests. The only procedural macro in engine crates is serde's derive, for configuration types (ADR 0006). | `cargo xtask standards`; the engine dependency allowlist in `xtask/src/layering.rs` |
| 9 | **Restricted pointers.** No `unsafe`, no `dyn` trait objects and no function pointer types outside tests; use generics. | `unsafe_code = "forbid"` in every crate but one (below); `cargo xtask standards` |
| 10 | **All warnings on, static analysis daily.** | `-D warnings`, clippy `pedantic`, `rustfmt`, `cargo-deny`, coverage of at least 90 %, mutation testing on every change and weekly on everything; CI daily |

Project rules on top of the ten:

- **One error model.** Fallible functions return `Result<_, EngineError>` (`mjk-core`).
  Configuration errors name the exact field; numerical failures report sim time and quantity.
  Errors are never swallowed. Enforced by the lints above and review.
- **Declare all data.** Public items are documented (`missing_docs`); physical quantities carry
  units in their names (`mass_kg`, `omega_rad_s`), which review checks.
- **Transcendental functions go through `libm`**, so native and WebAssembly builds produce
  identical bits. Clippy rejects the standard library's versions, and `mul_add`, through
  `disallowed-methods` in `clippy.toml`; write `libm::fma` for an explicit fused multiply-add
  ([Determinism and replay](docs/src/determinism.md)).
- **Conventions are fixed**: NED world frame, FRD body frame, Hamilton scalar-first quaternions,
  SI units with unit suffixes. Use the symbols and equation IDs in
  [`docs/src/conventions.md`](docs/src/conventions.md).

The source checks (rules 2, 5, 8 and 9) cover every workspace crate, test code excepted.

**Exemption: `xtask`.** Repository tooling is held to every rule except recursion: it walks
syntax trees with `syn`'s visitor, which recurses with the depth of the source's own nesting.

**Exemption: `mjk-alloc-counter`.** The allocation test needs a counting global allocator, and
implementing `GlobalAlloc` is unsafe by definition. That one `unsafe impl` lives in its own
test-support crate, which sets `unsafe_code = "deny"` instead of `"forbid"`, allows it on the
impl alone, and requires a `SAFETY:` comment on every unsafe block. The layering check lets any
crate use it as a development dependency and no crate use it for anything else, so it never
reaches a build that ships. Cargo cannot override a single workspace lint, so the crate repeats
the whole lint table; `cargo xtask standards` fails if that copy drifts from the workspace's in
anything but those two lints.

### Mutation testing

A mutant is a small change to the code, such as `<` to `<=` or `+` to `-`. If every test still
passes, the tests do not pin that behavior. Mutation testing runs on the lines each change
touches, and every week on the whole engine; xtask and the two front-end crates are excluded
(`.cargo/mutants.toml`).

Fix a surviving mutant with a test that fails on it. A mutant that no test can distinguish,
because both versions compute the same result for every input (for example, the switch point
between a series and a closed form, where both are exact to rounding), goes in `exclude_re` in
`.cargo/mutants.toml` with a comment saying why. Patterns name the file and function, never a
line number.

### Supply chain

`deny.toml` fails the build on any RustSec advisory (vulnerable, unmaintained or unsound), a
yanked crate, a license outside MIT, Apache-2.0 and Unicode-3.0, two versions of one crate, a
wildcard version or a source other than crates.io. Changing the policy is a reviewed change to
that file.

npm dependencies are pinned to exact versions and `npm audit` must report nothing. Dependabot
proposes a release only once it is 14 days old, so a compromised or broken release has time to
be noticed; a dependency added by hand should be as old, unless only a newer release clears an
audit advisory (the current ESLint and typescript-eslint pins are such releases). Security
updates are never delayed.

## Crate layering and dependencies

Crates form layers (`mjk-core` → `mjk-math` → … → `mjk-sim` → front ends). `cargo xtask layering`
rejects any dependency outside a crate's allowed set, and engine crates may use only `libm`,
`serde`, `serde_path_to_error` and `toml` as external dependencies outside of tests. Adding a crate or an engine dependency means updating
the rule table in `xtask/src/layering.rs` and recording the decision in an architecture decision
record ([#6](https://github.com/MatthewK84/MJKPhysics/issues/6)).

## TypeScript and JavaScript coding standard

`viewer/eslint.config.js` governs the viewer and the repository's other scripts (the README hero
in `assets/readme` and the book's renderer in `docs/theme`). It builds on `typescript-eslint`'s
strict type-checked rules. Every `tsconfig.json` enables `strict` plus `noUncheckedIndexedAccess`,
`exactOptionalPropertyTypes`, `noUnusedLocals`, `noUnusedParameters` and related flags.
JavaScript files use `// @ts-check` with JSDoc types and are type-checked with `checkJs`.

| # | Rule | How it is enforced |
|---|---|---|
| 1 | Early returns and shallow control flow | `max-depth` 3, `complexity` 10, `no-else-return` |
| 2 | No global state; don't mutate inputs | `no-restricted-syntax` (assigning to `window`/`globalThis`), `no-param-reassign` |
| 3 | No legacy or unsafe features: `eval`, `with`, `arguments`, `document.write`, `any`, non-null `!`, `@ts-ignore` | `no-eval`, `no-implied-eval`, `no-new-func`, `no-with`, `prefer-rest-params`, `no-restricted-properties`, `no-restricted-globals`, `no-explicit-any`, `no-non-null-assertion`, `ban-ts-comment`; `lint:ui` also catches `any` in JSDoc |
| 4 | Block scoping: `const` by default, never `var`; no shadowing | `no-var`, `prefer-const`, `no-shadow`, `block-scoped-var` |
| 5 | Static analysis without suppressions | ESLint, Prettier and `tsc` in CI; inline `eslint-disable` comments are ignored and reported (`noInlineConfig`) |
| 6 | No deep recursion | Review: ESLint has no rule for call cycles |
| 7 | Small, pure functions | `max-lines-per-function` 50 |
| 8 | Explicit data shapes and signatures | `explicit-function-return-type`, `explicit-module-boundary-types` in TypeScript, and `consistent-type-assertions` (annotate object literals rather than assert them); in JavaScript, JSDoc `@param` and `@returns` checked by `tsc --checkJs`, with review for missing `@returns` |
| 9 | Consistent async error handling; no floating promises | `no-floating-promises`, `no-misused-promises`, `only-throw-error`, `prefer-promise-reject-errors` |
| 10 | Never modify built-in prototypes | `no-extend-native`, `no-prototype-builtins` |

### Interface standard

Interface code follows [ADR 0007](docs/src/adr/0007-interface-standard.md): tokens from one file,
no gradients, blur, radius above 4 px, shadows outside a token (for floating layers only),
decorative color, emoji or slow motion, and self-hosted assets. `npm run lint:ui` scans `viewer/src` and `docs/theme` and must report nothing. The
rendered 3D scene and the README hero are exempt.

### Python (planned)

Python arrives with the bindings in [#78](https://github.com/MatthewK84/MJKPhysics/issues/78).
It will follow the same ten principles with `ruff` and `mypy --strict`, configured in its
`pyproject.toml` and run in CI from the first commit.

## Physical models and documentation

Each physical model gets a documentation page with numbered equations, assumptions, validity range,
parameters and references, and code comments cite those equation numbers. Every model needs at least
one test against an analytic result or published data. Parameters in airframe files carry a unit, a
source and, where known, an uncertainty.

## License of contributions

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed under the MIT and
Apache-2.0 licenses, without any additional terms or conditions.
