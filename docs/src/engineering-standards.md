# Engineering standards

A flight dynamics engine is only as credible as the evidence that its code does what its pages
say. This page describes the rules the code follows, how each one is enforced, and how the tests
themselves are tested. [CONTRIBUTING.md](https://github.com/MatthewK84/MJKPhysics/blob/main/CONTRIBUTING.md)
has the same rules as a checklist, with the commands to run them.

The rules come from the Power of Ten, NASA/JPL's rules for safety-critical code, adapted to Rust,
TypeScript and Python. The adaptation keeps their purpose: code whose control flow, memory use and
failure modes can be checked by a tool rather than trusted.

## When the checks run

| When | What runs |
|---|---|
| Every pull request and push to `main` | Formatting, Clippy, tests, crate layering, the Power of Ten checks, cargo-deny, spelling, coverage, mutation testing of the changed lines, API docs, the WebAssembly build, the viewer and script checks, the book build |
| Daily | The same suite, so a newly published advisory against a locked dependency fails within a day |
| Weekly | Tests on macOS and Windows; mutation testing of the whole engine; Dependabot updates |

## Rust: the Power of Ten

| # | Rule | Enforced by |
|---|---|---|
| 1 | Simple control flow, no recursion | Clippy `excessive_nesting`; call-graph cycle check |
| 2 | Every loop has a fixed upper bound | `// bound:` comment on every `loop` and `while`; Clippy `infinite_loop` |
| 3 | No heap allocation after initialization | Allocation-counting test on `World::step` ([#13](https://github.com/MatthewK84/MJKPhysics/issues/13)); review until then |
| 4 | Functions of at most 50 lines | Clippy `too_many_lines` |
| 5 | Assertions state preconditions | Review, with assertion density reported per crate |
| 6 | Smallest possible scope | Compiler; Clippy `shadow_unrelated` |
| 7 | Check every return value | `unused_must_use`, `let_underscore_must_use`, `let_underscore_untyped`; no `unwrap`, `expect` or `panic!` |
| 8 | Limited preprocessor | No `macro_rules!`, no conditional compilation outside tests; serde's derive is the only procedural macro in the engine |
| 9 | Restricted pointers | `unsafe` forbidden; no `dyn` trait objects or function pointer types |
| 10 | All warnings on, analyzed daily | `-D warnings`, Clippy `pedantic`, the gates below, daily CI |

Rules 2, 8 and 9 and the assertion report come from `cargo xtask standards`, which parses every
crate's source with `syn`. Test code is exempt from them.

### Finding recursion in compiled code

A call graph built from source needs type inference to know which function a method call reaches,
and the compiler has already done that work. So the recursion check reads the compiler's output:
it builds every workspace target unoptimized, with one codegen unit, as LLVM IR. In that IR every
direct call is explicit, including calls through generics (after monomorphization), trait methods
and closures. A breadth-first search from each of our functions looks for a path back to itself,
and the shortest such cycle is reported with demangled names, for example
`mjk_dynamics::ping → mjk_dynamics::Pong::pong → mjk_dynamics::ping`.

Cycles entirely inside the standard library, such as the recursion in its sort, are ignored. A
cycle that leaves our code and comes back through a library callback is reported. Calls through
trait objects or function pointers would be invisible in IR, which is one reason rule 9 bans
them: the two rules close each other's gaps.

The check was shown to catch direct recursion, mutual recursion through a method, and each of the
other source rules, on a deliberately bad sample, before it was enabled.

The one exemption is `xtask`, the repository's tooling. It walks syntax trees with `syn`'s
visitor, which recurses with the depth of the source's own nesting.

### Nesting

Clippy's `excessive_nesting` counts an `impl` block as a level. With the threshold at 4, a free
function may nest three levels deep and a method two. Deeper logic moves into a named helper.

### Assertions

Engine functions that take physical quantities check their preconditions with `debug_assert!`:
a unit quaternion's norm, the domain $`\lVert\boldsymbol\theta\rVert < 2\pi`$ of the SO(3)
Jacobians, a finite interpolation parameter, a symmetric matrix for the eigen-decomposition.
There is no per-function quota, because a quota produces assertions that check nothing. The
density is reported per crate so a drop shows up in review.

## Testing the tests

Passing tests show that the code does what the tests check. Two gates measure how much that is.

- **Coverage.** Engine line coverage must stay at or above 90 %. Coverage shows which lines ran,
  not whether any test would notice if they were wrong.
- **Mutation testing** answers that. `cargo-mutants` makes small changes to the code, one at a
  time: `<` to `<=`, `+` to `-`, a function body replaced by a default value. If every test
  still passes, the mutant has survived, and the tests do not pin that behavior. CI mutates
  the lines each change touches, and any survivor fails it. A weekly run mutates the whole engine.

### What mutation testing found

When the gate was introduced, a full run made 1,369 mutants of the engine, and 144 survived,
though line coverage was already above 90 %. The largest group, 65 survivors, was in the
inertia-tensor check. Mutating its 2×2 and 3×3 principal minors, or the tolerance attached to
each, changed nothing that any test observed.

The check accepts a symmetric tensor $`J`$ as the inertia of a rigid body when the second-moment
matrix $`S = \tfrac12\,\mathrm{tr}(J)\,I - J`$ is positive semidefinite. That is equivalent to the
triangle inequality on the principal moments. It tests all seven principal minors of $`S`$, each
against an allowance of the relative tolerance times the tensor's largest entry to the power of
the minor's order. The existing tests violated the triangle inequality so badly that the first
diagonal minor always caught it, so nothing tested the rest.

The new tests are built so that exactly one kind of minor can detect the violation:

- **A diagonal minor.** For a plate with largest moment $`L`$ that violates the inequality by
  $`\Delta`$, $`S`$ has eigenvalues $`(L/2,\ L/2,\ -\Delta/2)`$. A diagonal entry crosses its
  allowance at $`\Delta = 2\,\mathrm{tol}\,L`$, a 2×2 minor at $`4\,\mathrm{tol}\,L`$, and the
  determinant at $`8\,\mathrm{tol}\,L`$. A plate with $`\Delta = 3\,\mathrm{tol}\,L`$ is
  rejected, by the diagonal alone, and one with $`\Delta = \mathrm{tol}\,L`$ is accepted.
- **A 2×2 minor.** Give $`S`$ eigenvalues $`(-\delta,\ a,\ b)`$ and mix the first two at 45° in
  one coordinate plane. The diagonal stays positive, and the determinant $`-\delta a b`$ stays
  inside its allowance. Only that plane's minor, $`-a\delta`$, exceeds its own allowance. One
  case per plane covers all three minors.
- **The determinant.** In a general orientation, with a violation small against the other
  eigenvalues, every 2×2 minor is positive. Each equals $`\det S \cdot (S^{-1})_{kk}`$, and both
  factors are negative. Only the determinant detects the violation.

Each case also sits within about a factor of ten of its allowance, at a realistic tensor scale of
0.01 kg·m², so a mutant that divides a tolerance by the tensor's size instead of multiplying by
it, or drops a power of it, is caught. After these and the other new tests
(listed on the [Validation](validation/README.md) page), no mutant of the engine survives.

### Mutants no test can catch

Some mutants compute the same result as the original for every input. For example, `<` becomes
`<=` at the switch point between a series and its closed form, where both are exact to rounding.
These are excluded by name in `.cargo/mutants.toml`, each with a comment saying why no test can
tell the two versions apart. The list is kept short, and every entry carries its reason.

An exclusion can also expose code that does nothing measurable. Two mutants of the small-angle
logarithm survived because they replaced $`q_0^2`$ by 1 in the series
$`(2/q_0)\,(1 - n^2/(3q_0^2))`$. For a unit quaternion $`q_0^2 = 1 - n^2`$, so the factor
contributes only $`O(n^4)`$, under $`10^{-24}`$ where the series is used. The code now evaluates
$`(2/q_0)(1 - n^2/3)`$, which has the same truncation order and no equivalent mutants
([SO(3) maps](so3.md)).

## Spelling

The project writes American English. `typos`, with its US locale, runs on every change and fails
on misspellings and on British spellings, in prose and inside identifiers: a British spelling in
the middle of a function name fails as readily as one in a sentence. Project terms such as NED and
SITL, and the name Shepperd, are allowed in `typos.toml`.

## Supply chain

- **Rust.** `cargo-deny` fails the build on any RustSec advisory (vulnerable, unmaintained or
  unsound crates), a yanked crate, a license outside MIT, Apache-2.0 and Unicode-3.0, two versions
  of one crate, a wildcard version, or a source other than crates.io. Engine crates may depend on
  only four external crates (`libm`, `serde`, `serde_path_to_error`, `toml`), enforced by the
  layering check.
- **npm.** Versions are pinned exactly, and `npm audit` must report nothing.
- **Updates.** Dependabot proposes a release only once it is 14 days old, so a compromised or
  broken release has time to be noticed. Security updates are not delayed.
- **CI.** Third-party actions are pinned to full commit SHAs, and checkouts don't keep
  credentials.

## TypeScript and JavaScript

The viewer, the README hero script and the book's equation renderer share one ESLint policy,
built on typescript-eslint's strict type-checked rules, and strict `tsconfig` settings. JavaScript
files are type-checked from their JSDoc annotations. Functions are limited to 50 lines and three
levels of nesting, with no `any`, non-null assertions, `eval`, shadowing or floating promises.
Inline `eslint-disable` comments are ignored and reported, so no file can switch a rule off.

ESLint cannot see call cycles, so recursion is a review item in TypeScript.

## Interface

Interface code follows [ADR 0007](adr/0007-interface-standard.md): one token file for every
color, size and spacing value; no gradients, blur, radius above 4 px, shadows except on floating
layers, decorative color, emoji or slow motion; and self-hosted assets. A scan for the banned patterns runs in CI over the viewer and
this book's theme. The rendered 3D scene is exempt, because a physically based sky is rendering,
not decoration.

## Python

Python arrives with the bindings ([#78](https://github.com/MatthewK84/MJKPhysics/issues/78)). It
will follow the same principles with `ruff` and `mypy --strict`, in CI from its first commit.

## What review still covers

| Item | Why a tool doesn't check it |
|---|---|
| Heap allocation in `World::step` | The allocation-counting test needs the world loop ([#13](https://github.com/MatthewK84/MJKPhysics/issues/13)) |
| Whether an assertion states a real precondition | Only a reader can judge what a precondition should be |
| Units in names of physical quantities (`mass_kg`, `omega_rad_s`) | Names carry meaning a linter cannot infer |
| Recursion in TypeScript | ESLint has no call-graph rule |
| `@returns` on JavaScript functions | ESLint's return-type rules read TypeScript syntax only |
