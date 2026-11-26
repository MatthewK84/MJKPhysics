# Decision records

An architecture decision record (ADR) captures one significant design choice: the context, the
decision, its consequences, and the alternatives that were considered and rejected. ADRs are never
rewritten after acceptance. A changed decision gets a new ADR that supersedes the old one, so the
history of the reasoning is kept.

Write an ADR when a choice is expensive to reverse, or when a reader would reasonably ask why it was
done this way. Examples: adding an external dependency to an engine crate, choosing a numerical
method, or changing a convention. Start from the [template](../templates/decision-record.md) and
number it sequentially.

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-hand-written-math.md) | Hand-written, frame-typed math; no third-party math or physics crates in the engine | Accepted |
| [0002](0002-integrator-design.md) | Fixed-step integration behind a trait, with RK4 and a Lie-group method for attitude | Accepted |
| [0003](0003-contact-solver.md) | Sequential impulses with warm starting and soft constraints for contact | Accepted |
| [0004](0004-renderer-stack.md) | Rust physics compiled to WebAssembly, drawn by three.js with WebGPU | Accepted |
| [0005](0005-documentation-math.md) | GitHub-native math syntax, rendered in the book by KaTeX | Accepted |
| [0006](0006-errors-and-configuration.md) | One engine error type in `mjk-core`; configuration parsed from TOML text with exact field paths | Accepted |
| [0007](0007-interface-standard.md) | Operator-grade interface rules for the viewer and the book; the 3D scene and README hero are exempt | Accepted |

Status values: **Proposed**, **Accepted**, **Superseded by ADR NNNN**, **Rejected**. An accepted
ADR may also say **extended by ADR NNNN** when a later record adds to it without changing what it
decided; that note is the only edit an accepted ADR receives.
