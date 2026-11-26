# Writing documentation

Pages are Markdown in `docs/src/`, built with mdBook 0.5.4. They must read correctly both on
GitHub and in the book ([ADR 0005](adr/0005-documentation-math.md)).

## Math

- **Display equations** are fenced blocks with the language `math`, ending with the equation's ID:

  ````markdown
  ```math
  \dot{\mathbf{q}}_{WB} = \tfrac{1}{2}\,\mathbf{q}_{WB}\otimes(0,\ \boldsymbol{\omega}^B)
  \qquad\text{(CONV-9)}
  ```
  ````

- **Inline math** is a code span wrapped in dollar signs: `` $`\alpha`$ `` renders as $`\alpha`$.
- **Not allowed:** `$$` blocks and plain `$…$` (GitHub mangles them), `\tag` (breaks layout), and
  `\operatorname` (rejected by GitHub; use `\mathrm{…}`).
- Use the symbols from the [conventions](conventions.md). If a source uses different notation,
  give the mapping on the page.

## Equation IDs and citations

- An ID is a page prefix from the [models registry](models/README.md), a hyphen, and a number:
  `BEMT-4`. IDs are never reused or renumbered once published. New equations take the next free
  number, even if that means the numbers on a page are out of order.
- Each tagged equation has an anchor, so pages link to it as
  [`conventions.md#eq-CONV-9`](conventions.md#eq-CONV-9).
- Code cites equations in comments as `// Eq. (CONV-9)`.
- `cargo xtask equations` fails if an ID is duplicated, malformed, or cited from code without
  existing in the docs. CI runs it on every pull request.

## Style

- Plain, direct sentences. Define a term before using it.
- State assumptions and validity ranges with numbers.
- Cite sources in full at the end of the page. Link issues by URL so links work outside GitHub.

## Building and previewing

```bash
cargo install mdbook --version 0.5.4 --locked
```

```bash
mdbook serve docs --open
```

CI builds the book on every pull request and publishes it to GitHub Pages from `main`.
