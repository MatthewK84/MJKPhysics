# ADR 0005: GitHub-native math syntax, rendered in the book by KaTeX

- **Status:** Accepted
- **Date:** 2026-11-26
- **Issues:** [#5](https://github.com/MatthewK84/MJKPhysics/issues/5), [#6](https://github.com/MatthewK84/MJKPhysics/issues/6)

## Context

Documentation pages are read in two places: on GitHub, and in the mdBook site. Writing the
[conventions](../conventions.md) page showed that GitHub's renderer has four relevant quirks:

- It applies Markdown backslash escapes inside `$$` blocks, so `\;` and `\!` appear as visible
  punctuation and mid-line `\\` row breaks are lost.
- It rejects `\operatorname`.
- `\tag` breaks equation layout in Chromium.
- In `$…$` inline math, subscripts with underscores can be read as emphasis.

## Decision

- Display equations are fenced code blocks with the language `math`, and inline equations are
  written `` $`…`$ ``, a code span wrapped in dollar signs. Markdown never processes code, so the
  TeX arrives intact on GitHub and in the book.
- Equation IDs are written as a trailing `\qquad\text{(ID)}`, not with `\tag`.
- Use `\mathrm{…}` instead of `\operatorname{…}`.
- In the book, `docs/theme/math.js` renders both forms with KaTeX 0.19.1. KaTeX is loaded from a
  CDN at a pinned version with subresource integrity. The script also gives each tagged display
  equation the anchor `eq-ID`.

## Consequences

- A page renders correctly on GitHub and in the book from one source.
- Without JavaScript, the book shows readable TeX source in code blocks, so it degrades gracefully.
- Code comments can cite and link equations by stable ID. `cargo xtask equations` checks that IDs
  are unique and that every citation exists.
- The book depends on the CDN at view time. If KaTeX fails to load, the TeX is shown as text.

## Alternatives considered

- **`$$` and `$` delimiters with the mdbook-katex preprocessor.** Server-side rendering is
  attractive, but pages written that way render incorrectly on GitHub.
- **mdBook's built-in MathJax support.** It needs `\\(` and `\\[` delimiters, which GitHub doesn't
  understand, and MathJax is heavier than KaTeX.
- **Vendoring KaTeX into the repository.** It works offline, but adds fonts and minified
  JavaScript to the repository. A pinned CDN copy with integrity hashes gives the same guarantees
  about what code runs.
