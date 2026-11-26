# MJKPhysics

MJKPhysics is a six-degree-of-freedom flight dynamics engine for small unmanned aircraft, written
in Rust and rendered in the browser. This book is its engineering record. For every physical model
it gives the equations, the assumptions behind them, the range where they hold, the sources of
every parameter, and how the implementation was checked.

> **Status: pre-alpha.** The book grows with the
> [roadmap](https://github.com/MatthewK84/MJKPhysics/issues/1). Pages for models that don't exist
> yet are listed with the issue that will add them.

## How the book is organised

- **Foundations.** [Conventions and notation](conventions.md) fixes frames, rotation conventions,
  signs, units and symbols; everything else uses them. [Architecture](architecture.md) describes
  the crates, their layering and the rules the engine follows.
- **Physics.** [Models](models/README.md) has one page per physical model.
  [Validation](validation/README.md) collects the evidence that each model is implemented
  correctly. [Airframes](airframes/README.md) documents the four modelled aircraft and the source
  of every parameter.
- **Project.** [Decision records](adr/README.md) explain significant design choices and the
  alternatives that were rejected. [Writing documentation](writing-docs.md) covers the house style
  for pages, equations and citations.

## Equations and citations

Display equations carry stable IDs such as (CONV-3), and each one has an anchor: this book links
to it as `conventions.html#eq-CONV-3`. Code cites equations in comments as `// Eq. (CONV-3)`, and
`cargo xtask equations` fails if a citation points to an equation that doesn't exist, or if two
equations share an ID.

## Building the book locally

```bash
cargo install mdbook --version 0.5.4 --locked
```

```bash
mdbook serve docs --open
```
