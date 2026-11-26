# ADR 0004: Rust physics in WebAssembly, rendered by three.js with WebGPU

- **Status:** Accepted
- **Date:** 2026-11-26
- **Issues:** [#32](https://github.com/MatthewK84/MJKPhysics/issues/32), [#34](https://github.com/MatthewK84/MJKPhysics/issues/34), [#83](https://github.com/MatthewK84/MJKPhysics/issues/83)

## Context

The project aims for modern game-engine visual quality and a live demo anyone can open from the
README. The physics is the part that must be rigorous; the renderer must never be able to change
a simulated result.

## Decision

- The engine compiles to WebAssembly (`mjk-wasm`) and runs in a Web Worker. It exchanges
  fixed-layout state snapshots and pilot inputs with the page as transferable buffers. GitHub
  Pages cannot send the headers that `SharedArrayBuffer` requires.
- The viewer is TypeScript with three.js, using the WebGPU renderer and falling back to WebGL2.
  Conversion from world coordinates happens in one module (CONV-4, CONV-5).
- The documentation and the live demo share one GitHub Pages site: the book at the root and the
  demo under `/demo/`.

## Consequences

- Anyone can run the demo by opening a link.
- Physics and rendering are separated by a data interface, so either side can be tested alone.
  Native and WebAssembly results are compared bit for bit in CI.
- Two languages: rendering code follows the project's TypeScript standard and gets its own lint,
  type checks and visual regression tests.
- Visual quality is bounded by what WebGPU and WebGL2 offer in browsers, and quality tiers handle
  weaker hardware.

## Alternatives considered

- **Bevy (all Rust).** One language and a capable renderer, but large WebAssembly builds,
  breaking releases every few months, and heavy compile times. It would also pull the engine
  toward its ECS architecture.
- **A custom wgpu renderer.** The most impressive to graphics specialists, but the renderer would
  become the project.
- **Unreal Engine or Unity.** High fidelity, but not web-native, with licensing and tooling that
  don't fit an open, reproducible research codebase.
