// The parts of the WebAssembly JavaScript API that run-wasi.mjs uses. TypeScript declares the
// `WebAssembly` global only in its browser libraries, which these Node scripts do not load.

declare namespace WebAssembly {
  /** A compiled module. JavaScript sees no members on it; the script only passes it on. */
  type Module = object;

  /** An instance of a compiled module. */
  interface Instance {
    readonly exports: Readonly<Record<string, unknown>>;
  }

  /** Compiles a module from its binary. */
  function compile(bytes: Uint8Array): Promise<Module>;

  /** Instantiates a compiled module with its imports. */
  function instantiate(module: Module, imports: object): Promise<Instance>;
}
