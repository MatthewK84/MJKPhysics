// @ts-check
// Runs a WASI build of the `mjk` command-line tool under Node (#14), so CI can compare its output
// with the native build's byte for byte:
//
//   node scripts/run-wasi.mjs <module.wasm> [arguments...]
//
// The module gets its arguments and the standard streams, and nothing else: no files, no
// environment variables and no preopened directories. Its exit code becomes this script's.

import { readFile } from 'node:fs/promises';
import process from 'node:process';
import { WASI } from 'node:wasi';

const USAGE = 'usage: node scripts/run-wasi.mjs <module.wasm> [arguments...]';

/**
 * Runs the WASI command module at `modulePath` with `args`.
 * @param {string} modulePath Path to a `wasm32-wasip1` command module.
 * @param {readonly string[]} args Arguments after the program name.
 * @returns {Promise<number>} The module's exit code.
 */
async function run(modulePath, args) {
  const wasi = new WASI({
    version: 'preview1',
    args: ['mjk', ...args],
    env: {},
    returnOnExit: true,
  });
  const module = await WebAssembly.compile(await readFile(modulePath));
  const instance = await WebAssembly.instantiate(module, wasi.getImportObject());
  return wasi.start(instance);
}

/**
 * Reads the command line, runs the module and sets the exit code.
 * @returns {Promise<void>}
 */
async function main() {
  const [modulePath, ...args] = process.argv.slice(2);
  if (modulePath === undefined) {
    console.error(USAGE);
    process.exitCode = 2;
    return;
  }
  try {
    process.exitCode = await run(modulePath, args);
  } catch (error) {
    console.error(`error: ${error instanceof Error ? error.message : String(error)}`);
    process.exitCode = 1;
  }
}

await main();
