// @ts-check
// Renders assets/readme/hero.html to assets/readme/hero.png with headless Chromium.
// Usage: node assets/readme/render-hero.mjs [scale]
// Requires the `playwright` package, installed locally or found through NODE_PATH:
//   NODE_PATH="$(npm root -g)" node assets/readme/render-hero.mjs
//
// Browsers do not load ES modules from file:// URLs, so the page and its script are served from
// a private origin by request interception. Nothing else is served and no port is opened.

import { createRequire } from 'node:module';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

/** @typedef {import('./playwright-api').Playwright} Playwright */
/** @typedef {import('./playwright-api').Browser} Browser */
/** @typedef {import('./playwright-api').Route} Route */

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ORIGIN = 'http://hero.invalid';
/** Request path to file name, for the only files the page may load. */
const SERVED = new Map([
  ['/hero.html', 'hero.html'],
  ['/hero.js', 'hero.js'],
]);
const WIDTH = 1280;
const HEIGHT = 640;
const DEFAULT_SCALE = 2.5;
const MAX_SCALE = 4;
const READY_TIMEOUT_MS = 15000;

/** @param {string | undefined} arg @returns {number} */
function parseScale(arg) {
  const value = Number(arg ?? DEFAULT_SCALE);
  if (!Number.isFinite(value) || value <= 0 || value > MAX_SCALE) {
    throw new Error(`scale must be in (0, ${String(MAX_SCALE)}], got ${String(arg)}`);
  }
  return value;
}

/** @param {unknown} value @returns {value is Playwright} */
function isPlaywright(value) {
  if (typeof value !== 'object' || value === null || !('chromium' in value)) return false;
  const { chromium } = value;
  return typeof chromium === 'object' && chromium !== null && 'launch' in chromium;
}

/** @returns {Playwright} */
function loadPlaywright() {
  /** @type {unknown} */
  let loaded;
  try {
    loaded = createRequire(import.meta.url)('playwright');
  } catch (error) {
    throw new Error('playwright is not installed; see the usage line in render-hero.mjs', {
      cause: error,
    });
  }
  if (!isPlaywright(loaded)) throw new Error('the playwright module has no chromium launcher');
  return loaded;
}

/** @param {Route} route @returns {Promise<void>} */
async function serve(route) {
  const name = SERVED.get(new URL(route.request().url()).pathname);
  if (name === undefined) {
    await route.abort();
    return;
  }
  await route.fulfill({ path: path.join(HERE, name) });
}

/** @param {Browser} browser @param {number} scale @returns {Promise<string>} the PNG path */
async function capture(browser, scale) {
  const viewport = { width: WIDTH, height: HEIGHT };
  const page = await browser.newPage({ viewport, deviceScaleFactor: scale });
  /** @type {string[]} */
  const errors = [];
  page.on('pageerror', (error) => {
    errors.push(error.message);
  });
  await page.route(`${ORIGIN}/**`, serve);
  await page.goto(`${ORIGIN}/hero.html`);
  await page.waitForSelector('#hero[data-ready="true"]', { timeout: READY_TIMEOUT_MS });
  if (errors.length > 0) throw new Error(`page errors: ${errors.join('; ')}`);
  const out = path.join(HERE, 'hero.png');
  await page.locator('#hero').screenshot({ path: out });
  return out;
}

/** @param {Playwright} playwright @param {number} scale @returns {Promise<void>} */
async function render(playwright, scale) {
  const browser = await playwright.chromium.launch();
  try {
    const out = await capture(browser, scale);
    console.log(`wrote ${out} at ${String(WIDTH * scale)}x${String(HEIGHT * scale)}`);
  } finally {
    await browser.close();
  }
}

try {
  await render(loadPlaywright(), parseScale(process.argv[2]));
} catch (error) {
  console.error(error instanceof Error ? error.message : String(error));
  process.exitCode = 1;
}
