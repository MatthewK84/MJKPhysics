// Renders assets/readme/hero.html to assets/readme/hero.png with headless Chromium.
// Usage: node assets/readme/render-hero.mjs [scale]
// Requires the `playwright` package, installed locally or found through NODE_PATH:
//   NODE_PATH="$(npm root -g)" node assets/readme/render-hero.mjs

import { createRequire } from 'node:module';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';

const { chromium } = createRequire(import.meta.url)('playwright');

const HERE = path.dirname(fileURLToPath(import.meta.url));
const WIDTH = 1280;
const HEIGHT = 640;
const DEFAULT_SCALE = 2.5;

/** @param {string | undefined} arg @returns {number} */
function parseScale(arg) {
  const value = Number(arg ?? DEFAULT_SCALE);
  if (!Number.isFinite(value) || value <= 0 || value > 4) {
    throw new Error(`scale must be in (0, 4], got ${String(arg)}`);
  }
  return value;
}

/** @param {number} scale @returns {Promise<void>} */
async function render(scale) {
  const browser = await chromium.launch();
  try {
    const page = await browser.newPage({ viewport: { width: WIDTH, height: HEIGHT }, deviceScaleFactor: scale });
    const errors = [];
    page.on('pageerror', (error) => errors.push(error.message));
    await page.goto(pathToFileURL(path.join(HERE, 'hero.html')).href);
    await page.waitForSelector('#hero[data-ready="true"]', { timeout: 15000 });
    if (errors.length > 0) throw new Error(`page errors: ${errors.join('; ')}`);
    const out = path.join(HERE, 'hero.png');
    await page.locator('#hero').screenshot({ path: out });
    console.log(`wrote ${out} at ${WIDTH * scale}x${HEIGHT * scale}`);
  } finally {
    await browser.close();
  }
}

try {
  await render(parseScale(process.argv[2]));
} catch (error) {
  console.error(error instanceof Error ? error.message : error);
  process.exitCode = 1;
}
