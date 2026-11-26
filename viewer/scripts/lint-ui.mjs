// @ts-check
// Banned-pattern scan for interface code (operator-grade-ui, section 9): `npm run lint:ui`.
//
// Scans the viewer's source and the book's theme, the code that draws interface chrome. The
// rendered 3D scene and the README hero are exempt (ADR 0007). The patterns are the skill's,
// plus their plain-CSS and JSDoc equivalents, since this project uses neither Tailwind nor a
// component library. Every hit is listed; zero hits is the pass condition.

import { readdirSync, readFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

/** @typedef {{ name: string, pattern: RegExp, onlyIn?: RegExp, exceptIn?: RegExp }} Rule */
/** @typedef {{ file: string, line: number, rule: string, text: string }} Hit */

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const SCANNED = ['viewer/src', 'docs/theme'];
const EXTENSIONS = /\.(?:css|hbs|html|js|mjs|ts)$/;
const SKIPPED_DIRS = new Set(['node_modules', 'dist']);
/** Upper bound on files visited, so the walk always terminates. */
const MAX_FILES = 10000;
const CSS = /\.css$/;
const TOKEN_FILE = /(?:^|\/)tokens\.css$/;

/** @type {readonly Rule[]} */
const RULES = [
  { name: 'gradient', pattern: /gradient/i },
  { name: 'backdrop blur', pattern: /backdrop-(?:blur|filter)/ },
  { name: 'border radius above 4px', pattern: /border-radius\s*:[^;]*\b(?:[5-9]|\d{2,})px/ },
  { name: 'border radius in relative units', pattern: /border-radius\s*:[^;]*\d(?:rem|em|%)/ },
  { name: 'rounded utility', pattern: /\brounded-(?:md|lg|xl|2xl|3xl)\b/ },
  { name: 'shadow utility', pattern: /\bshadow-(?:md|lg|xl|2xl)\b/ },
  { name: 'purple-family utility', pattern: /\b(?:indigo|violet|purple|fuchsia)-\d{2,3}\b/ },
  {
    name: 'purple-family named color',
    pattern: /\b(?:indigo|violet|purple|fuchsia|blueviolet|darkviolet|rebeccapurple)\b/i,
    onlyIn: CSS,
  },
  { name: 'oversized text utility', pattern: /\btext-[3-9]xl\b/ },
  { name: 'bounce or pulse animation', pattern: /\banimate-(?:bounce|pulse)\b/ },
  {
    name: 'hard-coded color outside tokens.css',
    pattern: /#[0-9a-fA-F]{3,8}\b|\b(?:rgba?|hsla?)\(/,
    exceptIn: TOKEN_FILE,
  },
  { name: 'transition above 150ms', pattern: /\b(?:15[1-9]|1[6-9]\d|[2-9]\d\d|\d{4,})ms\b/ },
  { name: 'emoji', pattern: /[\u{1F300}-\u{1FAFF}\u{2600}-\u{27BF}]/u },
  { name: 'suppression marker', pattern: /@ts-(?:ignore|expect-error|nocheck)|eslint-disable/ },
  { name: 'any type', pattern: /:\s*any\b|\bas\s+any\b|<any>|\{\s*(?:any|\*)\s*\}/ },
];

/** @param {string} dir absolute directory @returns {string[]} absolute paths of scanned files */
function listFiles(dir) {
  /** @type {string[]} */
  const found = [];
  const pending = [dir];
  // bound: each iteration removes one directory; at most MAX_FILES entries are ever queued.
  while (pending.length > 0 && found.length < MAX_FILES) {
    const current = pending.pop() ?? '';
    for (const entry of readdirSync(current, { withFileTypes: true })) {
      const full = path.join(current, entry.name);
      if (entry.isDirectory() && !SKIPPED_DIRS.has(entry.name)) pending.push(full);
      if (entry.isFile() && EXTENSIONS.test(entry.name)) found.push(full);
    }
  }
  return found;
}

/** @param {Rule} rule @param {string} file repository-relative path @returns {boolean} */
function applies(rule, file) {
  const included = rule.onlyIn?.test(file) ?? true;
  const excluded = rule.exceptIn?.test(file) ?? false;
  return included && !excluded;
}

/** @param {string} file repository-relative path @param {string} text @returns {Hit[]} */
function scanText(file, text) {
  const rules = RULES.filter((rule) => applies(rule, file));
  return text
    .split('\n')
    .flatMap((line, index) =>
      rules
        .filter((rule) => rule.pattern.test(line))
        .map((rule) => ({ file, line: index + 1, rule: rule.name, text: line.trim() })),
    );
}

/** @returns {Hit[]} every hit in the scanned directories, in path order */
function scanRepository() {
  return SCANNED.flatMap((dir) => listFiles(path.join(ROOT, dir)))
    .sort()
    .flatMap((full) => {
      const file = path.relative(ROOT, full).split(path.sep).join('/');
      return scanText(file, readFileSync(full, 'utf8'));
    });
}

try {
  const hits = scanRepository();
  for (const hit of hits) {
    console.error(`${hit.file}:${String(hit.line)}: ${hit.rule}: ${hit.text}`);
  }
  if (hits.length > 0) {
    console.error(`lint:ui found ${String(hits.length)} banned pattern(s)`);
    process.exitCode = 1;
  } else {
    console.log(`lint:ui ok: ${SCANNED.join(', ')}`);
  }
} catch (error) {
  console.error(
    `lint:ui could not scan: ${error instanceof Error ? error.message : String(error)}`,
  );
  process.exitCode = 1;
}
