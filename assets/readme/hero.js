// @ts-check
// Draws the README hero into the page's #hero SVG; render-hero.mjs captures it as hero.png.
// The quadrotor is a true perspective projection of a body-frame model, so its frames, vectors
// and spin directions follow the documented conventions: world NED, body FRD, PX4 quad-X motor
// order.

/** @typedef {{ x: number, y: number, z: number }} Vec3 */
/** @typedef {{ x: number, y: number }} Point */
/** @typedef {{ x: number, y: number, depth: number }} Projected */
/** @typedef {readonly [Vec3, Vec3, Vec3]} Mat3 rows of a body-to-world rotation */
/** @typedef {{ position: Vec3, forward: Vec3, right: Vec3, up: Vec3, focalPx: number, cx: number, cy: number }} Camera */
/** @typedef {{ number: number, azimuthDeg: number, ccw: boolean, thrust: number, bladeDeg: number }} Motor */
/** @typedef {{ text: string, kind: 'normal' | 'sub' | 'sup' }} Segment */
/** @typedef {Record<string, string | number>} Attrs */
/** @typedef {{ width?: number, head?: number, dash?: string, opacity?: number }} ArrowStyle */
/** @typedef {readonly (readonly [number, number])[]} Stops offset and opacity pairs */
/** @typedef {(b: Vec3) => Projected} ToScreen */
/**
 * @typedef {{ x: number, y: number, fill: string, family: string, size: number, weight?: number,
 *   spacing?: number, text: string }} TextLine
 */
/** @typedef {{ cg: Projected, toScreen: ToScreen, world: ToScreen }} Rig projections of the quadrotor */

const SVG_NS = 'http://www.w3.org/2000/svg';
const DEG = Math.PI / 180;
const SANS = "'Inter', 'Inter Display', sans-serif";
const MONO = "'JetBrains Mono', monospace";

const COLOR = Object.freeze({
  bgTop: '#081022',
  bgBottom: '#0b1733',
  grid: 'rgba(126,162,255,0.06)',
  gridMajor: 'rgba(126,162,255,0.12)',
  line: '#cfe0ff',
  dim: '#8ea3c7',
  faint: 'rgba(142,163,199,0.55)',
  text: '#eef3ff',
  panel: 'rgba(9,18,40,0.72)',
  panelStroke: 'rgba(140,170,255,0.20)',
  frame: '#132243',
  frameTop: '#1b2f5a',
  thrust: '#5ee1ff',
  weight: '#ffb547',
  aero: '#ff6bb5',
  omega: '#b79bff',
  vel: '#e9f0ff',
  xAxis: '#ff8080',
  yAxis: '#7cf29c',
  zAxis: '#78afff',
});

// ------------------------------------------------------------------ vector math (pure)

/** @param {number} x @param {number} y @param {number} z @returns {Vec3} */
const vec = (x, y, z) => ({ x, y, z });
/** @param {Vec3} a @param {Vec3} b @returns {Vec3} */
const add = (a, b) => vec(a.x + b.x, a.y + b.y, a.z + b.z);
/** @param {Vec3} a @param {Vec3} b @returns {Vec3} */
const sub = (a, b) => vec(a.x - b.x, a.y - b.y, a.z - b.z);
/** @param {Vec3} a @param {number} s @returns {Vec3} */
const mul = (a, s) => vec(a.x * s, a.y * s, a.z * s);
/** @param {Vec3} a @param {Vec3} b @returns {number} */
const dot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
/** @param {Vec3} a @param {Vec3} b @returns {Vec3} */
const cross = (a, b) => vec(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
/** @param {Vec3} a @returns {Vec3} */
const unit = (a) => mul(a, 1 / Math.hypot(a.x, a.y, a.z));

/**
 * Body (FRD) to world (NED) rotation from 3-2-1 Euler angles.
 * @param {number} yaw @param {number} pitch @param {number} roll radians
 * @returns {Mat3}
 */
function bodyToWorld(yaw, pitch, roll) {
  const [cy, sy, cp, sp, cr, sr] = [
    Math.cos(yaw),
    Math.sin(yaw),
    Math.cos(pitch),
    Math.sin(pitch),
    Math.cos(roll),
    Math.sin(roll),
  ];
  return [
    vec(cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
    vec(sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
    vec(-sp, cp * sr, cp * cr),
  ];
}

/** @param {Mat3} m @param {Vec3} v @returns {Vec3} */
const rotate = (m, v) => vec(dot(m[0], v), dot(m[1], v), dot(m[2], v));

/**
 * @param {Vec3} position @param {Vec3} target @param {number} focalPx @param {number} cx
 * @param {number} cy
 * @returns {Camera}
 */
function makeCamera(position, target, focalPx, cx, cy) {
  const forward = unit(sub(target, position));
  const right = unit(cross(forward, vec(0, 0, -1)));
  return { position, forward, right, up: cross(right, forward), focalPx, cx, cy };
}

/** @param {Camera} cam @param {Vec3} p @returns {Projected} */
function project(cam, p) {
  const rel = sub(p, cam.position);
  const depth = dot(rel, cam.forward);
  const s = cam.focalPx / depth;
  return { x: cam.cx + dot(rel, cam.right) * s, y: cam.cy - dot(rel, cam.up) * s, depth };
}

// ------------------------------------------------------------------ SVG helpers

/** @param {Element} parent @param {string} tag @param {Attrs} attrs @returns {Element} */
function el(parent, tag, attrs) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [key, value] of Object.entries(attrs)) node.setAttribute(key, String(value));
  parent.appendChild(node);
  return node;
}

/** @param {Element} parent @param {TextLine} line @returns {Element} */
function textLine(parent, line) {
  const node = el(parent, 'text', {
    x: line.x,
    y: line.y,
    fill: line.fill,
    'font-family': line.family,
    'font-size': line.size,
    ...(line.weight === undefined ? {} : { 'font-weight': line.weight }),
    ...(line.spacing === undefined ? {} : { 'letter-spacing': line.spacing }),
  });
  node.textContent = line.text;
  return node;
}

/** @param {readonly Point[]} points @param {boolean} closed @returns {string} */
const pathOf = (points, closed) =>
  points.map((p, i) => `${i === 0 ? 'M' : 'L'}${p.x.toFixed(2)} ${p.y.toFixed(2)}`).join(' ') +
  (closed ? ' Z' : '');

/**
 * Parse `x_{b}` and `10^{-4}` style markup into segments, without recursion.
 * @param {string} markup @returns {Segment[]}
 */
function parseMarkup(markup) {
  /** @type {Segment[]} */
  const segments = [];
  let index = 0;
  let buffer = '';
  while (index < markup.length) {
    const ch = markup[index] ?? '';
    const isScript = (ch === '_' || ch === '^') && markup[index + 1] === '{';
    if (!isScript) {
      buffer += ch;
      index += 1;
      continue;
    }
    const close = markup.indexOf('}', index);
    if (buffer) segments.push({ text: buffer, kind: 'normal' });
    buffer = '';
    segments.push({ text: markup.slice(index + 2, close), kind: ch === '_' ? 'sub' : 'sup' });
    index = close + 1;
  }
  if (buffer) segments.push({ text: buffer, kind: 'normal' });
  return segments;
}

/**
 * @param {Element} parent @param {number} x @param {number} y @param {string} markup
 * @param {number} size @param {Attrs} attrs @returns {Element}
 */
function richText(parent, x, y, markup, size, attrs) {
  const text = el(parent, 'text', { x, y, 'font-size': size, ...attrs });
  const shifts = { normal: 0, sub: 0.3 * size, sup: -0.42 * size };
  let current = 0;
  for (const segment of parseMarkup(markup)) {
    const target = shifts[segment.kind];
    const fontSize = segment.kind === 'normal' ? size : size * 0.68;
    const span = el(text, 'tspan', { dy: target - current, 'font-size': fontSize });
    span.textContent = segment.text;
    current = target;
  }
  if (current !== 0) el(text, 'tspan', { dy: -current }).textContent = '​';
  return text;
}

/**
 * Screen-space arrow with a filled head.
 * @param {Element} parent @param {Point} from @param {Point} to @param {string} color
 * @param {ArrowStyle} style
 * @returns {void}
 */
function arrow(parent, from, to, color, style) {
  const width = style.width ?? 2.4;
  const head = style.head ?? 11;
  const length = Math.hypot(to.x - from.x, to.y - from.y);
  const ux = (to.x - from.x) / length;
  const uy = (to.y - from.y) / length;
  const base = { x: to.x - ux * head, y: to.y - uy * head };
  const g = el(parent, 'g', { opacity: style.opacity ?? 1 });
  el(g, 'line', {
    x1: from.x,
    y1: from.y,
    x2: base.x,
    y2: base.y,
    stroke: color,
    'stroke-width': width,
    'stroke-linecap': 'round',
    ...(style.dash ? { 'stroke-dasharray': style.dash } : {}),
  });
  const wing = head * 0.42;
  const left = { x: base.x - uy * wing, y: base.y + ux * wing };
  const right = { x: base.x + uy * wing, y: base.y - ux * wing };
  el(g, 'path', { d: pathOf([to, left, right], true), fill: color });
}

/** @param {number} count @returns {number[]} angles in radians over a full turn */
const turn = (count) => Array.from({ length: count + 1 }, (_, i) => (2 * Math.PI * i) / count);

/** @param {number} n @returns {string} a number as SVG path data writes it */
const num = (n) => String(n);

// ------------------------------------------------------------------ scene definition

const ARM_M = 1.0;
const ROTOR_R = 0.42;
const DISK_Z = -0.13;
const GROUND_D = 1.95;
/** PX4 quad-X order; azimuth measured from body x toward body y (clockwise seen from above). */
const MOTORS = /** @type {readonly Motor[]} */ ([
  { number: 1, azimuthDeg: 45, ccw: true, thrust: 0.78, bladeDeg: 20 },
  { number: 2, azimuthDeg: 225, ccw: true, thrust: 0.7, bladeDeg: 75 },
  { number: 3, azimuthDeg: 315, ccw: false, thrust: 0.8, bladeDeg: 140 },
  { number: 4, azimuthDeg: 135, ccw: false, thrust: 0.68, bladeDeg: 110 },
]);

/** World velocity direction at the current instant: north-east and climbing slightly. */
const V_DIR = unit(vec(Math.cos(40 * DEG), Math.sin(40 * DEG), -0.12));
/** Lateral direction used to bend the path into a gentle right turn. */
const V_LAT = vec(-Math.sin(40 * DEG), Math.cos(40 * DEG), 0);
const PATH_POINTS = [-2.2, -1.1, 0, 1.9, 4.0].map((t) =>
  add(mul(V_DIR, t), mul(V_LAT, 0.09 * t * t)),
);

/**
 * Centripetal-free uniform Catmull-Rom sampling through the control points.
 * @param {readonly Vec3[]} pts @param {number} perSegment @returns {Vec3[]}
 */
function catmullRom(pts, perSegment) {
  /** @type {Vec3[]} */
  const out = [];
  for (let i = 0; i < pts.length - 1; i += 1) {
    const p0 = pts[Math.max(i - 1, 0)] ?? vec(0, 0, 0);
    const p1 = pts[i] ?? vec(0, 0, 0);
    const p2 = pts[i + 1] ?? vec(0, 0, 0);
    const p3 = pts[Math.min(i + 2, pts.length - 1)] ?? vec(0, 0, 0);
    for (let k = 0; k < perSegment; k += 1) {
      const t = k / perSegment;
      const t2 = t * t;
      const t3 = t2 * t;
      const term = (/** @type {keyof Vec3} */ c) =>
        0.5 *
        (2 * p1[c] +
          (-p0[c] + p2[c]) * t +
          (2 * p0[c] - 5 * p1[c] + 4 * p2[c] - p3[c]) * t2 +
          (-p0[c] + 3 * p1[c] - 3 * p2[c] + p3[c]) * t3);
      out.push(vec(term('x'), term('y'), term('z')));
    }
  }
  out.push(pts[pts.length - 1] ?? vec(0, 0, 0));
  return out;
}

// ------------------------------------------------------------------ background

/** @param {Element} defs @returns {void} */
function defineBackdrop(defs) {
  const bg = el(defs, 'linearGradient', { id: 'bg', x1: 0, y1: 0, x2: 0, y2: 1 });
  el(bg, 'stop', { offset: 0, 'stop-color': COLOR.bgTop });
  el(bg, 'stop', { offset: 1, 'stop-color': COLOR.bgBottom });
  const glow = el(defs, 'radialGradient', { id: 'glow', cx: 0.52, cy: 0.46, r: 0.5 });
  el(glow, 'stop', { offset: 0, 'stop-color': 'rgba(70,130,255,0.22)' });
  el(glow, 'stop', { offset: 1, 'stop-color': 'rgba(70,130,255,0)' });
}

/** @param {Element} defs @returns {void} */
function defineGrid(defs) {
  const tile = { patternUnits: 'userSpaceOnUse' };
  const minor = el(defs, 'pattern', { id: 'gridMinor', width: 32, height: 32, ...tile });
  el(minor, 'path', { d: 'M32 0H0V32', fill: 'none', stroke: COLOR.grid, 'stroke-width': 1 });
  const major = el(defs, 'pattern', { id: 'gridMajor', width: 160, height: 160, ...tile });
  el(major, 'rect', { width: 160, height: 160, fill: 'url(#gridMinor)' });
  el(major, 'path', {
    d: 'M160 0H0V160',
    fill: 'none',
    stroke: COLOR.gridMajor,
    'stroke-width': 1,
  });
  const fade = el(defs, 'radialGradient', { id: 'fade', cx: 0.5, cy: 0.5, r: 0.75 });
  el(fade, 'stop', { offset: 0.35, 'stop-color': '#fff', 'stop-opacity': 1 });
  el(fade, 'stop', { offset: 1, 'stop-color': '#fff', 'stop-opacity': 0.15 });
  const mask = el(defs, 'mask', { id: 'gridMask' });
  el(mask, 'rect', { width: 1280, height: 640, fill: 'url(#fade)' });
}

/**
 * A full-frame mask whose opacity follows a linear gradient.
 * @param {Element} defs @param {string} gradientId @param {string} maskId
 * @param {Attrs} axis gradient end points @param {Stops} stops
 * @returns {void}
 */
function defineFadeMask(defs, gradientId, maskId, axis, stops) {
  const gradient = el(defs, 'linearGradient', { id: gradientId, ...axis });
  for (const [offset, alpha] of stops) {
    el(gradient, 'stop', { offset, 'stop-color': '#fff', 'stop-opacity': alpha });
  }
  const mask = el(defs, 'mask', { id: maskId });
  el(mask, 'rect', { width: 1280, height: 640, fill: `url(#${gradientId})` });
}

/** @param {Element} svg @returns {void} */
function drawBackground(svg) {
  const defs = el(svg, 'defs', {});
  defineBackdrop(defs);
  defineGrid(defs);
  const across = { x1: 0, y1: 0, x2: 1, y2: 0 };
  const down = { x1: 0, y1: 0, x2: 0, y2: 1 };
  /** @type {Stops} */
  const band = [
    [0, 0],
    [0.41, 0],
    [0.5, 1],
    [0.68, 1],
    [0.73, 0],
    [1, 0],
  ];
  defineFadeMask(defs, 'band', 'sceneMask', across, band);
  /** @type {Stops} */
  const lower = [
    [0, 1],
    [0.66, 1],
    [0.84, 0],
    [1, 0],
  ];
  defineFadeMask(defs, 'lower', 'lowerFade', down, lower);
  el(svg, 'rect', { width: 1280, height: 640, fill: 'url(#bg)' });
  el(svg, 'rect', { width: 1280, height: 640, fill: 'url(#gridMajor)', mask: 'url(#gridMask)' });
  el(svg, 'ellipse', { cx: 690, cy: 300, rx: 430, ry: 300, fill: 'url(#glow)' });
}

// ------------------------------------------------------------------ left column

const TITLE_LINES = /** @type {readonly TextLine[]} */ ([
  {
    x: 56,
    y: 92,
    fill: COLOR.dim,
    family: SANS,
    size: 14,
    weight: 600,
    spacing: 3.2,
    text: 'FLIGHT DYNAMICS ENGINE FOR sUAS',
  },
  {
    x: 52,
    y: 166,
    fill: COLOR.text,
    family: "'Inter Display', 'Inter', sans-serif",
    size: 78,
    weight: 800,
    spacing: -1.5,
    text: 'MJKPhysics',
  },
  {
    x: 56,
    y: 210,
    fill: '#d6e2fb',
    family: SANS,
    size: 23,
    weight: 500,
    text: 'Validated six-degree-of-freedom',
  },
  {
    x: 56,
    y: 240,
    fill: '#d6e2fb',
    family: SANS,
    size: 23,
    weight: 500,
    text: 'simulation of small unmanned aircraft',
  },
  {
    x: 56,
    y: 288,
    fill: COLOR.thrust,
    family: MONO,
    size: 17,
    weight: 500,
    spacing: 0.5,
    text: 'Rust  ·  WebAssembly  ·  WebGPU',
  },
  {
    x: 56,
    y: 316,
    fill: COLOR.dim,
    family: SANS,
    size: 14.5,
    weight: 500,
    text: 'Crazyflie 2.x  ·  Holybro X500 V2  ·  Aerosonde  ·  Skywalker X8',
  },
]);

/** @param {Element} svg @returns {void} */
function drawTitleBlock(svg) {
  for (const line of TITLE_LINES) textLine(svg, line);
}

const EQUATION_ROWS = Object.freeze([
  { label: 'TRANSLATION', eq: 'm v̇ = F_{w}', color: COLOR.weight },
  { label: 'ROTATION  ·  NEWTON–EULER', eq: 'I ω̇ = M_{b} − ω × (I ω)', color: COLOR.omega },
  { label: 'ATTITUDE  ·  HAMILTON QUATERNION', eq: 'q̇ = ½ q ⊗ (0, ω)', color: COLOR.yAxis },
  {
    label: 'ROTOR ELEMENT  ·  BEMT',
    eq: 'dT = ½ρW²c(C_{l}cosφ − C_{d}sinφ) B dr',
    color: COLOR.thrust,
  },
]);

/** @param {Element} svg @returns {void} */
function drawEquations(svg) {
  el(svg, 'rect', {
    x: 56,
    y: 338,
    width: 452,
    height: 196,
    rx: 12,
    fill: COLOR.panel,
    stroke: COLOR.panelStroke,
  });
  EQUATION_ROWS.forEach((row, i) => {
    const y = 370 + i * 44;
    el(svg, 'circle', { cx: 78, cy: y - 4, r: 3.5, fill: row.color });
    const caption = { x: 92, y, fill: COLOR.dim, family: SANS, size: 11.5, weight: 600 };
    textLine(svg, { ...caption, spacing: 1.6, text: row.label });
    richText(svg, 92, y + 22, row.eq, 18, { fill: COLOR.text, 'font-family': MONO });
  });
}

const CHIPS = Object.freeze([
  '6-DOF rigid body',
  'Lie-group integration',
  'BEMT rotors',
  'Dryden turbulence',
  'GJK/EPA + sequential impulses',
  'Bit-exact native ↔ WASM',
]);

/** @param {Element} svg @returns {void} */
function drawChips(svg) {
  let x = 56;
  for (const chip of CHIPS) {
    const g = el(svg, 'g', {});
    const text = textLine(g, {
      x: x + 14,
      y: 593,
      fill: '#dce6fb',
      family: SANS,
      size: 14.5,
      weight: 500,
      text: chip,
    });
    const width = /** @type {SVGTextElement} */ (text).getComputedTextLength() + 28;
    const rect = el(g, 'rect', {
      x,
      y: 570,
      width,
      height: 34,
      rx: 17,
      fill: 'rgba(94,225,255,0.06)',
      stroke: 'rgba(94,225,255,0.30)',
    });
    g.insertBefore(rect, text);
    x += width + 10;
  }
}

// ------------------------------------------------------------------ ground and path

/**
 * @param {Camera} cam @param {Vec3} origin @param {Mat3} R
 * @returns {ToScreen}
 */
const bodyProjector = (cam, origin, R) => (b) => project(cam, add(origin, rotate(R, b)));

/** @param {number} azimuthDeg @param {number} radius @param {number} z @returns {Vec3} */
const polar = (azimuthDeg, radius, z) =>
  vec(radius * Math.cos(azimuthDeg * DEG), radius * Math.sin(azimuthDeg * DEG), z);

/** @returns {(readonly [Vec3, Vec3])[]} north and east grid lines on the ground plane */
function groundLines() {
  /** @type {(readonly [Vec3, Vec3])[]} */
  const lines = [];
  for (let n = -5; n <= 7; n += 1) lines.push([vec(n, -5, GROUND_D), vec(n, 6, GROUND_D)]);
  for (let e = -5; e <= 6; e += 1) lines.push([vec(-5, e, GROUND_D), vec(7, e, GROUND_D)]);
  return lines;
}

/** @param {Element} svg @param {Camera} cam @returns {void} */
function drawGround(svg, cam) {
  const g = el(svg, 'g', {});
  const steps = 24;
  for (const [a, b] of groundLines()) {
    for (let i = 0; i < steps; i += 1) {
      const p = add(a, mul(sub(b, a), i / steps));
      const q = add(a, mul(sub(b, a), (i + 1) / steps));
      const mid = mul(add(p, q), 0.5);
      const fade = Math.max(0, 1 - Math.hypot(mid.x - 0.6, mid.y - 0.3) / 6.2);
      if (fade <= 0) continue;
      const pp = project(cam, p);
      const qq = project(cam, q);
      const stroke = `rgba(126,162,255,${(0.22 * fade).toFixed(3)})`;
      el(g, 'line', { x1: pp.x, y1: pp.y, x2: qq.x, y2: qq.y, stroke, 'stroke-width': 1 });
    }
  }
}

/** @param {Point} s @returns {string} a waypoint diamond centered on `s` */
const diamondPath = (s) =>
  `M${num(s.x)} ${num(s.y - 7)} L${num(s.x + 7)} ${num(s.y)} ` +
  `L${num(s.x)} ${num(s.y + 7)} L${num(s.x - 7)} ${num(s.y)} Z`;

/** @param {Element} svg @param {Camera} cam @returns {void} */
function drawTrajectory(svg, cam) {
  const samples = catmullRom(PATH_POINTS, 40).map((p) => project(cam, p));
  const split = 2 * 40;
  const stroke = { fill: 'none', 'stroke-width': 2 };
  const flown = pathOf(samples.slice(0, split + 1), false);
  el(svg, 'path', { d: flown, ...stroke, stroke: 'rgba(159,182,230,0.55)' });
  const planned = pathOf(samples.slice(split), false);
  el(svg, 'path', {
    d: planned,
    ...stroke,
    stroke: 'rgba(159,182,230,0.75)',
    'stroke-dasharray': '7 7',
  });
  PATH_POINTS.forEach((p, i) => {
    if (i === 2) return;
    el(svg, 'path', {
      d: diamondPath(project(cam, p)),
      fill: COLOR.bgBottom,
      stroke: 'rgba(200,215,245,0.85)',
      'stroke-width': 1.6,
    });
  });
}

/** @param {Element} svg @param {Camera} cam @returns {void} */
function drawWorldTriad(svg, cam) {
  const origin = vec(-0.5, 0.95, GROUND_D);
  const o = project(cam, origin);
  const axes = [
    { d: vec(0.55, 0, 0), name: 'N' },
    { d: vec(0, 0.55, 0), name: 'E' },
    { d: vec(0, 0, 0.55), name: 'D' },
  ];
  for (const axis of axes) {
    const tip = project(cam, add(origin, axis.d));
    arrow(svg, o, tip, COLOR.faint, { width: 1.8, head: 9 });
    const at = { x: tip.x + 6, y: tip.y + 5 };
    textLine(svg, { ...at, fill: COLOR.dim, family: MONO, size: 14, text: axis.name });
  }
  textLine(svg, {
    x: o.x - 96,
    y: o.y + 4,
    fill: COLOR.faint,
    family: SANS,
    size: 11.5,
    weight: 600,
    spacing: 1.4,
    text: 'WORLD  NED',
  });
}

// ------------------------------------------------------------------ quadrotor airframe

/** @param {Element} svg @param {ToScreen} toScreen @param {Motor} motor @returns {void} */
function drawArm(svg, toScreen, motor) {
  const tip = polar(motor.azimuthDeg, ARM_M, 0);
  const side = polar(motor.azimuthDeg + 90, 0.045, 0);
  const corners = [
    add(vec(0, 0, 0), side),
    add(tip, side),
    sub(tip, side),
    sub(vec(0, 0, 0), side),
  ];
  el(svg, 'path', {
    d: pathOf(corners.map(toScreen), true),
    fill: COLOR.frame,
    stroke: '#9fb6e6',
    'stroke-width': 1.3,
  });
}

/** @param {Element} svg @param {ToScreen} toScreen @param {Motor} motor @returns {void} */
function drawMotor(svg, toScreen, motor) {
  const center = polar(motor.azimuthDeg, ARM_M, 0);
  /** @param {number} r @param {number} z @returns {Projected[]} */
  const ring = (r, z) =>
    turn(36).map((a) => toScreen(add(center, vec(r * Math.cos(a), r * Math.sin(a), z))));
  const outline = { stroke: COLOR.line };
  const can = pathOf(ring(0.095, -0.02), true);
  el(svg, 'path', { d: can, fill: COLOR.frameTop, ...outline, 'stroke-width': 1.3 });
  const bell = pathOf(ring(0.05, -0.08), true);
  el(svg, 'path', { d: bell, fill: '#26407a', ...outline, 'stroke-width': 1 });
}

/** @param {Element} svg @param {ToScreen} toScreen @returns {void} */
function drawHub(svg, toScreen) {
  /** @param {number} h @param {number} z @returns {Projected[]} */
  const square = (h, z) =>
    [vec(h, h, z), vec(h, -h, z), vec(-h, -h, z), vec(-h, h, z)].map(toScreen);
  const plate = pathOf(square(0.23, 0.02), true);
  el(svg, 'path', { d: plate, fill: COLOR.frame, stroke: COLOR.line, 'stroke-width': 1.5 });
  const lid = pathOf(square(0.15, -0.07), true);
  el(svg, 'path', { d: lid, fill: COLOR.frameTop, stroke: COLOR.line, 'stroke-width': 1.2 });
  const nose = [vec(0.13, 0, -0.075), vec(0.02, -0.06, -0.075), vec(0.02, 0.06, -0.075)];
  el(svg, 'path', { d: pathOf(nose.map(toScreen), true), fill: COLOR.xAxis, opacity: 0.9 });
}

/** @param {Element} svg @param {ToScreen} toScreen @param {Motor} motor @returns {void} */
function drawRotor(svg, toScreen, motor) {
  const center = polar(motor.azimuthDeg, ARM_M, DISK_Z);
  /** @param {number} r @returns {Projected[]} */
  const circle = (r) =>
    turn(72).map((a) => toScreen(add(center, vec(r * Math.cos(a), r * Math.sin(a), 0))));
  el(svg, 'path', {
    d: pathOf(circle(ROTOR_R), true),
    fill: 'rgba(94,225,255,0.075)',
    stroke: 'rgba(94,225,255,0.65)',
    'stroke-width': 1.5,
  });
  for (const f of [0.3, 0.55, 0.8]) {
    el(svg, 'path', {
      d: pathOf(circle(ROTOR_R * f), true),
      fill: 'none',
      stroke: 'rgba(94,225,255,0.22)',
      'stroke-width': 1,
      'stroke-dasharray': '3 5',
    });
  }
  for (const offset of [0, 180]) drawBlade(svg, toScreen, center, motor.bladeDeg + offset);
  drawSpinArrow(svg, toScreen, center, motor);
}

/**
 * Tapered blade outline lofted from a linear chord distribution.
 * @param {Element} svg @param {ToScreen} toScreen @param {Vec3} center @param {number} azimuthDeg
 * @returns {void}
 */
function drawBlade(svg, toScreen, center, azimuthDeg) {
  const stations = [0.12, 0.35, 0.6, 0.85, 1.0];
  /** @param {number} f @returns {number} */
  const chordAt = (f) => 0.075 - 0.04 * f;
  /** @param {number} f @param {number} side @returns {Vec3} */
  const edge = (f, side) =>
    add(
      add(center, polar(azimuthDeg, ROTOR_R * f, 0)),
      polar(azimuthDeg + side, chordAt(f) * 0.5, 0),
    );
  const lead = stations.map((f) => edge(f, -90));
  const trail = stations.map((f) => edge(f, 90));
  const outline = [...lead, ...trail.reverse()].map(toScreen);
  el(svg, 'path', {
    d: pathOf(outline, true),
    fill: 'rgba(207,224,255,0.28)',
    stroke: 'rgba(207,224,255,0.55)',
    'stroke-width': 0.8,
  });
}

/**
 * @param {Element} svg @param {ToScreen} toScreen @param {Vec3} center @param {Motor} motor
 * @returns {void}
 */
function drawSpinArrow(svg, toScreen, center, motor) {
  const direction = motor.ccw ? -1 : 1;
  const start = motor.azimuthDeg - direction * 40;
  const angles = Array.from({ length: 17 }, (_, i) => start + direction * i * 5);
  const pts = angles.map((a) => toScreen(add(center, polar(a, ROTOR_R * 1.13, 0))));
  el(svg, 'path', {
    d: pathOf(pts, false),
    fill: 'none',
    stroke: COLOR.thrust,
    'stroke-width': 1.8,
    opacity: 0.85,
  });
  const last = pts[pts.length - 1];
  const prev = pts[pts.length - 3];
  if (last && prev) arrow(svg, prev, last, COLOR.thrust, { width: 1.8, head: 9, opacity: 0.85 });
}

/** @param {Element} svg @param {ToScreen} toScreen @returns {void} */
function drawAirframe(svg, toScreen) {
  /** @param {Motor} m @returns {number} */
  const byDepth = (m) => -toScreen(polar(m.azimuthDeg, ARM_M, 0)).depth;
  const ordered = [...MOTORS].sort((a, b) => byDepth(a) - byDepth(b));
  for (const m of ordered) drawArm(svg, toScreen, m);
  drawHub(svg, toScreen);
  for (const m of ordered) drawMotor(svg, toScreen, m);
  for (const m of ordered) drawRotor(svg, toScreen, m);
}

// ------------------------------------------------------------------ quadrotor vectors

/**
 * @param {Element} svg @param {Point} anchor @param {string} markup @param {string} color
 * @param {Point} offset @param {number} [size]
 * @returns {void}
 */
function label(svg, anchor, markup, color, offset, size = 17) {
  const attrs = { fill: color, 'font-family': MONO, 'font-weight': 600 };
  richText(svg, anchor.x + offset.x, anchor.y + offset.y, markup, size, attrs);
}

/** @param {Element} svg @param {Camera} cam @param {Rig} rig @returns {void} */
function drawAltitude(svg, cam, rig) {
  const ground = project(cam, vec(0, 0, GROUND_D));
  const faint = { stroke: COLOR.faint, 'stroke-width': 1.2 };
  const drop = { x1: rig.cg.x, y1: rig.cg.y, x2: ground.x, y2: ground.y };
  el(svg, 'line', { ...drop, ...faint, 'stroke-dasharray': '2 6' });
  el(svg, 'ellipse', { cx: ground.x, cy: ground.y, rx: 16, ry: 5, fill: 'none', ...faint });
  label(svg, project(cam, vec(0, 0, 1.6)), 'h', COLOR.dim, { x: 10, y: 6 }, 16);
}

/** @param {Element} svg @param {Rig} rig @returns {void} */
function drawWeightAndBodyZ(svg, rig) {
  const gTip = rig.world(vec(0, 0, 1.15));
  arrow(svg, rig.cg, gTip, COLOR.weight, { width: 2.8, head: 13 });
  label(svg, gTip, 'mg', COLOR.weight, { x: -40, y: 8 });
  const zTip = rig.toScreen(vec(0, 0, 0.95));
  arrow(svg, rig.cg, zTip, COLOR.zAxis, { width: 2.2, head: 11 });
  label(svg, zTip, 'z_{b}', COLOR.zAxis, { x: 9, y: 10 });
}

/** @param {Element} svg @param {ToScreen} toScreen @returns {void} */
function drawThrusts(svg, toScreen) {
  for (const m of MOTORS) {
    const base = toScreen(polar(m.azimuthDeg, ARM_M, DISK_Z - 0.09));
    const tip = toScreen(polar(m.azimuthDeg, ARM_M, DISK_Z - 0.09 - 0.85 * m.thrust));
    arrow(svg, base, tip, COLOR.thrust, { width: 2.6, head: 12 });
    label(svg, tip, `T_{${num(m.number)}}`, COLOR.thrust, { x: 8, y: 4 });
    const tag = toScreen(polar(m.azimuthDeg, ARM_M + ROTOR_R * 1.38, DISK_Z));
    const at = { x: tag.x - 5, y: tag.y + 5 };
    const style = { fill: COLOR.dim, family: MONO, size: 13, weight: 600 };
    textLine(svg, { ...at, ...style, text: String(m.number) });
  }
}

/** @param {Element} svg @param {Rig} rig @returns {void} */
function drawBodyXY(svg, rig) {
  const xTip = rig.toScreen(vec(1.25, 0, 0));
  arrow(svg, rig.cg, xTip, COLOR.xAxis, { width: 2.4, head: 12 });
  label(svg, xTip, 'x_{b}', COLOR.xAxis, { x: 8, y: -6 });
  const yTip = rig.toScreen(vec(0, 1.2, 0));
  arrow(svg, rig.cg, yTip, COLOR.yAxis, { width: 2.4, head: 12 });
  label(svg, yTip, 'y_{b}', COLOR.yAxis, { x: -10, y: 26 });
}

/** @param {Element} svg @param {ToScreen} toScreen @returns {void} */
function drawYawRate(svg, toScreen) {
  const arc = Array.from({ length: 31 }, (_, i) => toScreen(polar(200 + i * 10, 0.17, 0.52)));
  const d = pathOf(arc.slice(0, -2), false);
  el(svg, 'path', { d, fill: 'none', stroke: COLOR.omega, 'stroke-width': 2.2 });
  const o1 = arc[arc.length - 3];
  const o2 = arc[arc.length - 1];
  if (o1 && o2) arrow(svg, o1, o2, COLOR.omega, { width: 2.2, head: 11 });
  label(svg, toScreen(polar(20, 0.17, 0.52)), 'r', COLOR.omega, { x: 12, y: 6 }, 18);
}

/** @param {Element} svg @param {Rig} rig @returns {void} */
function drawVelocityAndDrag(svg, rig) {
  const vTip = rig.world(mul(V_DIR, 1.35));
  arrow(svg, rig.cg, vTip, COLOR.vel, { width: 2, head: 11, dash: '6 5' });
  label(svg, vTip, 'v', COLOR.vel, { x: 10, y: 10 });
  const dTip = rig.world(mul(V_DIR, -0.85));
  arrow(svg, rig.cg, dTip, COLOR.aero, { width: 2.6, head: 12 });
  label(svg, dTip, 'F_{drag}', COLOR.aero, { x: -62, y: 18 });
}

/** @param {Element} svg @param {Point} cg @returns {void} */
function drawCenterOfMass(svg, cg) {
  const style = { fill: COLOR.bgTop, stroke: COLOR.text, 'stroke-width': 1.6 };
  el(svg, 'circle', { cx: cg.x, cy: cg.y, r: 5, ...style });
  const [x, y] = [cg.x, cg.y];
  const d =
    `M${num(x - 5)} ${num(y)} A5 5 0 0 1 ${num(x)} ${num(y - 5)} L${num(x)} ${num(y)} Z ` +
    `M${num(x + 5)} ${num(y)} A5 5 0 0 1 ${num(x)} ${num(y + 5)} L${num(x)} ${num(y)} Z`;
  el(svg, 'path', { d, fill: COLOR.text });
}

/** @param {Element} svg @param {Camera} cam @returns {void} */
function drawQuadrotor(svg, cam) {
  // Pitched nose-down and rolled right: thrust is tilted toward the north-east velocity.
  const R = bodyToWorld(0, -12 * DEG, 10 * DEG);
  const origin = vec(0, 0, 0);
  /** @type {Rig} */
  const rig = {
    cg: project(cam, origin),
    toScreen: bodyProjector(cam, origin, R),
    world: (w) => project(cam, add(origin, w)),
  };
  drawAltitude(svg, cam, rig);
  drawWeightAndBodyZ(svg, rig);
  drawAirframe(svg, rig.toScreen);
  drawThrusts(svg, rig.toScreen);
  drawBodyXY(svg, rig);
  drawYawRate(svg, rig.toScreen);
  drawVelocityAndDrag(svg, rig);
  drawCenterOfMass(svg, rig.cg);
}

// ------------------------------------------------------------------ right panels

/**
 * @param {Element} svg @param {number} x @param {number} y @param {number} w @param {number} h
 * @param {string} title
 * @returns {void}
 */
function panel(svg, x, y, w, h, title) {
  const box = { x, y, width: w, height: h, rx: 12 };
  el(svg, 'rect', { ...box, fill: COLOR.panel, stroke: COLOR.panelStroke });
  const heading = { x: x + 18, y: y + 28, fill: COLOR.dim, family: SANS, size: 11.5, weight: 600 };
  textLine(svg, { ...heading, spacing: 1.6, text: title });
}

/** Flying-wing panel geometry, in screen pixels. */
const WING = Object.freeze({
  cx: 1081,
  half: 118,
  base: 176,
  le: 190,
  sweep: 44,
  rootChord: 58,
  tipChord: 26,
});

/** @param {number} eta spanwise station, −1 to 1 @returns {number} lift bar height */
const liftHeight = (eta) => 66 * Math.sqrt(Math.max(0, 1 - eta * eta)) * (1 - 0.12 * eta * eta);

/** @param {Element} svg @returns {void} */
function drawLiftDistribution(svg) {
  const { cx, half, base } = WING;
  const curve = Array.from({ length: 61 }, (_, i) => {
    const eta = -1 + i / 30;
    return { x: cx + eta * half, y: base - liftHeight(eta) };
  });
  const area = [...curve, { x: cx + half, y: base }, { x: cx - half, y: base }];
  el(svg, 'path', { d: pathOf(area, true), fill: 'rgba(255,107,181,0.12)' });
  el(svg, 'path', {
    d: pathOf(curve, false),
    fill: 'none',
    stroke: COLOR.aero,
    'stroke-width': 2.2,
  });
  for (let i = -5; i <= 5; i += 1) {
    const x = cx + (i / 6) * half;
    const y = base - liftHeight(i / 6);
    const bar = { x1: x, y1: base, x2: x, y2: y + 3 };
    el(svg, 'line', { ...bar, stroke: 'rgba(255,107,181,0.45)', 'stroke-width': 1 });
  }
  const axis = { x1: cx - half - 6, y1: base, x2: cx + half + 6, y2: base };
  el(svg, 'line', { ...axis, stroke: COLOR.faint, 'stroke-width': 1 });
  label(svg, { x: cx + 44, y: 112 }, "L'(y)", COLOR.aero, { x: 0, y: 0 }, 15);
}

/** @param {number} s −1 for the left wing, 1 for the right @returns {Point[]} */
function elevon(s) {
  const { cx, half, le, sweep, rootChord, tipChord } = WING;
  return [
    { x: cx + s * 40, y: le + rootChord - 2 },
    { x: cx + s * (half - 8), y: le + sweep + tipChord - 2 },
    { x: cx + s * (half - 8), y: le + sweep + tipChord - 12 },
    { x: cx + s * 40, y: le + rootChord - 14 },
  ];
}

/** @param {Element} svg @returns {void} */
function drawWingPlanform(svg) {
  const { cx, half, le, sweep, rootChord, tipChord } = WING;
  const wing = [
    { x: cx, y: le },
    { x: cx + half, y: le + sweep },
    { x: cx + half, y: le + sweep + tipChord },
    { x: cx + 18, y: le + rootChord + 6 },
    { x: cx, y: le + rootChord + 10 },
    { x: cx - 18, y: le + rootChord + 6 },
    { x: cx - half, y: le + sweep + tipChord },
    { x: cx - half, y: le + sweep },
  ];
  el(svg, 'path', {
    d: pathOf(wing, true),
    fill: COLOR.frame,
    stroke: COLOR.line,
    'stroke-width': 1.5,
  });
  for (const s of [-1, 1]) {
    const surface = { fill: 'rgba(255,107,181,0.25)', stroke: 'rgba(255,107,181,0.7)' };
    el(svg, 'path', { d: pathOf(elevon(s), true), ...surface, 'stroke-width': 1 });
    const tip = {
      x1: cx + s * half,
      y1: le + sweep - 4,
      x2: cx + s * half,
      y2: le + sweep + tipChord + 4,
    };
    el(svg, 'line', { ...tip, stroke: COLOR.line, 'stroke-width': 3, 'stroke-linecap': 'round' });
  }
}

/** @param {Element} svg @returns {void} */
function drawSpanDimension(svg) {
  const { cx, half } = WING;
  const dimY = 284;
  const style = { width: 1.2, head: 8 };
  arrow(svg, { x: cx, y: dimY }, { x: cx + half, y: dimY }, COLOR.faint, style);
  arrow(svg, { x: cx, y: dimY }, { x: cx - half, y: dimY }, COLOR.faint, style);
  el(svg, 'rect', { x: cx - 12, y: dimY - 10, width: 24, height: 20, fill: COLOR.bgBottom });
  label(svg, { x: cx - 5, y: dimY + 5 }, 'b', COLOR.dim, { x: 0, y: 0 }, 14);
}

/** @param {Element} svg @returns {void} */
function drawSpanwisePanel(svg) {
  panel(svg, 938, 56, 286, 248, 'FLYING WING  ·  SPANWISE LIFT');
  drawLiftDistribution(svg);
  drawWingPlanform(svg);
  drawSpanDimension(svg);
}

/** Convergence plot area, in screen pixels. */
const PLOT = Object.freeze({ x0: 1000, x1: 1200, y0: 356, y1: 500 });
/** @param {number} log10dt @returns {number} */
const plotX = (log10dt) => PLOT.x0 + ((log10dt + 4) / 2) * (PLOT.x1 - PLOT.x0);
/** @param {number} log10e @returns {number} */
const plotY = (log10e) => PLOT.y0 + ((0 - log10e) / 12) * (PLOT.y1 - PLOT.y0);

/** @param {Element} svg @returns {void} */
function drawConvergenceAxes(svg) {
  const { x0, x1, y0, y1 } = PLOT;
  const frame = `M${num(x0)} ${num(y0)} V${num(y1)} H${num(x1)}`;
  el(svg, 'path', { d: frame, fill: 'none', stroke: COLOR.faint, 'stroke-width': 1.2 });
  const tickText = { fill: COLOR.dim, 'font-family': MONO };
  for (const [d, text] of /** @type {const} */ ([
    [-4, '−4'],
    [-3, '−3'],
    [-2, '−2'],
  ])) {
    el(svg, 'line', { x1: plotX(d), y1: y1, x2: plotX(d), y2: y1 + 4, stroke: COLOR.faint });
    richText(svg, plotX(d) - 14, y1 + 20, `10^{${text}}`, 13, tickText);
  }
  for (const [e, text] of /** @type {const} */ ([
    [0, '0'],
    [-6, '−6'],
    [-12, '−12'],
  ])) {
    const y = plotY(e);
    el(svg, 'line', { x1: x0 - 4, y1: y, x2: x0, y2: y, stroke: COLOR.faint });
    richText(svg, x0 - 44, y + 5, `10^{${text}}`, 13, tickText);
  }
  label(svg, { x: x1 - 18, y: y1 - 8 }, 'Δt', COLOR.dim, { x: 0, y: 0 }, 13);
}

/** @param {Element} svg @returns {void} */
function drawConvergenceLines(svg) {
  const lines = [
    { slope: 1, at: -1, color: COLOR.weight, tag: 'O(Δt)', dy: -12 },
    { slope: 4, at: -3, color: COLOR.thrust, tag: 'O(Δt^{4})', dy: 24 },
  ];
  for (const ln of lines) {
    /** @param {number} d @returns {number} */
    const e = (d) => ln.at + ln.slope * (d + 2);
    const ends = { x1: plotX(-4), y1: plotY(e(-4)), x2: plotX(-2), y2: plotY(e(-2)) };
    el(svg, 'line', { ...ends, stroke: ln.color, 'stroke-width': 2.4 });
    const attrs = { fill: ln.color, 'font-family': MONO, 'font-weight': 600 };
    richText(svg, plotX(-2.55), plotY(e(-2.55)) + ln.dy, ln.tag, 14, attrs);
  }
}

/** @param {Element} svg @returns {void} */
function drawConvergencePanel(svg) {
  panel(svg, 938, 320, 286, 214, 'INTEGRATOR CONVERGENCE ORDER');
  drawConvergenceAxes(svg);
  drawConvergenceLines(svg);
}

// ------------------------------------------------------------------ compose

/** @returns {void} */
function draw() {
  const svg = document.getElementById('hero');
  if (!svg) throw new Error('missing #hero svg');
  const cam = makeCamera(vec(-4.53, -0.8, -4.2), vec(0.3, 0.05, 0.5), 860, 738, 250);
  drawBackground(svg);
  const scene = el(el(svg, 'g', { mask: 'url(#sceneMask)' }), 'g', { mask: 'url(#lowerFade)' });
  drawGround(scene, cam);
  drawTrajectory(scene, cam);
  drawWorldTriad(svg, cam);
  drawQuadrotor(svg, cam);
  drawTitleBlock(svg);
  drawEquations(svg);
  drawSpanwisePanel(svg);
  drawConvergencePanel(svg);
  drawChips(svg);
  svg.setAttribute('data-ready', 'true');
}

try {
  await document.fonts.ready;
  draw();
} catch (error) {
  document.body.textContent = `render failed: ${error instanceof Error ? error.message : String(error)}`;
}
