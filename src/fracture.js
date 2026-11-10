// PIECES (milestone 24). fracture(shape, options) cuts a box or a hull into
// convex pieces that fill it exactly: Voronoi cells, the part of the shape
// nearer one of `count` points scattered through it than to any other. Each
// cell is the shape cut by the plane halfway between its point and each
// other's, so the pieces meet face to face, with nothing between them and
// nothing overlapping. When to break, and what to do with the pieces, is the
// game's: remove the body and add the pieces where it was.
//
// The points come from a seeded generator of exact whole-number steps, and
// every cut is exact operations and Bearing's own atan2, so a seed gives the
// same pieces on every machine, as the step does.

import { fail, show, options, known, oneOf, positive3 } from './core/check.js';
import { atan2 } from './core/math.js';
import { build } from './hull.js';

const SHAPE_OPTIONS = ['shape', 'size', 'vertices'];
const OPTIONS = ['count', 'seed'];

/**
 * How many pieces, left out: eight, as many as halving a box each way makes,
 * and few enough that the pieces stay big against the whole.
 */
const COUNT = 8;

/**
 * How near two points may be: a quarter of the spacing `count` points would
 * have spread evenly through the shape's volume. Nearer, they make a cell a
 * sliver between them, a piece too thin to be a solid, or so thin it is a
 * thousand tiny contacts; random points are near each other more often than
 * one would think.
 */
const APART = 0.25;

/** The pieces of `shape`, a box or a hull: [{ vertices, position }, ...]. */
export function fracture(shape = {}, given = {}) {
  const caller = 'fracture';
  const s = options(caller, shape, 'shape');
  known(`${caller}: shape`, s, SHAPE_OPTIONS);
  const kind = s.shape === undefined ? 'box' : oneOf(caller, 'shape.shape', s.shape, ['box', 'hull']);
  if (kind === 'box' && s.vertices !== undefined) fail(caller, 'a box has no vertices');
  if (kind === 'hull' && s.size !== undefined) fail(caller, 'a hull has no size');
  const o = options(caller, given);
  known(caller, o, OPTIONS);
  const count = o.count === undefined ? COUNT : o.count;
  if (!Number.isSafeInteger(count) || count < 1) fail(caller, `count must be a whole number, 1 or more, got ${show(count)}`);
  const seed = o.seed === undefined ? 0 : o.seed;
  if (!Number.isSafeInteger(seed)) fail(caller, `seed must be a whole number, got ${show(seed)}`);

  let corners;
  if (kind === 'box') {
    const [x, y, z] = (s.size === undefined ? [1, 1, 1] : positive3(caller, 'shape.size', s.size)).map((v) => v / 2);
    corners = [-x, -y, -z, x, -y, -z, -x, y, -z, x, y, -z, -x, -y, z, x, -y, z, -x, y, z, x, y, z];
  } else {
    corners = s.vertices === undefined ? [-0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5, -0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, 0.5, 0.5] : vertices(caller, s.vertices);
  }
  const hull = build(corners);
  if (hull === null) fail(caller, `shape.vertices must span a solid (4 points not in one plane), got ${corners.length / 3 < 4 ? `${corners.length / 3} points` : `${corners.length / 3} points in one plane`}`);

  const whole = facesOf(hull);
  const { volume } = solid(whole);
  const points = scatter(hull, volume, count, seed);
  const pieces = [];
  for (let i = 0; i < count; i++) {
    let faces = whole;
    const [px, py, pz] = points[i];
    // Nearest first: the near cuts shrink the cell, and most far ones then miss it.
    const order = points.map((q, j) => [j, (q[0] - px) * (q[0] - px) + (q[1] - py) * (q[1] - py) + (q[2] - pz) * (q[2] - pz)])
      .filter(([j]) => j !== i).sort((a, b) => a[1] - b[1] || a[0] - b[0]);
    for (const [j] of order) {
      const [qx, qy, qz] = points[j];
      // Nearer p than q: (q - p) · x <= (|q|² - |p|²) / 2.
      const nx = qx - px, ny = qy - py, nz = qz - pz;
      faces = cut(faces, nx, ny, nz, ((qx * qx + qy * qy + qz * qz) - (px * px + py * py + pz * pz)) / 2);
    }
    const { center } = solid(faces);
    const seen = new Set(), out = [];
    for (const face of faces) {
      for (const v of face) {
        const key = `${v[0]},${v[1]},${v[2]}`;
        if (seen.has(key)) continue;
        seen.add(key);
        out.push(v[0] - center[0], v[1] - center[1], v[2] - center[2]);
      }
    }
    pieces.push({ vertices: out, position: center });
  }
  return pieces;
}

/** Hull vertices as given: finite numbers, three to a vertex. */
function vertices(caller, value) {
  const rule = 'shape.vertices must be finite numbers, three to a vertex';
  if (!(Array.isArray(value) || ArrayBuffer.isView(value))) fail(caller, `${rule}, got ${show(value)}`);
  if (value.length % 3 !== 0) fail(caller, `${rule}, got ${value.length} numbers`);
  for (let i = 0; i < value.length; i++) {
    if (typeof value[i] !== 'number' || !Number.isFinite(value[i])) fail(caller, `${rule}, got ${show(value[i])} at index ${i}`);
  }
  return Array.from(value);
}

/** A hull's faces, each its corners counter-clockwise seen from outside. */
function facesOf(hull) {
  const v = hull.vertices, faces = [];
  for (let f = 0; f + 1 < hull.start.length; f++) {
    const face = [];
    for (let i = hull.start[f]; i < hull.start[f + 1]; i++) {
      const c = 3 * hull.loops[i];
      face.push([v[c], v[c + 1], v[c + 2]]);
    }
    faces.push(face);
  }
  return faces;
}

/**
 * `count` points in the hull, from `seed`: Park and Miller's generator (state
 * times 16807, modulo 2^31 - 1, exact in a double), each point tried at random
 * in the hull's box, kept if it is inside and APART from those kept. Should the
 * shape be too thin to keep them apart, after a thousand tries a point the
 * rule is dropped, and they need only be inside.
 */
function scatter(hull, volume, count, seed) {
  const M = 2147483647;
  let state = 1 + (((seed % (M - 1)) + (M - 1)) % (M - 1));
  const next = () => (state = (state * 16807) % M) / M;
  const lo = hull.lo, hi = hull.hi, planes = hull.planes;
  // Apart by APART of the spacing (volume / count)^(1/3): as sixth powers, d⁶ against
  // APART⁶ (volume / count)², which needs no cube root (not the same on every machine).
  const share = volume / count, apart6 = APART * APART * APART * APART * APART * APART * share * share;
  const points = [];
  let tries = 0;
  while (points.length < count) {
    const p = [lo[0] + (hi[0] - lo[0]) * next(), lo[1] + (hi[1] - lo[1]) * next(), lo[2] + (hi[2] - lo[2]) * next()];
    tries++;
    let inside = true;
    for (let f = 0; f < planes.length && inside; f += 4) inside = planes[f] * p[0] + planes[f + 1] * p[1] + planes[f + 2] * p[2] < planes[f + 3];
    if (!inside) continue;
    if (tries <= 1000 * count && points.some((q) => {
      const x = q[0] - p[0], y = q[1] - p[1], z = q[2] - p[2], d2 = x * x + y * y + z * z;
      return d2 * d2 * d2 < apart6;
    })) continue;
    points.push(p);
  }
  return points;
}

/**
 * The convex solid `faces`, cut by the plane n · x = d, keeping n · x <= d: each
 * face's part on that side, and the cap the plane closes it with. A corner on
 * an edge that crosses is worked out from the edge's end that is kept, so the
 * two faces either side find the same corner, to the bit.
 */
function cut(faces, nx, ny, nz, d) {
  let any = false;
  for (const face of faces) {
    for (const v of face) if (nx * v[0] + ny * v[1] + nz * v[2] - d > 0) { any = true; break; }
    if (any) break;
  }
  if (!any) return faces;
  const out = [], cap = [], seen = new Set();
  const add = (p) => {
    const key = `${p[0]},${p[1]},${p[2]}`;
    if (!seen.has(key)) { seen.add(key); cap.push(p); }
  };
  for (const face of faces) {
    const kept = [];
    for (let i = 0; i < face.length; i++) {
      const a = face[i], b = face[(i + 1) % face.length];
      const ha = nx * a[0] + ny * a[1] + nz * a[2] - d, hb = nx * b[0] + ny * b[1] + nz * b[2] - d;
      if (ha <= 0) {
        kept.push(a);
        if (ha === 0) add(a);
      }
      if ((ha <= 0) !== (hb <= 0)) {
        const [p, q, hp, hq] = ha <= 0 ? [a, b, ha, hb] : [b, a, hb, ha];
        const t = hp / (hp - hq);
        const x = [p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]), p[2] + t * (q[2] - p[2])];
        kept.push(x);
        add(x);
      }
    }
    if (kept.length >= 3) out.push(kept);
  }
  if (cap.length >= 3) {
    // Round the cap's mean, counter-clockwise seen from outside, along n.
    const l = Math.sqrt(nx * nx + ny * ny + nz * nz), ux0 = nx / l, uy0 = ny / l, uz0 = nz / l;
    const m = [0, 0, 0];
    for (const p of cap) { m[0] += p[0]; m[1] += p[1]; m[2] += p[2]; }
    m[0] /= cap.length; m[1] /= cap.length; m[2] /= cap.length;
    // A basis in the plane: u square to n, v = n × u.
    let ux, uy, uz;
    if (Math.abs(ux0) <= Math.abs(uy0) && Math.abs(ux0) <= Math.abs(uz0)) { ux = 0; uy = -uz0; uz = uy0; }
    else if (Math.abs(uy0) <= Math.abs(uz0)) { ux = uz0; uy = 0; uz = -ux0; }
    else { ux = -uy0; uy = ux0; uz = 0; }
    const vx = uy0 * uz - uz0 * uy, vy = uz0 * ux - ux0 * uz, vz = ux0 * uy - uy0 * ux;
    const angle = (p) => atan2((p[0] - m[0]) * vx + (p[1] - m[1]) * vy + (p[2] - m[2]) * vz, (p[0] - m[0]) * ux + (p[1] - m[1]) * uy + (p[2] - m[2]) * uz);
    out.push(cap.map((p) => [angle(p), p]).sort((a, b) => a[0] - b[0]).map((e) => e[1]));
  }
  return out;
}

/** A convex solid's volume and centre, from tetrahedra fanned from its first corner (fluid.js's way). */
function solid(faces) {
  const o = faces[0][0];
  let volume = 0, mx = 0, my = 0, mz = 0;
  for (const face of faces) {
    const a = face[0], ax = a[0] - o[0], ay = a[1] - o[1], az = a[2] - o[2];
    for (let k = 1; k + 1 < face.length; k++) {
      const b = face[k], c = face[k + 1];
      const bx = b[0] - o[0], by = b[1] - o[1], bz = b[2] - o[2], cx = c[0] - o[0], cy = c[1] - o[1], cz = c[2] - o[2];
      const six = ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx);
      volume += six;
      mx += six * (ax + bx + cx); my += six * (ay + by + cy); mz += six * (az + bz + cz);
    }
  }
  return { volume: volume / 6, center: [o[0] + mx / (4 * volume), o[1] + my / (4 * volume), o[2] + mz / (4 * volume)] };
}
