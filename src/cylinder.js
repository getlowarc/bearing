// CYLINDERS (milestone 26): a cylinder against any other convex shape.
//
// A cylinder is round one way and flat the other, and has no finite list of
// faces and edges to test, as a box or a hull has, so it isn't written pair by
// pair. Every pair with one goes the same way, in two parts:
//
// WHICH WAY. Apart, GJK's closest points (gjk.js) give the direction from one
// to the other exactly; a cylinder takes part with no radius, its whole solid
// its core. Overlapping, separating axes over the directions that can part
// them: the cylinder's axis, the other's faces, the cylinder's axis crossed
// with each of the other's edges (a side against an edge), and the way from
// the axis to the other's centre (a side against a corner or a round
// surface); the one with the least overlap. A rim meeting an edge has no
// axis of its own in that list, so a deep overlap there is pushed apart a
// little further than it must be: overlaps are shallow in a step, since
// contacts are found ahead (speculative), and only teleports reach them.
//
// WHERE. Each shape's part that faces the other along that direction (its
// feature): a box's or hull's face, edge or corner; a capsule's side or end;
// a cylinder's flat end (a disc), its side (a line along it) or a point of
// its rim; a ball's point. Taken within the margin, so a cylinder lying on a
// floor is its side line, both ends, and standing on it its end. Then the
// flat one, a face, is the reference, and the other's feature is clipped to
// it: a side line to a box's face gives a point at each end, a disc's rim to
// a face the rim inside it. A disc as the reference keeps what lies within
// its radius, exactly. Two lines give their closest points, or both ends
// where they lie alongside. Each point's separation is its height off the
// reference's plane, so a point is where the surfaces are, not where the axis
// test put them.
//
// Everything is exact operations, square roots and Bearing's own sine and
// cosine (core/math.js), in a fixed order: the same on every machine.

import { SPHERE, CAPSULE, BOX, HULL, TRIANGLE, CYLINDER, Placed, reducePoints, mix } from './collide.js';
import { closest, Closest, radius } from './gjk.js';
import { cos, sin } from './core/math.js';

/** Points round a disc, when its rim is clipped to another's face: 16, so it lies within 2% of the radius of the circle. */
const RIM = 16;
const RIM_COS = new Float64Array(RIM), RIM_SIN = new Float64Array(RIM);
for (let k = 0; k < RIM; k++) {
  const angle = (2 * 3.141592653589793 * k) / RIM;
  RIM_COS[k] = cos(angle); RIM_SIN[k] = sin(angle);
}
/** Below this, GJK's distance is rounding: its direction says nothing. A part in 2^26 of the shapes' reach. */
const ROUNDING = 1 / 67108864;
const POINT = 1, LINE = 2, FACE = 3, DISC = 4;

/** A shape's feature: its points (in order round a face), their ids, what kind, and a disc's centre, normal and radius. */
class Feature {
  constructor() {
    this.kind = POINT;
    this.count = 0;
    this.points = new Float64Array(3 * (RIM + 16));
    this.ids = new Int32Array(RIM + 16);
    this.id = 0;
    this.center = new Float64Array(3);
    this.normal = new Float64Array(3);
    this.radius = 0;
  }
}

const FA = new Feature(), FB = new Feature();
const near = new Closest();
// Made at first use: collide.js imports this file before its own classes exist.
let PA = null, PB = null;
function placed(s, which) {
  if (PA === null) { PA = new Placed(); PB = new Placed(); }
  return s.placed !== null ? s.placed.fresh(s) : (which === 0 ? PA : PB).set(s);
}
// Clipping's scratch: two polygons and their ids, and the points kept, as collide.js keeps them.
const POLY = new Float64Array(3 * 64), POLY_ID = new Int32Array(64), CLIP = new Float64Array(3 * 64), CLIP_ID = new Int32Array(64);
const kept = new Float64Array(3 * 64), keptSep = new Float64Array(64), keptId = new Uint32Array(64);
const D = new Float64Array(3);

/** The most of placed shape s along unit (x, y, z): its support, its rounding included. */
function most(s, x, y, z) {
  if (s.kind === TRIANGLE) {
    const p = s.points;
    return Math.max(p[0] * x + p[1] * y + p[2] * z, p[3] * x + p[4] * y + p[5] * z, p[6] * x + p[7] * y + p[8] * z);
  }
  const c = s.center, A = s.axes, h = s.half, base = c[0] * x + c[1] * y + c[2] * z;
  if (s.kind === SPHERE) return base + h[0];
  const t = A[3] * x + A[4] * y + A[5] * z;
  if (s.kind === CAPSULE) return base + Math.abs(t) * h[1] + h[0];
  if (s.kind === CYLINDER) return base + Math.abs(t) * h[1] + h[0] * Math.sqrt(Math.max(0, 1 - t * t));
  if (s.kind === BOX) {
    return base + Math.abs(A[0] * x + A[1] * y + A[2] * z) * h[0] + Math.abs(t) * h[1] + Math.abs(A[6] * x + A[7] * y + A[8] * z) * h[2];
  }
  const v = s.hull.vertices, lx = A[0] * x + A[1] * y + A[2] * z, lz = A[6] * x + A[7] * y + A[8] * z;
  let m = -Infinity;
  for (let i = 0; i < v.length; i += 3) m = Math.max(m, v[i] * lx + v[i + 1] * t + v[i + 2] * lz);
  return base + m;
}

/** How far b's least lies past a's most along unit (x, y, z): their gap along it, negative where they overlap. */
function gapAlong(a, b, x, y, z) {
  return -most(b, -x, -y, -z) - most(a, x, y, z);
}

// The axis test's best so far.
let bestGap = -Infinity, bx = 0, by = 0, bz = 0;

/** Tries axis (x, y, z), any length, both ways. */
function axis(a, b, x, y, z) {
  const l = Math.sqrt(x * x + y * y + z * z);
  if (!(l > 0)) return;
  x /= l; y /= l; z /= l;
  let g = gapAlong(a, b, x, y, z);
  if (g > bestGap) { bestGap = g; bx = x; by = y; bz = z; }
  g = gapAlong(a, b, -x, -y, -z);
  if (g > bestGap) { bestGap = g; bx = -x; by = -y; bz = -z; }
}

/** Every edge direction of s, crossed with unit u, tried as an axis; and s's own face normals. */
function edgesAndFaces(a, b, s, ux, uy, uz) {
  const A = s.axes;
  if (s.kind === SPHERE) return;
  if (s.kind === TRIANGLE) {
    const p = s.points;
    axis(a, b, A[0], A[1], A[2]);
    for (let e = 0; e < 3; e++) {
      const q = 3 * e, r = 3 * ((e + 1) % 3), tx = p[r] - p[q], ty = p[r + 1] - p[q + 1], tz = p[r + 2] - p[q + 2];
      axis(a, b, uy * tz - uz * ty, uz * tx - ux * tz, ux * ty - uy * tx);
    }
    return;
  }
  if (s.kind === CAPSULE || s.kind === CYLINDER) {
    axis(a, b, uy * A[5] - uz * A[4], uz * A[3] - ux * A[5], ux * A[4] - uy * A[3]);
    if (s.kind === CYLINDER) axis(a, b, A[3], A[4], A[5]);
    return;
  }
  if (s.kind === BOX) {
    for (let k = 0; k < 9; k += 3) {
      axis(a, b, A[k], A[k + 1], A[k + 2]);
      axis(a, b, uy * A[k + 2] - uz * A[k + 1], uz * A[k] - ux * A[k + 2], ux * A[k + 1] - uy * A[k]);
    }
    return;
  }
  const P = placed(s, 0), h = P.hull, v = P.v, p = P.planes, e = h.edges;
  for (let f = 0; f < p.length && f < 4 * (h.planes.length / 4); f += 4) axis(a, b, p[f], p[f + 1], p[f + 2]);
  for (let i = 0; i < e.length; i += 2) {
    const q = 3 * e[i], r = 3 * e[i + 1];
    const tx = v[r] - v[q], ty = v[r + 1] - v[q + 1], tz = v[r + 2] - v[q + 2];
    axis(a, b, uy * tz - uz * ty, uz * tx - ux * tz, ux * ty - uy * tx);
  }
}

/** The way from cylinder c's axis line out to point (x, y, z), as an axis. */
function radial(a, b, c, x, y, z) {
  const A = c.axes, dx = x - c.center[0], dy = y - c.center[1], dz = z - c.center[2];
  const t = dx * A[3] + dy * A[4] + dz * A[5];
  axis(a, b, dx - t * A[3], dy - t * A[4], dz - t * A[5]);
}

/** The direction parting overlapping a and b, from a towards b, into D; returns their gap along it, negative. */
function parting(a, b) {
  bestGap = -Infinity; bx = 0; by = 1; bz = 0;
  axis(a, b, b.center[0] - a.center[0], b.center[1] - a.center[1], b.center[2] - a.center[2]);
  for (let k = 0; k < 2; k++) {
    const c = k === 0 ? a : b, o = k === 0 ? b : a;
    if (c.kind !== CYLINDER) continue;
    const A = c.axes;
    axis(a, b, A[3], A[4], A[5]);
    edgesAndFaces(a, b, o, A[3], A[4], A[5]);
    radial(a, b, c, o.center[0], o.center[1], o.center[2]);
  }
  D[0] = bx; D[1] = by; D[2] = bz;
  return bestGap;
}

// ---- features ----

function point(F, x, y, z, id) {
  const k = F.count++;
  F.points[3 * k] = x; F.points[3 * k + 1] = y; F.points[3 * k + 2] = z;
  F.ids[k] = id;
}

/**
 * Placed shape s's feature farthest along unit (x, y, z), into F: what lies
 * within `slack` of its most that way. A face, line or point; a cylinder's
 * end is a disc, given as its rim's RIM points.
 */
function feature(s, x, y, z, slack, F, whole = false) {
  F.count = 0; F.kind = POINT; F.id = 0;
  const c = s.center, A = s.axes, h = s.half;
  if (s.kind === SPHERE) {
    point(F, c[0] + h[0] * x, c[1] + h[0] * y, c[2] + h[0] * z, 0);
    return;
  }
  if (s.kind === CAPSULE || s.kind === CYLINDER) {
    const t = A[3] * x + A[4] * y + A[5] * z, e = t >= 0 ? 1 : -1;
    if (s.kind === CAPSULE) {
      const r = h[0];
      if (Math.abs(t) * 2 * h[1] <= slack) {
        F.kind = LINE; F.id = 2;
        for (let k = -1; k <= 1; k += 2) point(F, c[0] + k * h[1] * A[3] + r * x, c[1] + k * h[1] * A[4] + r * y, c[2] + k * h[1] * A[5] + r * z, k > 0 ? 1 : 0);
      } else {
        point(F, c[0] + e * h[1] * A[3] + h[0] * x, c[1] + e * h[1] * A[4] + h[0] * y, c[2] + e * h[1] * A[5] + h[0] * z, e > 0 ? 1 : 0);
      }
      return;
    }
    const r = h[0], px = x - t * A[3], py = y - t * A[4], pz = z - t * A[5], sp = Math.sqrt(px * px + py * py + pz * pz);
    const ex = c[0] + e * h[1] * A[3], ey = c[1] + e * h[1] * A[4], ez = c[2] + e * h[1] * A[5];
    // Whole, against a face that is the reference: its end if that faces more
    // nearly along than its side, else its side, every point with its own
    // height, so a cylinder tilted on its end still stands on its rim there.
    if (sp * 2 * r <= slack || (whole && t * t >= 0.5)) {
      // Its end: a disc, round its own x and z.
      F.kind = DISC; F.id = e > 0 ? 1 : 0;
      F.center[0] = ex; F.center[1] = ey; F.center[2] = ez;
      F.normal[0] = e * A[3]; F.normal[1] = e * A[4]; F.normal[2] = e * A[5];
      F.radius = r;
      for (let k = 0; k < RIM; k++) {
        const u = r * RIM_COS[k], w = r * RIM_SIN[k];
        point(F, ex + u * A[0] + w * A[6], ey + u * A[1] + w * A[7], ez + u * A[2] + w * A[8], k);
      }
      return;
    }
    const rx = (r * px) / sp, ry = (r * py) / sp, rz = (r * pz) / sp;
    if (Math.abs(t) * 2 * h[1] <= slack || whole) {
      F.kind = LINE; F.id = 2;
      for (let k = -1; k <= 1; k += 2) point(F, c[0] + k * h[1] * A[3] + rx, c[1] + k * h[1] * A[4] + ry, c[2] + k * h[1] * A[5] + rz, k > 0 ? 1 : 0);
      return;
    }
    F.id = 3;
    point(F, ex + rx, ey + ry, ez + rz, e > 0 ? 1 : 0);
    return;
  }
  if (s.kind === TRIANGLE) {
    // Its face if all three corners are within the slack, else those that are: an edge, or the most one.
    const p = s.points;
    let top = -Infinity;
    for (let i = 0; i < 3; i++) top = Math.max(top, p[3 * i] * x + p[3 * i + 1] * y + p[3 * i + 2] * z);
    for (let i = 0; i < 3; i++) if (top - (p[3 * i] * x + p[3 * i + 1] * y + p[3 * i + 2] * z) <= slack) point(F, p[3 * i], p[3 * i + 1], p[3 * i + 2], i);
    F.kind = F.count === 3 ? FACE : F.count === 2 ? LINE : POINT;
    F.id = F.kind === FACE ? 8 : F.kind === LINE ? 9 + F.ids[0] + F.ids[1] : 12 + F.ids[0];
    return;
  }
  // A box or a hull: its face most along, if all of it is within the slack;
  // else the corners within it, an edge's two or the one.
  const P = placed(s, F === FA ? 0 : 1), hull = P.hull, v = P.v, p = P.planes;
  const V = hull.vertices.length / 3, faces = hull.planes.length / 4;
  let top = -Infinity, best = 0;
  for (let i = 0; i < V; i++) {
    const d = v[3 * i] * x + v[3 * i + 1] * y + v[3 * i + 2] * z;
    if (d > top) { top = d; best = i; }
  }
  let f = 0, along = -Infinity;
  for (let k = 0; k < faces; k++) {
    const d = p[4 * k] * x + p[4 * k + 1] * y + p[4 * k + 2] * z;
    if (d > along) { along = d; f = k; }
  }
  const i0 = hull.start[f], i1 = hull.start[f + 1];
  let flat = true;
  for (let i = i0; i < i1 && flat; i++) {
    const q = 3 * hull.loops[i];
    if (top - (v[q] * x + v[q + 1] * y + v[q + 2] * z) > slack) flat = false;
  }
  if (flat) {
    F.kind = FACE; F.id = 16 + f;
    for (let i = i0; i < i1; i++) { const q = 3 * hull.loops[i]; point(F, v[q], v[q + 1], v[q + 2], hull.loops[i]); }
    return;
  }
  // An edge from the most corner whose other end is within the slack too.
  const e = hull.edges;
  for (let i = 0; i < e.length; i += 2) {
    const other = e[i] === best ? e[i + 1] : e[i + 1] === best ? e[i] : -1;
    if (other < 0) continue;
    const q = 3 * other;
    if (top - (v[q] * x + v[q + 1] * y + v[q + 2] * z) > slack) continue;
    F.kind = LINE; F.id = 1024 + i / 2;
    point(F, v[3 * best], v[3 * best + 1], v[3 * best + 2], best);
    point(F, v[q], v[q + 1], v[q + 2], other);
    return;
  }
  F.id = 512 + best;
  point(F, v[3 * best], v[3 * best + 1], v[3 * best + 2], best);
}

/**
 * A cylinder's feature farthest along unit (x, y, z), for triangle.js: its
 * points into `points` (three numbers each), their ids, and how many: RIM for
 * its end, 2 for its side, 1 for a point of its rim.
 */
export function cylinderFeature(s, x, y, z, slack, points, ids) {
  feature(s, x, y, z, slack, FA, true);
  for (let k = 0; k < 3 * FA.count; k++) points[k] = FA.points[k];
  for (let k = 0; k < FA.count; k++) ids[k] = FA.ids[k];
  return FA.count;
}

// ---- contacts ----

let keptCount = 0;

/** A point of the incident shape at q, its separation off the reference plane: kept if within the margin, midway. */
function keep(qx, qy, qz, separation, mx, my, mz, margin, id) {
  if (separation > margin || keptCount === 64) return;
  const k = keptCount++, t = separation * 0.5;
  kept[3 * k] = qx - mx * t; kept[3 * k + 1] = qy - my * t; kept[3 * k + 2] = qz - mz * t;
  keptSep[k] = separation; keptId[k] = id;
}

/**
 * Incident feature I clipped to reference face R's sides, in R's plane
 * through its first point with outward normal (mx, my, mz): each point kept
 * by its height off it.
 */
function onFace(R, I, mx, my, mz, margin, flip) {
  let n = I.count;
  for (let k = 0; k < 3 * n; k++) POLY[k] = I.points[k];
  for (let k = 0; k < n; k++) POLY_ID[k] = I.ids[k];
  const rp = R.points, m = R.count;
  // The face's centre, to tell outward from inward for each side.
  let cx = 0, cy = 0, cz = 0;
  for (let k = 0; k < m; k++) { cx += rp[3 * k]; cy += rp[3 * k + 1]; cz += rp[3 * k + 2]; }
  cx /= m; cy /= m; cz /= m;
  let src = POLY, srcId = POLY_ID, dst = CLIP, dstId = CLIP_ID;
  for (let j = 0; j < m && n > 0; j++) {
    const a = 3 * j, b = 3 * ((j + 1) % m);
    const ex = rp[b] - rp[a], ey = rp[b + 1] - rp[a + 1], ez = rp[b + 2] - rp[a + 2];
    let sx = ey * mz - ez * my, sy = ez * mx - ex * mz, sz = ex * my - ey * mx;
    if (sx * (cx - rp[a]) + sy * (cy - rp[a + 1]) + sz * (cz - rp[a + 2]) > 0) { sx = -sx; sy = -sy; sz = -sz; }
    const offset = sx * rp[a] + sy * rp[a + 1] + sz * rp[a + 2];
    let out = 0;
    // A line clips as a line, not a closed polygon, so its one edge isn't walked twice.
    const closed = n > 2 ? n : n - 1;
    if (n === 1) {
      if (src[0] * sx + src[1] * sy + src[2] * sz - offset <= 0) { dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2]; dstId[0] = srcId[0]; out = 1; }
    } else {
      for (let v = 0; v < closed; v++) {
        const w = (v + 1) % n;
        const da = src[3 * v] * sx + src[3 * v + 1] * sy + src[3 * v + 2] * sz - offset;
        const db = src[3 * w] * sx + src[3 * w + 1] * sy + src[3 * w + 2] * sz - offset;
        if (da <= 0) { dst[3 * out] = src[3 * v]; dst[3 * out + 1] = src[3 * v + 1]; dst[3 * out + 2] = src[3 * v + 2]; dstId[out++] = srcId[v]; }
        if ((da <= 0) !== (db <= 0)) {
          const t = da / (da - db);
          dst[3 * out] = src[3 * v] + (src[3 * w] - src[3 * v]) * t;
          dst[3 * out + 1] = src[3 * v + 1] + (src[3 * w + 1] - src[3 * v + 1]) * t;
          dst[3 * out + 2] = src[3 * v + 2] + (src[3 * w + 2] - src[3 * v + 2]) * t;
          dstId[out++] = 4096 + 64 * j + v;
        }
        if (n === 2 && db <= 0) { dst[3 * out] = src[3 * w]; dst[3 * out + 1] = src[3 * w + 1]; dst[3 * out + 2] = src[3 * w + 2]; dstId[out++] = srcId[w]; }
      }
    }
    n = out;
    const t = src; src = dst; dst = t;
    const ti = srcId; srcId = dstId; dstId = ti;
  }
  const ox = rp[0], oy = rp[1], oz = rp[2];
  for (let k = 0; k < n; k++) {
    const qx = src[3 * k], qy = src[3 * k + 1], qz = src[3 * k + 2];
    keep(qx, qy, qz, (qx - ox) * mx + (qy - oy) * my + (qz - oz) * mz, mx, my, mz, margin, mix(flip ? 21 : 20, R.id, I.id, srcId[k]));
  }
}

/** Incident feature I against reference disc R, outward normal (mx, my, mz): what lies within its radius. */
function onDisc(R, I, mx, my, mz, margin, flip) {
  const c = R.center, r2 = R.radius * R.radius, kind = flip ? 23 : 22, p = I.points;
  if (I.kind === LINE) {
    // The line's stretch over the disc: |(p0 + t d) - c|² ≤ r² in its plane.
    const dx = p[3] - p[0], dy = p[4] - p[1], dz = p[5] - p[2];
    const ox = p[0] - c[0], oy = p[1] - c[1], oz = p[2] - c[2];
    const dn = dx * mx + dy * my + dz * mz, on = ox * mx + oy * my + oz * mz;
    const ux = dx - dn * mx, uy = dy - dn * my, uz = dz - dn * mz, vx = ox - on * mx, vy = oy - on * my, vz = oz - on * mz;
    const qa = ux * ux + uy * uy + uz * uz, qb = 2 * (ux * vx + uy * vy + uz * vz), qc = vx * vx + vy * vy + vz * vz - r2;
    let t0 = 0, t1 = 1;
    if (qa > 0) {
      const disc = qb * qb - 4 * qa * qc;
      if (disc < 0) return;
      const root = Math.sqrt(disc);
      t0 = Math.max(0, (-qb - root) / (2 * qa)); t1 = Math.min(1, (-qb + root) / (2 * qa));
      if (t0 > t1) return;
    } else if (qc > 0) return;
    for (let e = 0; e < 2; e++) {
      const t = e === 0 ? t0 : t1, x = p[0] + dx * t, y = p[1] + dy * t, z = p[2] + dz * t;
      const end = t === (e === 0 ? 0 : 1);
      keep(x, y, z, (x - c[0]) * mx + (y - c[1]) * my + (z - c[2]) * mz, mx, my, mz, margin, mix(kind, R.id, I.id, end ? I.ids[e] : 32 + e));
    }
    return;
  }
  for (let k = 0; k < I.count; k++) {
    const x = p[3 * k], y = p[3 * k + 1], z = p[3 * k + 2];
    const dx = x - c[0], dy = y - c[1], dz = z - c[2], t = dx * mx + dy * my + dz * mz;
    const ux = dx - t * mx, uy = dy - t * my, uz = dz - t * mz;
    if (ux * ux + uy * uy + uz * uz <= r2) keep(x, y, z, t, mx, my, mz, margin, mix(kind, R.id, I.id, I.ids[k]));
  }
  // Two discs: the reference's rim within the other's too, taken onto it.
  if (I.kind === DISC) {
    const ic = I.center, ir2 = I.radius * I.radius, rp = R.points, h = (ic[0] - c[0]) * mx + (ic[1] - c[1]) * my + (ic[2] - c[2]) * mz;
    for (let k = 0; k < R.count; k++) {
      const dx = rp[3 * k] - ic[0], dy = rp[3 * k + 1] - ic[1], dz = rp[3 * k + 2] - ic[2], t = dx * mx + dy * my + dz * mz;
      const ux = dx - t * mx, uy = dy - t * my, uz = dz - t * mz;
      if (ux * ux + uy * uy + uz * uz > ir2) continue;
      keep(rp[3 * k] + h * mx, rp[3 * k + 1] + h * my, rp[3 * k + 2] + h * mz, h, mx, my, mz, margin, mix(flip ? 25 : 24, R.id, I.id, R.ids[k]));
    }
  }
}

/** Two lines, A's and B's, normal (nx, ny, nz) from A to B: both ends where they lie alongside, else their closest points. */
function lines(A, B, nx, ny, nz, margin) {
  const a = A.points, b = B.points;
  const ux = a[3] - a[0], uy = a[4] - a[1], uz = a[5] - a[2], vx = b[3] - b[0], vy = b[4] - b[1], vz = b[5] - b[2];
  const uu = ux * ux + uy * uy + uz * uz, vv = vx * vx + vy * vy + vz * vz, uv = ux * vx + uy * vy + uz * vz;
  const cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
  if (cx * cx + cy * cy + cz * cz <= uu * vv * ROUNDING) {
    // Alongside: B's stretch over A's, its ends at either end of that.
    let t0 = ((a[0] - b[0]) * vx + (a[1] - b[1]) * vy + (a[2] - b[2]) * vz) / vv;
    let t1 = ((a[3] - b[0]) * vx + (a[4] - b[1]) * vy + (a[5] - b[2]) * vz) / vv;
    if (t0 > t1) { const t = t0; t0 = t1; t1 = t; }
    t0 = Math.max(0, t0); t1 = Math.min(1, t1);
    if (t0 > t1) return;
    for (let e = 0; e < 2; e++) {
      const t = e === 0 ? t0 : t1, x = b[0] + vx * t, y = b[1] + vy * t, z = b[2] + vz * t;
      keep(x, y, z, (x - a[0]) * nx + (y - a[1]) * ny + (z - a[2]) * nz, nx, ny, nz, margin, mix(26, A.id, B.id, e));
    }
    return;
  }
  // Crossing: the closest points of the two segments (Ericson 5.1.9).
  const rx = a[0] - b[0], ry = a[1] - b[1], rz = a[2] - b[2];
  const d1 = ux * rx + uy * ry + uz * rz, d2 = vx * rx + vy * ry + vz * rz, den = uu * vv - uv * uv;
  let s = den > 0 ? (uv * d2 - vv * d1) / den : 0;
  s = s < 0 ? 0 : s > 1 ? 1 : s;
  let t = (uv * s + d2) / vv;
  if (t < 0) { t = 0; s = Math.min(1, Math.max(0, -d1 / uu)); } else if (t > 1) { t = 1; s = Math.min(1, Math.max(0, (uv - d1) / uu)); }
  const x = b[0] + vx * t, y = b[1] + vy * t, z = b[2] + vz * t;
  const px = a[0] + ux * s, py = a[1] + uy * s, pz = a[2] + uz * s;
  keep(x, y, z, (x - px) * nx + (y - py) * ny + (z - pz) * nz, nx, ny, nz, margin, mix(27, A.id, B.id, 0));
}

/**
 * A ball a against cylinder b, exactly, as Bearing's other balls are: the
 * nearest point of the cylinder to its centre, or, its centre inside, the way
 * out through the nearer of the side and the end. GJK only approaches a round
 * surface (gjk.js), and a few tenths of a millimetre off a turned cylinder it
 * was out by a tenth of a millimetre.
 */
function ball(a, b, margin, out) {
  const c = a.center, o = b.center, A = b.axes, r = b.half[0], h = b.half[1], ra = a.half[0];
  const dx = c[0] - o[0], dy = c[1] - o[1], dz = c[2] - o[2];
  const lx = A[0] * dx + A[1] * dy + A[2] * dz, ly = A[3] * dx + A[4] * dy + A[5] * dz, lz = A[6] * dx + A[7] * dy + A[8] * dz;
  const radial = Math.sqrt(lx * lx + lz * lz);
  let qx, qy, qz, nx, ny, nz, gap;
  if (radial <= r && ly >= -h && ly <= h) {
    // Inside: out through the side or the end, whichever is nearer; on a tie, the end.
    const side = r - radial, end = h - Math.abs(ly);
    if (end <= side || radial === 0) { nx = 0; ny = ly < 0 ? 1 : -1; nz = 0; gap = -end - ra; }
    else { nx = -lx / radial; ny = 0; nz = -lz / radial; gap = -side - ra; }
  } else {
    const k = radial > r ? r / radial : 1;
    qx = lx * k; qy = ly < -h ? -h : ly > h ? h : ly; qz = lz * k;
    const ex = qx - lx, ey = qy - ly, ez = qz - lz, d = Math.sqrt(ex * ex + ey * ey + ez * ez);
    nx = ex / d; ny = ey / d; nz = ez / d; gap = d - ra;
  }
  if (gap > margin) return 0;
  // Back into the world: from the ball towards the cylinder, a point midway between the surfaces.
  const wx = A[0] * nx + A[3] * ny + A[6] * nz, wy = A[1] * nx + A[4] * ny + A[7] * nz, wz = A[2] * nx + A[5] * ny + A[8] * nz;
  out.normal[0] = wx; out.normal[1] = wy; out.normal[2] = wz;
  const t = ra + gap * 0.5;
  out.points[0] = c[0] + wx * t; out.points[1] = c[1] + wy * t; out.points[2] = c[2] + wz * t;
  out.separations[0] = gap; out.ids[0] = 0;
  out.count = 1;
  return 1;
}

/**
 * Where placed shapes a and b touch, or are within `margin` of it, when
 * either is a cylinder: into out, its normal from a towards b, as collide()
 * gives every pair.
 */
export function cylinderPair(a, b, margin, tolerance, out) {
  out.count = 0;
  if (a.kind === SPHERE) return ball(a, b, margin, out);
  const ra = radius(a), rb = radius(b);
  const d = closest(a, b, near);
  const reach = Math.max(Math.abs(a.half[0]), Math.abs(a.half[1])) + Math.max(Math.abs(b.half[0]), Math.abs(b.half[1]));
  let nx, ny, nz, gap;
  if (d > reach * ROUNDING) {
    nx = (near.b[0] - near.a[0]) / d; ny = (near.b[1] - near.a[1]) / d; nz = (near.b[2] - near.a[2]) / d;
    gap = d - ra - rb;
    if (gap > margin) return 0;
  } else {
    gap = parting(a, b);
    if (gap > margin) return 0;
    nx = D[0]; ny = D[1]; nz = D[2];
  }
  const slack = tolerance;
  feature(a, nx, ny, nz, slack, FA);
  feature(b, -nx, -ny, -nz, slack, FB);
  // A cylinder meeting a face is taken whole (see feature()).
  if (FA.kind === FACE && b.kind === CYLINDER) feature(b, -nx, -ny, -nz, slack, FB, true);
  else if (FB.kind === FACE && a.kind === CYLINDER) feature(a, nx, ny, nz, slack, FA, true);
  keptCount = 0;
  if (FA.kind === FACE) onFace(FA, FB, nx, ny, nz, margin, false);
  else if (FB.kind === FACE) onFace(FB, FA, -nx, -ny, -nz, margin, true);
  else if (FA.kind === DISC) onDisc(FA, FB, nx, ny, nz, margin, false);
  else if (FB.kind === DISC) onDisc(FB, FA, -nx, -ny, -nz, margin, true);
  else if (FA.kind === LINE && FB.kind === LINE) lines(FA, FB, nx, ny, nz, margin);
  if (keptCount === 0) {
    // One point, on a's surface where it reaches farthest towards b, the gap
    // found along the normal: a's feature's point most along it (the first
    // of equals), not the middle of a face, which can be far from b.
    const pa = FA.points;
    let k = 0, top = -Infinity;
    for (let i = 0; i < FA.count; i++) {
      const d = pa[3 * i] * nx + pa[3 * i + 1] * ny + pa[3 * i + 2] * nz;
      if (d > top) { top = d; k = i; }
    }
    keep(pa[3 * k] + nx * gap, pa[3 * k + 1] + ny * gap, pa[3 * k + 2] + nz * gap, gap, nx, ny, nz, margin, mix(28, FA.id, FB.id, FA.ids[k]));
  }
  out.normal[0] = nx; out.normal[1] = ny; out.normal[2] = nz;
  reducePoints(kept, keptSep, keptId, keptCount, nx, ny, nz, tolerance, out);
  return out.count;
}
