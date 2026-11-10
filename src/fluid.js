// WATER (milestone 22). A static or kinematic box given `fluid: true` is water:
// a sensor, so nothing collides with it, and every dynamic body in it is pushed
// up by the weight of the water it displaces, Archimedes' principle, at the
// centre of the part of it under the surface (the centre of buoyancy), and
// slowed by the water's damping by how much of it is under.
//
// The surface is the box's top face, a plane. How much of a body lies below a
// plane, and where that part's centre is, is worked out for each shape: a
// box's or hull's faces are cut by the plane exactly, a ball's cap has a
// closed form, and a capsule is summed slice by slice across its axis, each
// slice a disc cut by a line, also in closed form (CAPSULES). Only the top is
// asked: a body poking out of the box's side counts as under the surface
// there too, as Jolt's buoyancy, which is given only a plane, counts it.
//
// The push acts on every substep, as gravity does. A floating body is a
// spring: pushed down by d it is pushed back by the weight of the water a
// waterline of area A displaces, ρ g A d, and for a light or small body that
// spring is stiff. Pushed once a step it bobbed; on every substep it settles
// as a damped spring should.
//
// Which bodies a fluid holds comes from the sensor pairs the narrowphase lists
// as each step begins (World._sensed), so the push needs no search of its own,
// and a world with no fluid never comes here (World._fluidCount).

import { atan2, cos } from './core/math.js';
import { BOX, SPHERE, CAPSULE, HULL, Shape, Parts, placePart } from './collide.js';
import { load } from './contact.js';
import { resized } from './core/slots.js';

const DYNAMIC = 0;

/**
 * What submerged() adds to: the volume under the plane, m³, and its first
 * moment, m⁴, so the centre of buoyancy is moment / volume once every part
 * is in. It isn't re-entered.
 */
export const UNDER = new Float64Array(4);

/**
 * Adds to `out` the volume of placed shape s below the plane n · x = c (n
 * unit, pointing up out of the water), and that volume's first moment.
 */
export function submerged(s, nx, ny, nz, c, out) {
  if (s.kind === SPHERE) ball(s.center, s.half[0], nx, ny, nz, c, out);
  else if (s.kind === CAPSULE) capsule(s, nx, ny, nz, c, out);
  else if (s.kind === BOX) {
    const A = s.axes, h = s.half, o = s.center;
    for (let i = 0; i < 8; i++) {
      const x = i & 1 ? h[0] : -h[0], y = i & 2 ? h[1] : -h[1], z = i & 4 ? h[2] : -h[2];
      CORNERS[3 * i] = o[0] + A[0] * x + A[3] * y + A[6] * z;
      CORNERS[3 * i + 1] = o[1] + A[1] * x + A[4] * y + A[7] * z;
      CORNERS[3 * i + 2] = o[2] + A[2] * x + A[5] * y + A[8] * z;
    }
    solid(CORNERS, BOX_START, BOX_LOOPS, 6, o, nx, ny, nz, c, out);
  } else if (s.kind === HULL) {
    const hull = s.hull, v = hull.vertices, V = v.length / 3, A = s.axes, o = s.center;
    if (corners.length < 3 * V) corners = new Float64Array(3 * V);
    for (let i = 0; i < V; i++) {
      const x = v[3 * i], y = v[3 * i + 1], z = v[3 * i + 2];
      corners[3 * i] = o[0] + A[0] * x + A[3] * y + A[6] * z;
      corners[3 * i + 1] = o[1] + A[1] * x + A[4] * y + A[7] * z;
      corners[3 * i + 2] = o[2] + A[2] * x + A[5] * y + A[8] * z;
    }
    solid(corners, hull.start, hull.loops, hull.start.length - 1, o, nx, ny, nz, c, out);
  }
}

// ---- a ball ----

/**
 * A ball's cap under the plane. Of height h (0 to 2r), its volume is
 * π h² (3r - h) / 3 and its centre lies 3 (2r - h)² / (4 (3r - h)) from the
 * ball's, toward the cap: r for a sliver at the bottom, 3r/8 for a half.
 */
function ball(o, r, nx, ny, nz, c, out) {
  const h = r - (nx * o[0] + ny * o[1] + nz * o[2] - c);
  if (!(h > 0)) return;
  if (h >= 2 * r) {
    const v = (4 / 3) * Math.PI * r * r * r;
    out[0] += v; out[1] += v * o[0]; out[2] += v * o[1]; out[3] += v * o[2];
    return;
  }
  const v = Math.PI * h * h * (3 * r - h) / 3, k = 2 * r - h, down = 3 * k * k / (4 * (3 * r - h));
  out[0] += v;
  out[1] += v * (o[0] - nx * down); out[2] += v * (o[1] - ny * down); out[3] += v * (o[2] - nz * down);
}

// ---- a box or a hull ----
//
// A convex solid's volume below the plane is its faces cut by the plane, and
// the cap the plane closes them with. Summed as tetrahedra from a point on
// the plane, the cap's are all flat and add nothing, so only the faces are
// cut: each face's part below, fanned from its first corner. The faces run
// counter-clockwise seen from outside, so every tetrahedron of a face facing
// away from the point counts positive and every one facing it negative, and
// the sum is the volume whatever the point, exactly as far as rounding goes.

// A box's corners, numbered by bits: x the 1s, y the 2s, z the 4s, + when set.
const CORNERS = new Float64Array(24);
const BOX_START = Int32Array.from([0, 4, 8, 12, 16, 20, 24]);
const BOX_LOOPS = Int32Array.from([
  1, 3, 7, 5,    // +x
  0, 4, 6, 2,    // -x
  2, 6, 7, 3,    // +y
  0, 1, 5, 4,    // -y
  4, 5, 7, 6,    // +z
  0, 2, 3, 1,    // -z
]);
let corners = new Float64Array(3 * 64);
// A face's corners cut by the plane: at most one more than it had.
let cut = new Float64Array(3 * 16);

/** Faces start[f]..start[f + 1] of loops, over corners v, below n · x = c, into out; o is a point inside. */
function solid(v, start, loops, F, o, nx, ny, nz, c, out) {
  // The point the tetrahedra are taken from: o carried along n onto the plane.
  const lift = nx * o[0] + ny * o[1] + nz * o[2] - c;
  const px = o[0] - nx * lift, py = o[1] - ny * lift, pz = o[2] - nz * lift;
  let volume = 0, mx = 0, my = 0, mz = 0;
  for (let f = 0; f < F; f++) {
    const from = start[f], to = start[f + 1], n = to - from;
    if (cut.length < 3 * (n + 1)) cut = new Float64Array(6 * (n + 1));
    // The face's part below: each corner below kept, each edge that crosses cut where it crosses.
    let m = 0;
    for (let i = from; i < to; i++) {
      const a = 3 * loops[i], b = 3 * loops[i + 1 < to ? i + 1 : from];
      const ha = nx * v[a] + ny * v[a + 1] + nz * v[a + 2] - c, hb = nx * v[b] + ny * v[b + 1] + nz * v[b + 2] - c;
      if (ha <= 0) { cut[3 * m] = v[a]; cut[3 * m + 1] = v[a + 1]; cut[3 * m + 2] = v[a + 2]; m++; }
      if ((ha <= 0) !== (hb <= 0)) {
        const t = ha / (ha - hb);
        cut[3 * m] = v[a] + t * (v[b] - v[a]); cut[3 * m + 1] = v[a + 1] + t * (v[b + 1] - v[a + 1]);
        cut[3 * m + 2] = v[a + 2] + t * (v[b + 2] - v[a + 2]);
        m++;
      }
    }
    const ax = cut[0] - px, ay = cut[1] - py, az = cut[2] - pz;
    for (let k = 1; k + 1 < m; k++) {
      const bx = cut[3 * k] - px, by = cut[3 * k + 1] - py, bz = cut[3 * k + 2] - pz;
      const cx = cut[3 * k + 3] - px, cy = cut[3 * k + 4] - py, cz = cut[3 * k + 5] - pz;
      const six = ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx);
      // A tetrahedron's centre is its corners' mean; p's share is added once, below.
      volume += six;
      mx += six * (ax + bx + cx); my += six * (ay + by + cy); mz += six * (az + bz + cz);
    }
  }
  if (!(volume > 0)) return;
  const v6 = volume / 6;
  out[0] += v6;
  out[1] += v6 * px + mx / 24; out[2] += v6 * py + my / 24; out[3] += v6 * pz + mz / 24;
}

// ---- a capsule ----
//
// CAPSULES. Sliced square to its axis, a capsule is discs: of its radius r
// along its segment, shrinking to its tips over each end. The plane cuts each
// disc along a line, and the part of a disc of radius ρ beyond a line e from
// its centre is ρ² acos(e/ρ) - e √(ρ² - e²), its moment about the centre
// (2/3)(ρ² - e²)^(3/2): both closed forms. Summed along the axis by
// Gauss-Legendre, between the places where the line leaves a disc's edge
// (where the sum's integrand has a kink) so each piece is smooth, its points
// bunched toward the piece's ends. Measured over 300 random poses against
// 400,000 slices each, the volume was within 2^-21 (5e-7) of the capsule's;
// without the bunching 2^-16.7. An upright capsule, whose discs are each
// wholly under or not, is exact to rounding (test/fluid.test.js).

/** Gauss-Legendre points a piece of a capsule's axis. See CAPSULES. */
const NODES = 8;
const X = new Float64Array(NODES), W = new Float64Array(NODES);
// The roots of the Legendre polynomial of degree NODES, by Newton's method
// from the usual first guesses, and their weights: exact operations and
// Bearing's own cosine, so the same on every machine.
for (let i = 0; i < NODES; i++) {
  let x = cos(Math.PI * (i + 0.75) / (NODES + 0.5)), d = 0;
  for (let k = 0; k < 100; k++) {
    let p0 = 1, p1 = x;
    for (let j = 2; j <= NODES; j++) { const p = ((2 * j - 1) * x * p1 - (j - 1) * p0) / j; p0 = p1; p1 = p; }
    d = NODES * (x * p1 - p0) / (x * x - 1);
    const next = x - p1 / d;
    if (next === x) break;
    x = next;
  }
  X[i] = x;
  W[i] = 2 / ((1 - x * x) * d * d);
}
/** The places along the axis where a piece ends: at most the 4 ends and 2 kinks on each of the 3 parts. */
const BREAKS = new Float64Array(10);

function capsule(s, nx, ny, nz, c, out) {
  const o = s.center, A = s.axes, r = s.half[0], half = s.half[1];
  const ax = A[3], ay = A[4], az = A[5];
  const h0 = nx * o[0] + ny * o[1] + nz * o[2] - c, k = nx * ax + ny * ay + nz * az;
  // Wholly above or below: nothing to cut.
  const reach = Math.abs(k) * half + r;
  if (h0 >= reach) return;
  if (h0 <= -reach) {
    const v = Math.PI * r * r * (2 * half + (4 / 3) * r);
    out[0] += v; out[1] += v * o[0]; out[2] += v * o[1]; out[3] += v * o[2];
    return;
  }
  // The plane's normal square to the axis, and how long it is: each disc is cut by a line square to it.
  let q2 = 1 - k * k;
  if (q2 < 0) q2 = 0;
  const q = Math.sqrt(q2);
  const px = q > 0 ? (nx - k * ax) / q : 0, py = q > 0 ? (ny - k * ay) / q : 0, pz = q > 0 ? (nz - k * az) / q : 0;
  // The pieces: the segment and the two caps, cut again wherever a disc's edge meets the line.
  let n = 0;
  BREAKS[n++] = -half - r; BREAKS[n++] = -half; BREAKS[n++] = half; BREAKS[n++] = half + r;
  // (Written out, not looped over little arrays: a step makes no new arrays.)
  if (k !== 0) {
    const t0 = (-r * q - h0) / k, t1 = (r * q - h0) / k;
    if (t0 > -half && t0 < half) BREAKS[n++] = t0;
    if (t1 > -half && t1 < half) BREAKS[n++] = t1;
  }
  if (q > 0) {
    // Over a cap, with u from the segment's end: (H + k u)² = q² (r² - u²), H the height there.
    for (let side = 1; side >= -1; side -= 2) {
      const H = h0 + side * k * half, disc = r * r - H * H;
      if (!(disc > 0)) continue;
      const root = q * Math.sqrt(disc), mid = -H * k * side, u0 = mid - root, u1 = mid + root;
      if (u0 > 0 && u0 < r) BREAKS[n++] = side * (half + u0);
      if (u1 > 0 && u1 < r) BREAKS[n++] = side * (half + u1);
    }
  }
  // In order, for the pieces between: few enough that inserting each is quickest.
  for (let i = 1; i < n; i++) {
    const t = BREAKS[i];
    let j = i - 1;
    for (; j >= 0 && BREAKS[j] > t; j--) BREAKS[j + 1] = BREAKS[j];
    BREAKS[j + 1] = t;
  }
  let volume = 0, along = 0, aside = 0;
  for (let i = 0; i + 1 < n; i++) {
    const t0 = BREAKS[i], t1 = BREAKS[i + 1];
    if (!(t1 > t0)) continue;
    const mid = 0.5 * (t0 + t1), span = 0.5 * (t1 - t0);
    for (let g = 0; g < NODES; g++) {
      // x = (3u - u³) / 2 bunches the points toward the piece's ends, where the
      // line leaving a disc's edge goes as a power 3/2 of the distance, and
      // makes that a smooth cube for the points to sum.
      const u = X[g], x = 0.5 * u * (3 - u * u), t = mid + span * x, w = span * W[g] * 1.5 * (1 - u * u);
      const end = t > half ? t - half : t < -half ? -half - t : 0, rho2 = r * r - end * end;
      if (!(rho2 > 0)) continue;
      const height = h0 + k * t;
      // How far into the disc, along the way down, the line lies past its centre.
      let area = 0, moment = 0;
      if (q === 0) area = height < 0 ? Math.PI * rho2 : 0;
      else {
        const e = -height / q;
        if (e * e >= rho2) area = e > 0 ? Math.PI * rho2 : 0;
        else {
          const root = Math.sqrt(rho2 - e * e);
          area = Math.PI * rho2 - (rho2 * atan2(root, e) - e * root);
          moment = (2 / 3) * root * root * root;
        }
      }
      volume += w * area; along += w * area * t; aside += w * moment;
    }
  }
  if (!(volume > 0)) return;
  out[0] += volume;
  // The part below lies down the plane's slope across each disc: against p.
  out[1] += volume * o[0] + along * ax - aside * px;
  out[2] += volume * o[1] + along * ay - aside * py;
  out[3] += volume * o[2] + along * az - aside * pz;
}

// ---- the push ----

const SHAPE = new Shape(), POSE = new Float64Array(7);

/**
 * Every dynamic body a fluid holds, pushed up and slowed for a substep of h
 * seconds: see WATER. Called as each substep starts, after gravity.
 */
export function float(world, h) {
  const pairs = world._pairs, sensed = world._sensed, fluid = world._fluid, type = world._type;
  const frozen = world._frozen, awake = world._awake, pos = world._position, rot = world._rotation;
  const v = world._velocity, w = world._angular, g = world._gravity, dims = world._dims;
  for (let i = 0; i < world._sensedCount; i++) {
    const p = sensed[i], a = pairs.a[p], b = pairs.b[p];
    const f = fluid[a] === 1 ? a : fluid[b] === 1 ? b : -1;
    if (f < 0) continue;
    const s = f === a ? b : a;
    if (type[s] !== DYNAMIC || frozen[s] !== 0 || awake[s] === 0) continue;
    // The surface: the box's top, its own +y face.
    const f3 = 3 * f;
    const pose = world._pose(f, POSE), x = pose[3], y = pose[4], z = pose[5], qw = pose[6];
    const nx = 2 * (x * y - z * qw), ny = 1 - 2 * (x * x + z * z), nz = 2 * (y * z + x * qw);
    const top = nx * pose[0] + ny * pose[1] + nz * pose[2] + dims[f3 + 1] * 0.5;
    UNDER[0] = 0; UNDER[1] = 0; UNDER[2] = 0; UNDER[3] = 0;
    const inner = world._inner[s];
    if (inner instanceof Parts) {
      for (let k = 0; k < inner.list.length; k++) submerged(placePart(inner.list[k], pos, 3 * s, rot, 4 * s, SHAPE), nx, ny, nz, top, UNDER);
    } else {
      // Turned as it is now: load() gives the axes World._bounds worked out
      // as the step began, and the push, read from them on later substeps,
      // lagged the body's own turning and fed every rock: an undamped plank
      // rocked from 1 rad/s to 5.5 in 30 s at 8 substeps, and held at 1.
      load(world, s, SHAPE);
      SHAPE.place(pos[3 * s], pos[3 * s + 1], pos[3 * s + 2], rot, 4 * s);
      submerged(SHAPE, nx, ny, nz, top, UNDER);
    }
    const under = UNDER[0];
    if (!(under > 0)) continue;
    const s3 = 3 * s, mass = world._mass[s], density = world._density[f];
    // The water's weight it displaces, against gravity, as an impulse over h.
    const k = -density * under * h;
    const jx = k * g[0], jy = k * g[1], jz = k * g[2];
    v[s3] += jx / mass; v[s3 + 1] += jy / mass; v[s3 + 2] += jz / mass;
    const rx = UNDER[1] / under - pos[s3], ry = UNDER[2] / under - pos[s3 + 1], rz = UNDER[3] / under - pos[s3 + 2];
    world._turn(s, ry * jz - rz * jy, rz * jx - rx * jz, rx * jy - ry * jx);
    // Slowed against the water's flow by how much of it is under, as the
    // body's own damping slows it (World._accelerate): v / (1 + c h).
    let share = under * world._density[s] / mass;
    if (share > 1) share = 1;
    const damping = world._damping[f], angular = world._angularDamping[f], flow = world._flow;
    if (damping > 0) {
      const d = 1 / (1 + h * damping * share);
      v[s3] = flow[f3] + (v[s3] - flow[f3]) * d;
      v[s3 + 1] = flow[f3 + 1] + (v[s3 + 1] - flow[f3 + 1]) * d;
      v[s3 + 2] = flow[f3 + 2] + (v[s3 + 2] - flow[f3 + 2]) * d;
    }
    if (angular > 0) {
      const d = 1 / (1 + h * angular * share);
      w[s3] *= d; w[s3 + 1] *= d; w[s3 + 2] *= d;
    }
  }
}
