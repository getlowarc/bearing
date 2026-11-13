// SOFT BODIES (milestone 25). A soft body is a triangle mesh whose corners
// move on their own: particles, held together by constraints. An open mesh
// is cloth; a closed one given `pressure` keeps its volume, a ball or a
// balloon. They fall, are pushed by the air, and collide with rigid bodies
// both ways, inside the same substeps as the rigid solver.
//
// XPBD (Macklin, Müller and Chentanez, 2016), one pass a substep, as "Small
// Steps in Physics Simulation" (2019) runs it and Jolt's soft bodies do. Each
// substep moves every particle by its velocity, then moves them back onto
// their constraints one at a time, each by the share its inverse mass gives
// it, and reads the velocity back from how far they went. A constraint with
// compliance α (m/N, the inverse of a stiffness) gives by α / h² against the
// particles' inverse masses, so how stiff it is doesn't depend on the step;
// α = 0 holds it exactly.
//
// The constraints: each edge is kept from growing past its length
// (`stretch`), never from shrinking, as cloth buckles rather than resists
// being squashed (both ways, a ball's bottom couldn't flatten on a floor
// without buckling, and buzzed for good); the two corners
// across each edge shared by two triangles keep their distance (`bend`), the
// simplest bending rule, which resists folding as that distance shortens; a
// closed mesh keeps pressure times its volume; an attached corner stays at
// its point on a body, or in the world; and each corner stays out of rigid
// bodies by its radius, half the `thickness`, with friction.
//
// COLLISIONS. Found once a step, before the narrowphase: each corner's two
// nearest rigid bodies within how far either can move in the step, each kept
// as a plane in that body's own frame (Jolt's way, which keeps one). One was
// measured too few: a corner of cloth heaped where a box stands on the floor
// was pushed out of the box one step and the floor the next, buzzing for good. Each
// substep the corner is moved out of the plane where the body will be at the
// substep's end, and the body is pushed back by the same impulse, through
// its mass and inertia at that point: two ways. A sleeping or frozen body is
// as heavy as the floor, and a corner newly touching one wakes it.
//
// Known limits, each with what to do when it is reached:
// - Corners collide, not triangles: a rigid edge thinner than the mesh's
//   spacing can pass between corners. Use a finer mesh where it must not.
// - Two planes a corner: one pressed into where three bodies meet is held
//   out of the nearer two only. More planes, if it shows.
// - A soft body doesn't collide with itself or with other soft bodies, isn't
//   seen by queries or contact events, and doesn't float.
// - A closed body heavy enough to sag under its own weight (a 60 kg ball of
//   water) can go on wobbling a few tenths of a millimetre a step and not
//   sleep: its edges and its volume, each put right alone, never quite agree.
//   A second pass of them a substep was measured and helped some balls and
//   not others. A solve of them together (a block, as contacts have) would.
// - A body much heavier than the cloth under it sinks into it further than
//   the cloth's stiffness says: the corners it presses take the push by their
//   own small mass. Making a corner on a taut tether heavy was tried and
//   pumped energy into a box hung from cloth; leave it unless it shows.
//
// Everything is exact operations and Math.sqrt, in a fixed order, so the
// same on every machine (docs/API.md, Determinism).

import { fail, show, options, known, positive, vec3, quat, readOnly, handle } from './core/check.js';
import { Slots, resized, MAX_SLOTS } from './core/slots.js';
import { rotate } from './core/math.js';
import { Shape, SPHERE } from './collide.js';
import { PIXEL, turnedInverse } from './contact.js';
import { nearestOf } from './query.js';
import { Body } from './body.js';
import { SLEEP_TIME } from './world.js';

const DYNAMIC = 0, STATIC = 1;   // World's type codes

export const OPTIONS = ['vertices', 'indices', 'density', 'stretch', 'bend', 'thickness', 'pressure', 'attach',
  'friction', 'damping', 'wind', 'layers', 'mask', 'position', 'rotation', 'velocity'];
const ATTACH_OPTIONS = ['vertex', 'body'];

/**
 * What a soft body is left out: a 1 m square sheet of two triangles, flat in
 * x and z, as a body starts as a 1 m cube. Density is water's, as a body's.
 * Cloth 1 mm thick, about everyday cloth's, so a square metre of it weighs
 * 1 kg (a heavy cotton, canvas). Stretching not at all, folding freely, no
 * pressure; friction, damping, layers and mask as a body's.
 */
const DEFAULTS = Object.freeze({
  vertices: [-0.5, 0, -0.5, 0.5, 0, -0.5, 0.5, 0, 0.5, -0.5, 0, 0.5], indices: [0, 2, 1, 0, 3, 2],
  density: 1000, stretch: 1, bend: 0, thickness: 0.001, pressure: 0, attach: [], friction: 0.5, damping: 0,
  wind: [0, 0, 0], layers: 1, mask: 0xffffffff, position: [0, 0, 0], rotation: [0, 0, 0, 1], velocity: [0, 0, 0],
});

const THIRD = 1 / 3, NINTH = 1 / 9;
/**
 * How fast a soft body's own wobble and sway die away, 1/s, by settle():
 * measured, the least whole rate that put six cloths dropped over a box to
 * sleep within 10 s (4.5 to 8.8 s). With none, two of the six still swayed
 * at 30 s: a hanging fold has little else to lose its motion to, and the air
 * takes little at a few centimetres a second; real cloth loses it to
 * friction between its threads. Only the deformation is slowed: how the body
 * falls, slides or spins as a whole is untouched, and a flag in a wind with
 * any crosswise part to it waves as well at 2 as at 0 (5 to 40 mm a step).
 */
const SETTLE = 1;
/** Planes a corner (COLLISIONS). */
const PLANES = 2;   // contacts() reads a corner as c >> 1

/** Air at 20 °C at sea level, kg/m³: what pushes on cloth. */
const AIR = 1.2;
/** The drag coefficient of a flat square plate across the flow (Hoerner, Fluid-Dynamic Drag, 1965): each triangle's. */
const DRAG = 1.17;
/**
 * Earth's gravity, m/s²: what `stretch` and `bend` are measured against, so
 * they mean the same in any world, a world without gravity too (STIFFNESS).
 */
const EARTH = 9.81;

/**
 * STIFFNESS. `stretch` s is how far a soft body hung by one edge under its
 * own weight, in Earth's gravity, would stretch at the top: by (1 - s) / s of
 * its length, so 1 not at all, 0.99 by about 1%, 0.5 by its own length, 0
 * with nothing holding it. That is a sheet of Young's modulus times thickness
 * Y = ρ g L s / (1 - s), ρ its mass for each square metre and L its size; an
 * edge of length l with area a beside it (the triangles it borders) is then
 * a spring of Y a / l², compliance l² / (Y a): exactly a sheet of Y where
 * the triangles are equilateral. The sheet's
 * own numbers make it the same at any size, weight or mesh spacing. `bend`
 * is the same, for the distance across each fold.
 */
function compliance(s, rho, L, l, a) {
  if (s >= 1) return 0;
  return (l * l) * (1 - s) / (rho * EARTH * L * s * a);
}

/** The store: a world's soft bodies, by slot, with ids as bodies' are. */
export class Softs {
  constructor() {
    this.slots = new Slots();
    this.list = [];
  }

  get count() { return this.slots.count; }
}

/** One soft body's state. Its fields named here first: see World's FAST PROPERTIES. */
export class Soft {
  constructor() {
    // As given: corners in its own frame, triangles, pose.
    this.vertices = null; this.indices = null; this.position = null; this.rotation = null;
    // Settings.
    this.density = 0; this.stretch = 0; this.bend = 0; this.thickness = 0; this.pressure = 0;
    this.friction = 0; this.damping = 0; this.wind = null; this.layers = 0; this.mask = 0;
    // Particles: position, velocity, inverse mass; where each rests (sleep), and its public copy.
    this.n = 0; this.x = null; this.v = null; this.w = null; this.rest = null; this.positions = null;
    this.mass = 0;
    // Constraints: edges and their lengths and compliances, folds the same, the volume.
    this.edges = null; this.length = null; this.give = null;
    this.bends = null; this.across = null; this.bendGive = null;
    this.closed = 0; this.volume = 0;
    // Its size, for stiffness; the mean edge, for its tolerance.
    this.size = 0; this.spacing = 0;
    // Attachments: the corner, the body's id (-1: the world), the point in that body's frame.
    this.attached = 0; this.attachVertex = null; this.attachBody = null; this.attachLocal = null;
    // By corner, the body it is attached to (an id), or -2, so it doesn't collide with that one.
    this.pinned = null;
    // By corner, the nearest attachment along the mesh (TETHERS), or -1, and how far along the mesh it is.
    this.tether = null; this.reach = null;
    // By corner, this step's PLANES planes: the body's slot (-1 none) and id, a point on it and its outward normal, in its frame.
    this.hit = null; this.hitId = null; this.hitPoint = null; this.hitNormal = null;
    // Sleeping, as a body's: time still, asleep or not, and its box.
    this.still = 0; this.asleep = 0; this.box = null;
    // Scratch: where each corner began the substep, the volume's gradient, the nearest plane found, where each attachment is going.
    this.prev = null; this.grad = null; this.near = null; this.anchors = null; this.pressed = null; this.pressedBy = null; this.blocked = null;
  }
}

/** A handle to one soft body: a world and an id, as a Body is. Compare `a.id === b.id`, not `a === b`. */
export class SoftBody {
  #world;
  #id;

  constructor(world, id) {
    this.#world = world;
    this.#id = id;
  }

  get world() { return this.#world; }
  set world(_) { readOnly('softBody.world'); }

  /** A number, unique to this soft body for the life of its world. */
  get id() { return this.#id; }
  set id(_) { readOnly('softBody.id'); }

  /** false once the soft body is removed, or its world destroyed. Never throws. */
  get alive() { return !this.#world._destroyed && this.#world._softs.slots.slotOf(this.#id) >= 0; }
  set alive(_) { readOnly('softBody.alive'); }

  /** true while it is asleep: at rest, and left out of steps until something disturbs it. */
  get asleep() { return softOf(this.#world, 'softBody.asleep', this.#id).asleep === 1; }
  set asleep(_) { readOnly('softBody.asleep'); }

  /** The corners where the last step left them, x, y, z each, m: the world's own array, updated by every step. */
  get positions() { return softOf(this.#world, 'softBody.positions', this.#id).positions; }
  set positions(_) { readOnly('softBody.positions'); }

  /** Same as world.remove(softBody). Does nothing if it is already gone. */
  destroy() {
    this.#world._remove('destroy', this);
  }
}

function softOf(world, caller, id) {
  world._live(caller);
  const s = world._softs.slots.slotOf(id);
  if (s < 0) fail(caller, 'this soft body was removed');
  return world._softs.list[s];
}

/** The slot of a soft body handle given to `caller`. */
export function softSlot(world, caller, soft, gone = false) {
  world._live(caller);
  return handle(caller, 'softBody', soft, world, SoftBody, world._softs.slots, gone);
}

// ---- making, changing and reading ----

export function addSoftBody(world, given) {
  const caller = 'addSoftBody';
  world._live(caller);
  const o = options(caller, given);
  known(caller, o, OPTIONS);
  if ((o.vertices === undefined) !== (o.indices === undefined)) fail(caller, 'vertices and indices must be given together');
  const soft = new Soft();
  settings(caller, soft, DEFAULTS, o);
  topology(caller, soft, o.vertices === undefined ? DEFAULTS.vertices : vertexList(caller, o.vertices),
    o.indices === undefined ? DEFAULTS.indices : o.indices);
  if (soft.pressure > 0 && soft.closed === 0) fail(caller, 'pressure needs a closed mesh: every edge between two triangles, wound the same way round');
  masses(soft);
  place(soft, o.velocity === undefined ? DEFAULTS.velocity : vec3(caller, 'velocity', o.velocity));
  attach(world, caller, soft, o.attach === undefined ? [] : o.attach);
  const store = world._softs, slots = store.slots;
  let s = slots.take();
  if (s < 0) {
    if (slots.capacity >= slots.limit) fail(caller, `a world holds at most ${MAX_SLOTS} soft bodies (the slot field of an id is 24 bits)`);
    slots.resize(Math.min(Math.max(slots.capacity * 2, 1), slots.limit));
    s = slots.take();
  }
  store.list[s] = soft;
  return new SoftBody(world, slots.id(s));
}

export function setSoftBody(world, handle_, changes) {
  const caller = 'setSoftBody';
  const s = softSlot(world, caller, handle_);
  const o = options(caller, changes, 'changes');
  known(caller, o, OPTIONS);
  const soft = world._softs.list[s];
  // Everything checked before anything is written (rule 22): onto a copy.
  const next = Object.assign(new Soft(), soft);
  settings(caller, next, soft, o);
  const reshaped = o.vertices !== undefined || o.indices !== undefined;
  if (reshaped) {
    topology(caller, next, o.vertices === undefined ? soft.vertices : vertexList(caller, o.vertices),
      o.indices === undefined ? soft.indices : o.indices);
  }
  if (next.pressure > 0 && next.closed === 0) fail(caller, 'pressure needs a closed mesh: every edge between two triangles, wound the same way round');
  const velocity = o.velocity === undefined ? null : vec3(caller, 'velocity', o.velocity);
  if (o.attach !== undefined) checkAttach(world, caller, next, o.attach);
  // Written: the copy's fields onto the soft body, whose arrays the handle's positions are.
  if (reshaped) {
    masses(next);
    place(next, velocity === null ? [0, 0, 0] : velocity);
  } else {
    if (o.density !== undefined || o.thickness !== undefined || o.stretch !== undefined || o.bend !== undefined) masses(next);
    if (o.position !== undefined || o.rotation !== undefined) teleport(next, soft.position, soft.rotation);
    if (velocity !== null) for (let i = 0; i < next.n; i++) { next.v[3 * i] = velocity[0]; next.v[3 * i + 1] = velocity[1]; next.v[3 * i + 2] = velocity[2]; }
  }
  Object.assign(soft, next);
  if (o.attach !== undefined) attach(world, caller, soft, o.attach);
  else if (reshaped) soft.attached = 0;
  for (let i = 0; i < soft.n; i++) soft.pinned[i] = -2;
  for (let k = 0; k < soft.attached; k++) soft.pinned[soft.attachVertex[k]] = soft.attachBody[k];
  wakeSoft(soft);
  copyOut(soft);
}

export function softBodyOf(world, handle_) {
  const s = softSlot(world, 'softBodyOf', handle_);
  const soft = world._softs.list[s];
  const attachList = [];
  for (let k = 0; k < soft.attached; k++) {
    const id = soft.attachBody[k];
    attachList.push({ vertex: soft.attachVertex[k], body: id < 0 ? null : world.body(id) });
  }
  // Its velocity: the mean of its corners', weighed by their mass.
  let mx = 0, my = 0, mz = 0, m = 0;
  for (let i = 0; i < soft.n; i++) {
    const mi = 1 / soft.w[i];
    mx += mi * soft.v[3 * i]; my += mi * soft.v[3 * i + 1]; mz += mi * soft.v[3 * i + 2]; m += mi;
  }
  return {
    vertices: Array.from(soft.vertices), indices: Array.from(soft.indices),
    density: soft.density, stretch: soft.stretch, bend: soft.bend, thickness: soft.thickness, pressure: soft.pressure,
    attach: attachList, friction: soft.friction, damping: soft.damping, wind: Array.from(soft.wind),
    layers: soft.layers, mask: soft.mask, position: Array.from(soft.position), rotation: Array.from(soft.rotation),
    velocity: [mx / m, my / m, mz / m],
  };
}

/** Frees soft body slot s. */
export function dropSoft(world, s) {
  const store = world._softs;
  store.list[s] = null;
  store.slots.release(s);
  const top = store.slots.top;
  if (top <= store.slots.capacity / 4) {
    let capacity = top === 0 ? 0 : 1;
    while (capacity < top) capacity *= 2;
    store.slots.resize(capacity);
    store.list.length = capacity;
  }
}

/** Body slot s, id `id`, is being removed: what was attached to it lets go, and whatever touched it wakes. */
export function bodyRemoved(world, s, id) {
  for (const soft of world._softs.list) {
    if (soft === null || soft === undefined) continue;
    let kept = 0, woke = false;
    for (let k = 0; k < soft.attached; k++) {
      if (soft.attachBody[k] === id) { soft.pinned[soft.attachVertex[k]] = -2; woke = true; continue; }
      soft.attachVertex[kept] = soft.attachVertex[k];
      soft.attachBody[kept] = soft.attachBody[k];
      for (let c = 0; c < 3; c++) soft.attachLocal[3 * kept + c] = soft.attachLocal[3 * k + c];
      kept++;
    }
    soft.attached = kept;
    for (let k = 0; k < kept; k++) soft.pinned[soft.attachVertex[k]] = soft.attachBody[k];
    if (woke) tethers(world, soft);
    for (let c = 0; c < soft.hit.length; c++) if (soft.hit[c] === s) { soft.hit[c] = -1; woke = true; }
    if (woke) wakeSoft(soft);
  }
}

/**
 * Body slot s, id `id`, was set by hand (setBody): each point attached to it
 * stays where it was on the body as given, though its centre of mass,
 * principal axes or place moved. `before` is [centre, rotation, origin's
 * position, origin's rotation] as they were, 14 numbers; the points are kept
 * in the centre's frame.
 */
export function bodySet(world, s, id, before, after) {
  const p = world._position, q = world._rotation, s3 = 3 * s, s4 = 4 * s;
  for (const soft of world._softs.list) {
    if (soft === null || soft === undefined) continue;
    for (let k = 0; k < soft.attached; k++) {
      if (soft.attachBody[k] !== id) continue;
      const l = soft.attachLocal, k3 = 3 * k;
      // Into the world as it was, then into the origin's frame, which moves with the body as given.
      rotate(before, 3, l[k3], l[k3 + 1], l[k3 + 2], false, V);
      rotate(before, 10, V[0] + before[0] - before[7], V[1] + before[1] - before[8], V[2] + before[2] - before[9], true, V);
      rotate(after, 3, V[0], V[1], V[2], false, V);
      rotate(q, s4, V[0] + after[0] - p[s3], V[1] + after[1] - p[s3 + 1], V[2] + after[2] - p[s3 + 2], true, V);
      l[k3] = V[0]; l[k3 + 1] = V[1]; l[k3 + 2] = V[2];
    }
    // Its type may have changed, and what tethers from it with it.
    for (let k = 0; k < soft.attached; k++) if (soft.attachBody[k] === id) { tethers(world, soft); wakeSoft(soft); break; }
  }
}

/** The settings, from `given` over `base`, checked, onto soft. */
function settings(caller, soft, base, given) {
  const has = (k) => given[k] !== undefined;
  soft.density = has('density') ? positive(caller, 'density', given.density) : base.density;
  soft.thickness = has('thickness') ? positive(caller, 'thickness', given.thickness) : base.thickness;
  soft.stretch = has('stretch') ? fraction(caller, 'stretch', given.stretch) : base.stretch;
  soft.bend = has('bend') ? fraction(caller, 'bend', given.bend) : base.bend;
  soft.pressure = has('pressure') ? atLeastZero(caller, 'pressure', given.pressure) : base.pressure;
  soft.friction = has('friction') ? atLeastZero(caller, 'friction', given.friction) : base.friction;
  soft.damping = has('damping') ? atLeastZero(caller, 'damping', given.damping) : base.damping;
  soft.wind = Float64Array.from(has('wind') ? vec3(caller, 'wind', given.wind) : base.wind);
  soft.layers = has('layers') ? bits(caller, 'layers', given.layers) : base.layers;
  soft.mask = has('mask') ? bits(caller, 'mask', given.mask) : base.mask;
  soft.position = Float64Array.from(has('position') ? vec3(caller, 'position', given.position) : base.position);
  soft.rotation = Float64Array.from(has('rotation') ? quat(caller, 'rotation', given.rotation) : base.rotation);
}

/** A number from 0 to 1, as a body's restitution is checked. */
function fraction(caller, name, value) {
  if (typeof value !== 'number' || !(value >= 0 && value <= 1)) fail(caller, `${name} must be from 0 to 1, got ${show(value)}`);
  return value;
}

/** A finite number, 0 or more, as a body's friction is checked. */
function atLeastZero(caller, name, value) {
  if (typeof value !== 'number' || !(value >= 0) || value === Infinity) fail(caller, `${name} must be 0 or more, got ${show(value)}`);
  return value;
}

/** 32 bits, as a body's layers and mask take them (World's bits()). */
function bits(caller, name, value) {
  if (!Number.isInteger(value) || value < -0x80000000 || value > 0xffffffff) {
    fail(caller, `${name} must be a whole number that fits in 32 bits, got ${show(value)}`);
  }
  return value >>> 0;
}

/** A mesh's corners and triangles, as addBody checks a mesh's: the same messages. */
function vertexList(caller, value) {
  const rule = 'vertices must be finite numbers, three to a vertex';
  if (!(Array.isArray(value) || ArrayBuffer.isView(value))) fail(caller, `${rule}, got ${show(value)}`);
  if (value.length % 3 !== 0) fail(caller, `${rule}, got ${value.length} numbers`);
  for (let i = 0; i < value.length; i++) {
    if (typeof value[i] !== 'number' || !Number.isFinite(value[i])) fail(caller, `${rule}, got ${show(value[i])} at index ${i}`);
  }
  return value;
}

/**
 * The mesh's triangles, edges, folds and whether it is closed, onto soft.
 * Closed: every edge between exactly two triangles, one going each way round
 * it, so the triangles are wound alike and enclose a volume.
 */
function topology(caller, soft, vertices, indices) {
  const n = vertices.length / 3;
  // At least one triangle: a soft body is its triangles.
  const rule = `indices must be whole numbers below the vertex count (${n}), three to a triangle, one triangle or more`;
  if (!(Array.isArray(indices) || ArrayBuffer.isView(indices))) fail(caller, `${rule}, got ${show(indices)}`);
  if (indices.length === 0 || indices.length % 3 !== 0) fail(caller, `${rule}, got ${indices.length} numbers`);
  for (let i = 0; i < indices.length; i++) {
    const k = indices[i];
    if (!Number.isInteger(k) || k < 0 || k >= n) fail(caller, `${rule}, got ${show(k)} at index ${i}`);
  }
  for (let t = 0; t < indices.length; t += 3) {
    const a = indices[t], b = indices[t + 1], c = indices[t + 2];
    if (a === b || b === c || a === c) fail(caller, `triangle ${t / 3} names a vertex twice: ${a},${b},${c}`);
  }
  soft.vertices = Float64Array.from(vertices);
  soft.indices = Int32Array.from(indices);
  soft.n = n;
  // Edges in the order the triangles first name them; each with the corner
  // opposite it in its first triangle, then the fold across to its second's.
  const edgeOf = new Map(), edges = [], opposite = [], sides = [], bends = [];
  let directed = true;
  const forward = new Set();
  for (let t = 0; t < indices.length; t += 3) {
    for (let e = 0; e < 3; e++) {
      const a = indices[t + e], b = indices[t + (e + 1) % 3], c = indices[t + (e + 2) % 3];
      const key = a < b ? a * n + b : b * n + a;
      if (forward.has(a * n + b)) directed = false;
      forward.add(a * n + b);
      let k = edgeOf.get(key);
      if (k === undefined) {
        k = edges.length / 2;
        edgeOf.set(key, k);
        edges.push(a, b);
        opposite.push(c);
        sides.push(1);
      } else {
        sides[k]++;
        if (sides[k] === 2) bends.push(opposite[k], c);
      }
    }
  }
  let closed = directed;
  for (const s of sides) if (s !== 2) closed = false;
  const used = new Uint8Array(n);
  for (let i = 0; i < indices.length; i++) used[indices[i]] = 1;
  for (let i = 0; i < n; i++) if (used[i] === 0) fail(caller, `vertex ${i} is in no triangle`);
  soft.edges = Int32Array.from(edges);
  soft.bends = Int32Array.from(bends);
  soft.closed = closed ? 1 : 0;
  soft.x = new Float64Array(3 * n); soft.v = new Float64Array(3 * n); soft.prev = new Float64Array(3 * n);
  soft.rest = new Float64Array(3 * n); soft.grad = new Float64Array(3 * n); soft.positions = new Float32Array(3 * n);
  soft.w = new Float64Array(n); soft.pressed = new Float64Array(6 * n); soft.pressedBy = new Int32Array(2 * n); soft.blocked = new Float64Array(2 * n); soft.pinned = new Float64Array(n).fill(-2);
  soft.hit = new Int32Array(PLANES * n).fill(-1); soft.hitId = new Float64Array(PLANES * n).fill(-1);
  soft.hitPoint = new Float64Array(3 * PLANES * n); soft.hitNormal = new Float64Array(3 * PLANES * n); soft.near = new Float64Array(n);
  soft.length = new Float64Array(edges.length / 2); soft.give = new Float64Array(edges.length / 2);
  soft.across = new Float64Array(bends.length / 2); soft.bendGive = new Float64Array(bends.length / 2);
  soft.box = new Float64Array(6);
  soft.attachVertex = new Int32Array(0); soft.attachBody = new Float64Array(0); soft.attachLocal = new Float64Array(0);
  soft.attached = 0; soft.anchors = new Float64Array(0);
  soft.tether = new Int32Array(n).fill(-1); soft.reach = new Float64Array(n);
  const v = soft.vertices;
  for (let t = 0; t < indices.length; t += 3) {
    if (area(v, indices[t], indices[t + 1], indices[t + 2]) === 0) fail(caller, `triangle ${t / 3} has no area: its corners are in a line`);
  }
}

/** Triangle (a, b, c) of corners v: its area, m². */
function area(v, a, b, c) {
  const a3 = 3 * a, b3 = 3 * b, c3 = 3 * c;
  const ux = v[b3] - v[a3], uy = v[b3 + 1] - v[a3 + 1], uz = v[b3 + 2] - v[a3 + 2];
  const wx = v[c3] - v[a3], wy = v[c3 + 1] - v[a3 + 1], wz = v[c3 + 2] - v[a3 + 2];
  const nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
  return Math.sqrt(nx * nx + ny * ny + nz * nz) / 2;
}

/** Six times the signed volume the triangles of corners v enclose, from corner 0. */
function volume6(v, idx) {
  let six = 0;
  const ox = v[0], oy = v[1], oz = v[2];
  for (let t = 0; t < idx.length; t += 3) {
    const a = 3 * idx[t], b = 3 * idx[t + 1], c = 3 * idx[t + 2];
    const ax = v[a] - ox, ay = v[a + 1] - oy, az = v[a + 2] - oz;
    const bx = v[b] - ox, by = v[b + 1] - oy, bz = v[b + 2] - oz;
    const cx = v[c] - ox, cy = v[c + 1] - oy, cz = v[c + 2] - oz;
    six += ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx);
  }
  return six;
}

/**
 * Each corner's inverse mass, and every constraint's rest length and give,
 * from the shape as given. Cloth weighs its density times its area times its
 * thickness; a closed mesh its density times its volume. Each triangle's
 * share goes a third to each corner, by area.
 */
function masses(soft) {
  const v = soft.vertices, idx = soft.indices, n = soft.n;
  const share = new Float64Array(n);
  let total = 0;
  for (let t = 0; t < idx.length; t += 3) {
    const a = area(v, idx[t], idx[t + 1], idx[t + 2]);
    total += a;
    share[idx[t]] += a / 3; share[idx[t + 1]] += a / 3; share[idx[t + 2]] += a / 3;
  }
  soft.volume = soft.closed === 1 ? volume6(v, idx) / 6 : 0;
  const mass = soft.closed === 1 ? soft.density * Math.abs(soft.volume) : soft.density * soft.thickness * total;
  soft.mass = mass;
  for (let i = 0; i < n; i++) soft.w[i] = total / (mass * share[i]);
  let lo0 = Infinity, lo1 = Infinity, lo2 = Infinity, hi0 = -Infinity, hi1 = -Infinity, hi2 = -Infinity;
  for (let i = 0; i < n; i++) {
    const x = v[3 * i], y = v[3 * i + 1], z = v[3 * i + 2];
    if (x < lo0) lo0 = x; if (x > hi0) hi0 = x;
    if (y < lo1) lo1 = y; if (y > hi1) hi1 = y;
    if (z < lo2) lo2 = z; if (z > hi2) hi2 = z;
  }
  soft.size = Math.max(hi0 - lo0, hi1 - lo1, hi2 - lo2);
  const rho = mass / total, L = soft.size, E = soft.edges, B = soft.bends;
  // The area beside each edge: the triangles it borders.
  const beside = new Float64Array(E.length / 2), key = new Map();
  for (let k = 0; k < E.length; k += 2) key.set(E[k] < E[k + 1] ? E[k] * n + E[k + 1] : E[k + 1] * n + E[k], k / 2);
  for (let t = 0; t < idx.length; t += 3) {
    const a = area(v, idx[t], idx[t + 1], idx[t + 2]);
    for (let e = 0; e < 3; e++) {
      const p = idx[t + e], q = idx[t + (e + 1) % 3];
      beside[key.get(p < q ? p * n + q : q * n + p)] += a;
    }
  }
  let spacing = 0;
  for (let k = 0; k < E.length / 2; k++) {
    const l = distance(v, E[2 * k], E[2 * k + 1]);
    soft.length[k] = l;
    soft.give[k] = compliance(soft.stretch, rho, L, l, beside[k]);
    spacing += l;
  }
  soft.spacing = spacing / (E.length / 2);
  // A fold's area: its two triangles, which share the edge it crosses.
  const foldArea = new Float64Array(B.length / 2);
  {
    let b = 0;
    const seen = new Map();
    for (let t = 0; t < idx.length; t += 3) {
      for (let e = 0; e < 3; e++) {
        const p = idx[t + e], q = idx[t + (e + 1) % 3], k = p < q ? p * n + q : q * n + p;
        const a = area(v, idx[t], idx[t + 1], idx[t + 2]);
        if (seen.has(k)) { const first = seen.get(k); if (first >= 0) { foldArea[b++] = first + a; seen.set(k, -1); } }
        else seen.set(k, a);
      }
    }
  }
  for (let k = 0; k < B.length / 2; k++) {
    const l = distance(v, B[2 * k], B[2 * k + 1]);
    soft.across[k] = l;
    soft.bendGive[k] = compliance(soft.bend, rho, L, l, foldArea[k]);
  }
}

function distance(v, a, b) {
  const x = v[3 * b] - v[3 * a], y = v[3 * b + 1] - v[3 * a + 1], z = v[3 * b + 2] - v[3 * a + 2];
  return Math.sqrt(x * x + y * y + z * z);
}

/** The corners at the soft body's pose, all moving at `velocity`, at rest and awake. */
function place(soft, velocity) {
  const v = soft.vertices, q = soft.rotation, p = soft.position;
  for (let i = 0; i < soft.n; i++) {
    rotate(q, 0, v[3 * i], v[3 * i + 1], v[3 * i + 2], false, V);
    soft.x[3 * i] = p[0] + V[0]; soft.x[3 * i + 1] = p[1] + V[1]; soft.x[3 * i + 2] = p[2] + V[2];
    soft.v[3 * i] = velocity[0]; soft.v[3 * i + 1] = velocity[1]; soft.v[3 * i + 2] = velocity[2];
  }
  wakeSoft(soft);
  copyOut(soft);
}

/** Moved by hand from pose (p0, q0) to its own: every corner, and its pins to the world, as one. */
function teleport(soft, p0, q0) {
  const move = (a, o) => {
    rotate(q0, 0, a[o] - p0[0], a[o + 1] - p0[1], a[o + 2] - p0[2], true, V);
    rotate(soft.rotation, 0, V[0], V[1], V[2], false, V);
    a[o] = soft.position[0] + V[0]; a[o + 1] = soft.position[1] + V[1]; a[o + 2] = soft.position[2] + V[2];
  };
  soft.x = Float64Array.from(soft.x);
  for (let i = 0; i < soft.n; i++) move(soft.x, 3 * i);
  soft.attachLocal = Float64Array.from(soft.attachLocal);
  for (let k = 0; k < soft.attached; k++) if (soft.attachBody[k] < 0) move(soft.attachLocal, 3 * k);
}

function checkAttach(world, caller, soft, list) {
  if (!Array.isArray(list)) fail(caller, `attach must be a list of { vertex, body }, got ${show(list)}`);
  for (let k = 0; k < list.length; k++) {
    const a = options(caller, list[k], `attach[${k}]`);
    known(`${caller}: attach[${k}]`, a, ATTACH_OPTIONS);
    if (!Number.isInteger(a.vertex) || a.vertex < 0 || a.vertex >= soft.n) {
      fail(caller, `attach[${k}].vertex must be a whole number below the vertex count (${soft.n}), got ${show(a.vertex)}`);
    }
    if (a.body !== undefined && a.body !== null) handle(caller, `attach[${k}].body`, a.body, world, Body, world._slots);
  }
}

/** Each corner listed held where it is now, to its body or the world. */
function attach(world, caller, soft, list) {
  checkAttach(world, caller, soft, list);
  const m = list.length;
  soft.attachVertex = new Int32Array(m); soft.attachBody = new Float64Array(m); soft.attachLocal = new Float64Array(3 * m);
  soft.attached = m;
  soft.pinned.fill(-2);
  const p = world._position, q = world._rotation;
  for (let k = 0; k < m; k++) {
    const i = list[k].vertex, body = list[k].body, i3 = 3 * i, k3 = 3 * k;
    soft.attachVertex[k] = i;
    if (body === undefined || body === null) {
      soft.attachBody[k] = -1;
      soft.attachLocal[k3] = soft.x[i3]; soft.attachLocal[k3 + 1] = soft.x[i3 + 1]; soft.attachLocal[k3 + 2] = soft.x[i3 + 2];
    } else {
      const s = world._slots.slotOf(body.id), s3 = 3 * s;
      soft.attachBody[k] = body.id;
      rotate(q, 4 * s, soft.x[i3] - p[s3], soft.x[i3 + 1] - p[s3 + 1], soft.x[i3 + 2] - p[s3 + 2], true, V);
      soft.attachLocal[k3] = V[0]; soft.attachLocal[k3 + 1] = V[1]; soft.attachLocal[k3 + 2] = V[2];
    }
    soft.pinned[i] = soft.attachBody[k];
  }
  tethers(world, soft);
}

/**
 * TETHERS (long-range attachments: Kim, Chentanez and Müller, 2012; Jolt's
 * soft bodies have them too). One pass of the edges a substep moves a pull
 * along the mesh only so far: a cloth hung by its top edge, 20 edges long,
 * hung 1% longer than it is at 8 substeps, and 4% at 40 edges. So each
 * corner is also kept within its distance along the mesh, at rest, of the
 * nearest attachment, divided by `stretch`: it can't hang further from it
 * than the whole length between could stretch (STIFFNESS: (1 - s) / s more
 * at the top, the most anywhere). Only ever pulling a corner in, so a cloth
 * draped round something, further from its pin in a straight line than
 * along itself, never feels it. Distances by Dijkstra's method over the edges.
 */
function tethers(world, soft) {
  const n = soft.n, E = soft.edges, len = soft.length, tie = soft.tether, reach = soft.reach;
  tie.fill(-1);
  reach.fill(Infinity);
  soft.anchors = new Float64Array(3 * soft.attached);
  if (soft.attached === 0) { reach.fill(0); return; }
  // Each corner's edges, for the walk.
  const start = new Int32Array(n + 1), to = new Int32Array(E.length), along = new Float64Array(E.length);
  for (let e = 0; e < E.length; e++) start[E[e] + 1]++;
  for (let i = 0; i < n; i++) start[i + 1] += start[i];
  const fill = start.slice(0, n);
  for (let k = 0; k < E.length / 2; k++) {
    const a = E[2 * k], b = E[2 * k + 1];
    to[fill[a]] = b; along[fill[a]++] = len[k];
    to[fill[b]] = a; along[fill[b]++] = len[k];
  }
  // A binary heap of (distance, corner), the nearest on top, ties to the lower corner.
  const hd = [], hi = [];
  const less = (x, y) => hd[x] < hd[y] || (hd[x] === hd[y] && hi[x] < hi[y]);
  const swap = (x, y) => { [hd[x], hd[y]] = [hd[y], hd[x]]; [hi[x], hi[y]] = [hi[y], hi[x]]; };
  const add = (d, i) => {
    let c = hd.length;
    hd.push(d); hi.push(i);
    while (c > 0 && less(c, (c - 1) >> 1)) { swap(c, (c - 1) >> 1); c = (c - 1) >> 1; }
  };
  const take = () => {
    const d = hd[0], i = hi[0], ld = hd.pop(), li = hi.pop();
    if (hd.length > 0) {
      hd[0] = ld; hi[0] = li;
      for (let c = 0; ;) {
        const l = 2 * c + 1, r = l + 1;
        let m = c;
        if (l < hd.length && less(l, m)) m = l;
        if (r < hd.length && less(r, m)) m = r;
        if (m === c) break;
        swap(m, c);
        c = m;
      }
    }
    return [d, i];
  };
  // From what the cloth's weight doesn't move: the world, a static or a
  // kinematic body. A dynamic body hangs from the cloth as anything else does.
  for (let k = 0; k < soft.attached; k++) {
    const i = soft.attachVertex[k], id = soft.attachBody[k];
    if (reach[i] === 0 || (id >= 0 && world._type[world._slots.slotOf(id)] === DYNAMIC)) continue;
    reach[i] = 0; tie[i] = k;
    add(0, i);
  }
  while (hd.length > 0) {
    const [d, i] = take();
    if (d > reach[i]) continue;
    for (let e = start[i]; e < start[i + 1]; e++) {
      const j = to[e], dj = d + along[e];
      if (dj < reach[j]) { reach[j] = dj; tie[j] = tie[i]; add(dj, j); }
    }
  }
  // A corner held where it is has its own hold, and a piece joined to no such attachment hangs from nothing.
  for (let i = 0; i < n; i++) if (reach[i] === 0 || reach[i] === Infinity) { tie[i] = -1; reach[i] = 0; }
}

function wakeSoft(soft) {
  soft.asleep = 0;
  soft.still = 0;
  soft.rest.set(soft.x);
}

/** The corners into the public Float32Array. */
function copyOut(soft) {
  const x = soft.x, out = soft.positions;
  for (let i = 0; i < x.length; i++) out[i] = x[i];
}

/** Wakes every soft body (world.sleeping turned off). */
export function wakeSofts(world) {
  for (const soft of world._softs.list) if (soft !== null && soft !== undefined) wakeSoft(soft);
}

// ---- the step ----

// Module scratch: the step isn't re-entered, and it allocates nothing (rule 76).
const V = new Float64Array(3), U = new Float64Array(3), W = new Float64Array(3), OUT = new Float64Array(4);
/** What this substep's constraints turned the soft body by against the corners' starts, times h (SPIN). */
const SPIN = new Float64Array(3);
const BALL = new Shape();
BALL.kind = SPHERE;
const IDENTITY = new Float64Array([0, 0, 0, 1]);
let found = new Int32Array(16), foundCount = 0;

function collect(s) {
  if (foundCount === found.length) found = resized(found, 2 * found.length);
  found[foundCount++] = s;
}

/** Whether a soft body with these layers and mask collides with slot s, both ways, as two bodies do. */
function collides(world, soft, s) {
  return world._sensor[s] === 0 && (soft.layers & world._mask[s]) !== 0 && (world._layers[s] & soft.mask) !== 0;
}

/** Whether slot s has moved since the last step's sleep check, or is gone (id), so what rests on it should wake. */
function stirred(world, s, id) {
  if (s < 0 || world._slots.slotOf(id) !== s) return true;
  if (world._type[s] === STATIC) return world._staticMoved[s] === 1;
  return world._still[s] === 0;
}

/**
 * Before the narrowphase, once a step: each awake soft body's nearest rigid
 * body for each corner, as a plane in that body's frame (COLLISIONS). And
 * each asleep one woken if what it touches, hangs from or overlaps moved.
 */
export function findSoft(world, dt) {
  const list = world._softs.list, g = world._gravity, gl = Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
  const trees = world._broadphase.trees;
  for (let k = 0; k < list.length; k++) {
    const soft = list[k];
    if (soft === null || soft === undefined) continue;
    if (soft.asleep === 1) {
      if (!disturbed(world, soft)) continue;
      wakeSoft(soft);
    }
    const n = soft.n, x = soft.x, v = soft.v, r = soft.thickness * 0.5, near = soft.near;
    // How far each corner can go this step, its box grown by that and its radius.
    let x0 = Infinity, y0 = Infinity, z0 = Infinity, x1 = -Infinity, y1 = -Infinity, z1 = -Infinity, most = 0;
    for (let i = 0; i < n; i++) {
      const i3 = 3 * i, sp = Math.sqrt(v[i3] * v[i3] + v[i3 + 1] * v[i3 + 1] + v[i3 + 2] * v[i3 + 2]);
      const reach = (sp + gl * dt) * dt + r;
      near[i] = reach;
      if (reach > most) most = reach;
      if (x[i3] < x0) x0 = x[i3]; if (x[i3] > x1) x1 = x[i3];
      if (x[i3 + 1] < y0) y0 = x[i3 + 1]; if (x[i3 + 1] > y1) y1 = x[i3 + 1];
      if (x[i3 + 2] < z0) z0 = x[i3 + 2]; if (x[i3 + 2] > z1) z1 = x[i3 + 2];
    }
    const box = soft.box;
    box[0] = x0 - most; box[1] = y0 - most; box[2] = z0 - most; box[3] = x1 + most; box[4] = y1 + most; box[5] = z1 + most;
    foundCount = 0;
    for (let t = 0; t < trees.length; t++) trees[t].query(box[0], box[1], box[2], box[3], box[4], box[5], collect);
    // prev is free until the substeps: the separations of the planes found so far.
    const hit = soft.hit, hitId = soft.hitId, sep = soft.prev, P = soft.hitPoint, N = soft.hitNormal;
    for (let c = 0; c < PLANES * n; c++) { sep[c] = Infinity; hit[c] = -1; }
    const B = world._box, motion = world._motion, size = world._size, pos = world._position, rot = world._rotation;
    for (let k = 0; k < soft.attached; k++) if (soft.attachBody[k] >= 0) inverseOf(world, world._slots.slotOf(soft.attachBody[k]));
    for (let f = 0; f < foundCount; f++) {
      const s = found[f];
      if (!collides(world, soft, s)) continue;
      inverseOf(world, s);
      const id = world._slots.id(s), b6 = 6 * s, s3 = 3 * s, s4 = 4 * s;
      // The smaller size, as any contact's tolerance; a mesh has none of its own.
      const tolerance = (size[s] > 0 ? Math.min(size[s], 2 * r) : 2 * r) * PIXEL;
      for (let i = 0; i < n; i++) {
        if (soft.pinned[i] === id) continue;
        const i3 = 3 * i, reach = near[i] + motion[s] + tolerance;
        if (x[i3] + reach < B[b6] || x[i3] - reach > B[b6 + 3] || x[i3 + 1] + reach < B[b6 + 1] || x[i3 + 1] - reach > B[b6 + 4]
          || x[i3 + 2] + reach < B[b6 + 2] || x[i3 + 2] - reach > B[b6 + 5]) continue;
        BALL.place(x[i3], x[i3 + 1], x[i3 + 2], IDENTITY, 0);
        BALL.half[0] = r;
        const c0 = PLANES * i;
        if (!nearestOf(world, s, BALL, reach - r, tolerance, OUT) || !(OUT[0] < sep[c0 + 1])) continue;
        // Kept nearest first: the second moves down if this is nearer than the first.
        let c = c0 + 1;
        if (OUT[0] < sep[c0]) {
          sep[c] = sep[c0]; hit[c] = hit[c0];
          for (let e = 0; e < 3; e++) { P[3 * c + e] = P[3 * c0 + e]; N[3 * c + e] = N[3 * c0 + e]; }
          c = c0;
        }
        sep[c] = OUT[0];
        hit[c] = s;
        // The point on the body, and its outward normal, in its own frame; a
        // static body's in the world's, as it doesn't move, so the substeps
        // needn't turn them.
        const d = r + OUT[0], nx = OUT[1], ny = OUT[2], nz = OUT[3], c3 = 3 * c;
        if (world._type[s] === STATIC) {
          P[c3] = x[i3] + nx * d; P[c3 + 1] = x[i3 + 1] + ny * d; P[c3 + 2] = x[i3 + 2] + nz * d;
          N[c3] = -nx; N[c3 + 1] = -ny; N[c3 + 2] = -nz;
          continue;
        }
        rotate(rot, s4, x[i3] + nx * d - pos[s3], x[i3 + 1] + ny * d - pos[s3 + 1], x[i3 + 2] + nz * d - pos[s3 + 2], true, V);
        P[c3] = V[0]; P[c3 + 1] = V[1]; P[c3 + 2] = V[2];
        rotate(rot, s4, -nx, -ny, -nz, true, V);
        N[c3] = V[0]; N[c3 + 1] = V[1]; N[c3 + 2] = V[2];
      }
    }
    // A corner newly on a sleeping or frozen body wakes it, as a body would.
    for (let i = 0; i < n; i++) {
      const c0 = PLANES * i, was0 = hitId[c0], was1 = hitId[c0 + 1];
      for (let c = c0; c < c0 + PLANES; c++) {
        const s = hit[c];
        if (s < 0) { hitId[c] = -1; continue; }
        const id = world._slots.id(s);
        if (id !== was0 && id !== was1 && (world._island[s] !== 0 || world._frozen[s] !== 0)) world._wake(s);
        hitId[c] = id;
      }
    }
  }
}

/** Whether an asleep soft body should wake: what it hangs from or touches moved, or something not still is in its box. */
function disturbed(world, soft) {
  for (let k = 0; k < soft.attached; k++) {
    const id = soft.attachBody[k];
    if (id >= 0 && stirred(world, world._slots.slotOf(id), id)) return true;
  }
  for (let c = 0; c < soft.hit.length; c++) if (soft.hit[c] >= 0 && stirred(world, soft.hit[c], soft.hitId[c])) return true;
  const b = soft.box, tree = world._broadphase.trees[1];
  foundCount = 0;
  tree.query(b[0], b[1], b[2], b[3], b[4], b[5], collect);
  for (let f = 0; f < foundCount; f++) {
    const s = found[f];
    if (world._still[s] === 0 && collides(world, soft, s)) return true;
  }
  return false;
}

/**
 * By slot, the world inverse inertia (xx xy xz yy yz zz) of each dynamic body
 * a soft body touches or hangs from, as the step began (findSoft): what a
 * push on it turns it by, worked out once a step, as the contacts' is.
 */
let INVERSE = new Float64Array(0);

function inverseOf(world, s) {
  if (INVERSE.length < 6 * world._type.length) INVERSE = resized(INVERSE, 6 * world._type.length);
  if (world._type[s] === DYNAMIC) turnedInverse(world, s, INVERSE, 6 * s);
}

/** How much a push along unit (ux, uy, uz) at (rx, ry, rz) from slot s's centre moves that point: its inverse mass there. */
function inverseAt(world, s, rx, ry, rz, ux, uy, uz) {
  if (world._type[s] !== DYNAMIC || world._awake[s] === 0 || world._frozen[s] !== 0) return 0;
  const inv = world._invMass[s];
  if (inv === 0) return 0;
  const lock = world._lockMove[s], I = INVERSE, o = 6 * s;
  const lin = inv * (((lock & 1) !== 0 ? 0 : ux * ux) + ((lock & 2) !== 0 ? 0 : uy * uy) + ((lock & 4) !== 0 ? 0 : uz * uz));
  const tx = ry * uz - rz * uy, ty = rz * ux - rx * uz, tz = rx * uy - ry * ux;
  return lin + tx * (I[o] * tx + I[o + 1] * ty + I[o + 2] * tz) + ty * (I[o + 1] * tx + I[o + 3] * ty + I[o + 4] * tz)
    + tz * (I[o + 2] * tx + I[o + 4] * ty + I[o + 5] * tz);
}

/** An impulse (jx, jy, jz) on slot s at (rx, ry, rz) from its centre. */
function push(world, s, rx, ry, rz, jx, jy, jz) {
  const inv = world._invMass[s], lock = world._lockMove[s], v = world._velocity, w = world._angular, s3 = 3 * s, I = INVERSE, o = 6 * s;
  if ((lock & 1) === 0) v[s3] += jx * inv;
  if ((lock & 2) === 0) v[s3 + 1] += jy * inv;
  if ((lock & 4) === 0) v[s3 + 2] += jz * inv;
  const tx = ry * jz - rz * jy, ty = rz * jx - rx * jz, tz = rx * jy - ry * jx;
  w[s3] += I[o] * tx + I[o + 1] * ty + I[o + 2] * tz;
  w[s3 + 1] += I[o + 1] * tx + I[o + 3] * ty + I[o + 4] * tz;
  w[s3 + 2] += I[o + 2] * tx + I[o + 4] * ty + I[o + 5] * tz;
}

/**
 * Where point `local` of slot s will be at the end of a substep h, into V,
 * and its offset from the body's centre now into U: carried by the body's
 * velocity, as it will move.
 */
function ahead(world, s, l, o, h) {
  const p = world._position, w = world._angular, v = world._velocity, s3 = 3 * s;
  rotate(world._rotation, 4 * s, l[o], l[o + 1], l[o + 2], false, U);
  const rx = U[0], ry = U[1], rz = U[2];
  V[0] = p[s3] + rx + (v[s3] + w[s3 + 1] * rz - w[s3 + 2] * ry) * h;
  V[1] = p[s3 + 1] + ry + (v[s3 + 1] + w[s3 + 2] * rx - w[s3] * rz) * h;
  V[2] = p[s3 + 2] + rz + (v[s3 + 2] + w[s3] * ry - w[s3 + 1] * rx) * h;
}

/** One substep h of every awake soft body: forces, motion, constraints, velocities. */
export function stepSoft(world, h, first, dt) {
  const list = world._softs.list, g = world._gravity;
  for (let k = 0; k < list.length; k++) {
    const soft = list[k];
    if (soft === null || soft === undefined || soft.asleep === 1) continue;
    const n = soft.n, x = soft.x, v = soft.v, w = soft.w, prev = soft.prev;
    // The air once a step, for the whole step: what it pushes with changes
    // as the cloth turns and moves, a few times a second as a flag flaps, not
    // within a step; once a substep it was a third of a cloth's step time.
    if (first) air(soft, dt);
    if (first) settle(soft, SETTLE, dt);
    const gx = g[0] * h, gy = g[1] * h, gz = g[2] * h;
    const damp = soft.damping === 0 ? 1 : 1 / (1 + h * soft.damping);
    for (let i = 0; i < n; i++) {
      const i3 = 3 * i;
      v[i3] = (v[i3] + gx) * damp; v[i3 + 1] = (v[i3 + 1] + gy) * damp; v[i3 + 2] = (v[i3 + 2] + gz) * damp;
      prev[i3] = x[i3]; prev[i3 + 1] = x[i3 + 1]; prev[i3 + 2] = x[i3 + 2];
      x[i3] += v[i3] * h; x[i3 + 1] += v[i3 + 1] * h; x[i3 + 2] += v[i3 + 2] * h;
    }
    const h2 = h * h;
    SPIN[0] = 0; SPIN[1] = 0; SPIN[2] = 0;
    if (soft.stretch > 0) lengths(x, prev, w, soft.edges, soft.length, soft.give, h2, true);
    if (soft.bend > 0) lengths(x, prev, w, soft.bends, soft.across, soft.bendGive, h2, false);
    if (soft.attached > 0) {
      anchor(world, soft, h);
      if (soft.stretch > 0) tether(soft);
    }
    contacts(world, soft, h);
    // After the contacts, and never into what they pushed off (hold()).
    // The volume moves corners the contacts didn't push, some of them onto
    // something; they are held out of it once more, as the step's last word
    // (friction had its say the first time).
    if (soft.pressure > 0) { hold(world, soft, h); contacts(world, soft, h, true); }
    if (soft.attached > 0) attachments(world, soft, h);
    const inv = 1 / h;
    for (let i = 0; i < 3 * n; i++) v[i] = (x[i] - prev[i]) * inv;
    unbounce(world, soft);
    unspin(soft, inv);
  }
}

/**
 * SPIN. Forces within a body can't turn it: each pair of an edge is equal
 * and opposite along the line between, and a closed volume's push sums to
 * no turn. Each constraint moved alone keeps that for positions, but the
 * velocity a substep reads back is measured from where the corners began it,
 * and against that every move turns the body by m (start × move): for an
 * edge, λ (start_a - start_b) × d, d the edge as it is now, which the
 * substep has turned. In a body that moves as one it averages away; in one
 * held still under load, where the edges and the volume put each other
 * right every substep, it doesn't: a ball at rest on a frictionless floor
 * spun about the vertical faster and faster, 0.3 rad/s more each second. So
 * the constraints add up the turn they made (SPIN, ×h), and the substep's
 * velocities lose exactly that much spin about the centre of mass, as a turn
 * of the whole: its momentum and every other turn left as they were.
 */
function unspin(soft, inv) {
  const lx = SPIN[0] * inv, ly = SPIN[1] * inv, lz = SPIN[2] * inv;
  if (lx === 0 && ly === 0 && lz === 0) return;
  const x = soft.x, v = soft.v, w = soft.w, n = soft.n;
  let m = 0, cx = 0, cy = 0, cz = 0;
  for (let i = 0; i < n; i++) {
    const mi = 1 / w[i], i3 = 3 * i;
    m += mi; cx += mi * x[i3]; cy += mi * x[i3 + 1]; cz += mi * x[i3 + 2];
  }
  cx /= m; cy /= m; cz /= m;
  let ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
  for (let i = 0; i < n; i++) {
    const mi = 1 / w[i], i3 = 3 * i, rx = x[i3] - cx, ry = x[i3 + 1] - cy, rz = x[i3 + 2] - cz;
    ixx += mi * (ry * ry + rz * rz); iyy += mi * (rx * rx + rz * rz); izz += mi * (rx * rx + ry * ry);
    ixy -= mi * rx * ry; ixz -= mi * rx * rz; iyz -= mi * ry * rz;
  }
  if (!spinOf(ixx, iyy, izz, ixy, ixz, iyz, lx, ly, lz)) return;
  const ox = W[0], oy = W[1], oz = W[2];
  for (let i = 0; i < n; i++) {
    const i3 = 3 * i, rx = x[i3] - cx, ry = x[i3 + 1] - cy, rz = x[i3 + 2] - cz;
    v[i3] -= oy * rz - oz * ry; v[i3 + 1] -= oz * rx - ox * rz; v[i3 + 2] -= ox * ry - oy * rx;
  }
}

/** ω = I⁻¹ L, into W, by cofactors; false if I is singular, a body all in a line. */
function spinOf(ixx, iyy, izz, ixy, ixz, iyz, lx, ly, lz) {
  const a = iyy * izz - iyz * iyz, b = ixz * iyz - ixy * izz, c = ixy * iyz - ixz * iyy;
  const det = ixx * a + ixy * b + ixz * c;
  if (!(det > 0)) return false;
  const d = ixx * izz - ixz * ixz, e = ixy * ixz - ixx * iyz, f = ixx * iyy - ixy * ixy;
  W[0] = (a * lx + b * ly + c * lz) / det; W[1] = (b * lx + d * ly + e * lz) / det; W[2] = (c * lx + e * ly + f * lz) / det;
  return true;
}

/** Each pair (a, b) of `pairs` back toward its rest length, giving by its compliance. */
function lengths(x, prev, w, pairs, rest, give, h2, longer) {
  let tx = 0, ty = 0, tz = 0;
  for (let k = 0, e = 0; k < rest.length; k++, e += 2) {
    const a = pairs[e], b = pairs[e + 1], a3 = 3 * a, b3 = 3 * b;
    const dx = x[b3] - x[a3], dy = x[b3 + 1] - x[a3 + 1], dz = x[b3 + 2] - x[a3 + 2];
    const l = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (l === 0 || (longer && l <= rest[k])) continue;
    const wa = w[a], wb = w[b], sum = wa + wb + give[k] / h2;
    if (sum === 0) continue;
    const lambda = (l - rest[k]) / (l * sum);
    x[a3] += wa * lambda * dx; x[a3 + 1] += wa * lambda * dy; x[a3 + 2] += wa * lambda * dz;
    x[b3] -= wb * lambda * dx; x[b3 + 1] -= wb * lambda * dy; x[b3 + 2] -= wb * lambda * dz;
    // Its turn against the starts (SPIN): λ (start_a - start_b) × d.
    const px = prev[a3] - prev[b3], py = prev[a3 + 1] - prev[b3 + 1], pz = prev[a3 + 2] - prev[b3 + 2];
    tx += lambda * (py * dz - pz * dy); ty += lambda * (pz * dx - px * dz); tz += lambda * (px * dy - py * dx);
  }
  SPIN[0] += tx; SPIN[1] += ty; SPIN[2] += tz;
}

/**
 * The volume back to pressure times its rest volume, exactly: one
 * constraint over every corner. Its gradient at a corner is a sixth of the
 * cross product (twice the area along the normal) of each triangle round it.
 */
function hold(world, soft, h) {
  const x = soft.x, idx = soft.indices, grad = soft.grad, n = soft.n, w = soft.w, pressed = soft.pressed, blocked = soft.blocked;
  grad.fill(0);
  const six = volume6(x, idx);
  for (let t = 0; t < idx.length; t += 3) {
    const a = 3 * idx[t], b = 3 * idx[t + 1], c = 3 * idx[t + 2];
    const ux = x[b] - x[a], uy = x[b + 1] - x[a + 1], uz = x[b + 2] - x[a + 2];
    const vx = x[c] - x[a], vy = x[c + 1] - x[a + 1], vz = x[c + 2] - x[a + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    grad[a] += nx; grad[a + 1] += ny; grad[a + 2] += nz;
    grad[b] += nx; grad[b + 1] += ny; grad[b + 2] += nz;
    grad[c] += nx; grad[c + 1] += ny; grad[c + 2] += nz;
  }
  // A corner a contact pushed off a body this substep isn't moved back into
  // it: the body bears that part of the push, along its normal, as a floor
  // holds up what stands on it. Moved into the floor anyway, each substep's
  // contacts pushed the bottom of a ball back up, the volume was always short
  // when the next one began, and the push it was put back with, always the
  // same way, turned a ball at rest about the vertical, faster and faster.
  const more = six < 6 * soft.pressure * soft.volume;   // the push is then along +grad, outward
  // Its turn against the starts, before contacts bend it (SPIN): λ Σ start × grad.
  const prev = soft.prev;
  let tx = 0, ty = 0, tz = 0;
  for (let i = 0; i < n; i++) {
    const i3 = 3 * i, px = prev[i3], py = prev[i3 + 1], pz = prev[i3 + 2];
    tx += py * grad[i3 + 2] - pz * grad[i3 + 1]; ty += pz * grad[i3] - px * grad[i3 + 2]; tz += px * grad[i3 + 1] - py * grad[i3];
  }
  let any = false;
  for (let i = 0; i < n; i++) {
    const i3 = 3 * i, i6 = 6 * i;
    blocked[2 * i] = 0; blocked[2 * i + 1] = 0;
    for (let j = i6, b = 2 * i; j < i6 + 6; j += 3, b++) {
      const ox = pressed[j], oy = pressed[j + 1], oz = pressed[j + 2];
      if (ox === 0 && oy === 0 && oz === 0) break;
      const d = grad[i3] * ox + grad[i3 + 1] * oy + grad[i3 + 2] * oz;
      if (more ? d < 0 : d > 0) { grad[i3] -= d * ox; grad[i3 + 1] -= d * oy; grad[i3 + 2] -= d * oz; blocked[b] = d; any = true; }
    }
  }
  // With C = V - p V0 = (six - 6 p V0) / 6 and its gradient grad / 6, each
  // corner moves by w (-C / sum of w |grad / 6|²) grad / 6: the sixes cancel to this.
  let sum = 0;
  for (let i = 0; i < n; i++) {
    const i3 = 3 * i;
    sum += w[i] * (grad[i3] * grad[i3] + grad[i3 + 1] * grad[i3 + 1] + grad[i3 + 2] * grad[i3 + 2]);
  }
  if (sum === 0) return;
  const lambda = -(six - 6 * soft.pressure * soft.volume) / sum;
  SPIN[0] += lambda * tx; SPIN[1] += lambda * ty; SPIN[2] += lambda * tz;
  for (let i = 0; i < n; i++) {
    const i3 = 3 * i, k = w[i] * lambda;
    x[i3] += k * grad[i3]; x[i3 + 1] += k * grad[i3 + 1]; x[i3 + 2] += k * grad[i3 + 2];
  }
  if (!any) return;
  // What a moving body bore of the push: the momentum the corner would have
  // been moved by into it, m w λ d / h along its normal, so none is lost.
  const by = soft.pressedBy, p = world._position;
  for (let b = 0; b < 2 * n; b++) {
    const d = blocked[b];
    if (d === 0) continue;
    const s = by[b], i3 = 3 * (b >> 1), j = 3 * b, s3 = 3 * s;
    const rx = x[i3] - p[s3], ry = x[i3 + 1] - p[s3 + 1], rz = x[i3 + 2] - p[s3 + 2];
    if (inverseAt(world, s, rx, ry, rz, pressed[j], pressed[j + 1], pressed[j + 2]) === 0) continue;
    const k = lambda * d / h;
    push(world, s, rx, ry, rz, k * pressed[j], k * pressed[j + 1], k * pressed[j + 2]);
  }
}

/**
 * Each corner out of its plane by its radius, with friction, the body
 * pushed back: the plane where the body will be at the substep's end.
 * Friction takes the corner's slide along the plane, relative to the body's
 * surface, all of it while that is within friction times how far the plane
 * pushed (holding), else that much of it (sliding), Coulomb's rule in
 * positions (Müller et al., "Detailed Rigid Body Simulation with Extended
 * Position Based Dynamics", 2020).
 */
function contacts(world, soft, h, again = false) {
  const n = soft.n, x = soft.x, w = soft.w, prev = soft.prev, r = soft.thickness * 0.5;
  const hit = soft.hit, P = soft.hitPoint, N = soft.hitNormal, rot = world._rotation, pos = world._position;
  const fr = world._friction, type = world._type, pressed = soft.pressed;
  if (!again) pressed.fill(0);
  for (let c = 0; c < PLANES * n; c++) {
    const s = hit[c];
    if (s < 0) continue;
    const i = c >> 1, i3 = 3 * i, s3 = 3 * s, c3 = 3 * c;
    if (type[s] === STATIC) {
      // Its plane is in the world's frame (findSoft), and nothing moves it.
      const gap = (x[i3] - P[c3]) * N[c3] + (x[i3 + 1] - P[c3 + 1]) * N[c3 + 1] + (x[i3 + 2] - P[c3 + 2]) * N[c3 + 2] - r;
      if (gap >= 0) continue;
      const ox = N[c3], oy = N[c3 + 1], oz = N[c3 + 2];
      press(pressed, soft.pressedBy, i, ox, oy, oz, s);
      x[i3] -= gap * ox; x[i3 + 1] -= gap * oy; x[i3 + 2] -= gap * oz;
      const mu = again ? 0 : Math.sqrt(soft.friction * fr[s]);
      if (mu === 0) continue;
      let dx = x[i3] - prev[i3], dy = x[i3 + 1] - prev[i3 + 1], dz = x[i3 + 2] - prev[i3 + 2];
      const along = dx * ox + dy * oy + dz * oz;
      dx -= along * ox; dy -= along * oy; dz -= along * oz;
      const slide = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (slide === 0) continue;
      const f = Math.min(slide, mu * -gap) / slide;
      x[i3] -= f * dx; x[i3 + 1] -= f * dy; x[i3 + 2] -= f * dz;
      continue;
    }
    ahead(world, s, P, c3, h);
    const px = V[0], py = V[1], pz = V[2], rx = U[0], ry = U[1], rz = U[2];
    rotate(rot, 4 * s, N[c3], N[c3 + 1], N[c3 + 2], false, V);
    const ox = V[0], oy = V[1], oz = V[2];
    const gap = (x[i3] - px) * ox + (x[i3 + 1] - py) * oy + (x[i3 + 2] - pz) * oz - r;
    if (gap >= 0) continue;
    press(pressed, soft.pressedBy, i, ox, oy, oz, s);
    const wi = w[i], wb = inverseAt(world, s, rx, ry, rz, ox, oy, oz);
    const lambda = -gap / (wi + wb);
    x[i3] += wi * lambda * ox; x[i3 + 1] += wi * lambda * oy; x[i3 + 2] += wi * lambda * oz;
    if (wb > 0) push(world, s, rx, ry, rz, -lambda * ox / h, -lambda * oy / h, -lambda * oz / h);
    // The slide: the corner's move over the substep less the surface's, along the plane.
    const mu = again ? 0 : Math.sqrt(soft.friction * fr[s]);
    if (mu === 0) continue;
    const vb = world._velocity, wv = world._angular;
    const sx = (vb[s3] + wv[s3 + 1] * rz - wv[s3 + 2] * ry) * h, sy = (vb[s3 + 1] + wv[s3 + 2] * rx - wv[s3] * rz) * h, sz = (vb[s3 + 2] + wv[s3] * ry - wv[s3 + 1] * rx) * h;
    let dx = x[i3] - prev[i3] - sx, dy = x[i3 + 1] - prev[i3 + 1] - sy, dz = x[i3 + 2] - prev[i3 + 2] - sz;
    const along = dx * ox + dy * oy + dz * oz;
    dx -= along * ox; dy -= along * oy; dz -= along * oz;
    const slide = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (slide === 0) continue;
    const ux = dx / slide, uy = dy / slide, uz = dz / slide;
    const wt = wb > 0 ? inverseAt(world, s, rx, ry, rz, ux, uy, uz) : 0;
    // Pushed by `lambda (wi + wb)` in all, the plane's whole push; friction takes up to mu times it.
    const taken = Math.min(slide, mu * -gap);
    const l = taken / (wi + wt);
    x[i3] -= wi * l * ux; x[i3 + 1] -= wi * l * uy; x[i3 + 2] -= wi * l * uz;
    if (wt > 0) push(world, s, rx, ry, rz, l * ux / h, l * uy / h, l * uz / h);
  }
}

/** Corner i was pushed off a body along unit (ox, oy, oz): kept in the first of its two places that is empty. */
function press(pressed, by, i, ox, oy, oz, s) {
  const first = pressed[6 * i] === 0 && pressed[6 * i + 1] === 0 && pressed[6 * i + 2] === 0;
  const j = first ? 6 * i : 6 * i + 3;
  pressed[j] = ox; pressed[j + 1] = oy; pressed[j + 2] = oz;
  by[first ? 2 * i : 2 * i + 1] = s;
}

/**
 * After the velocities are read back: each corner a contact pushed this
 * substep leaves the body's surface no faster than it met it, which with no
 * restitution is not at all. The push that put it back out moved it as far
 * as it was in, however it got there, and read back as velocity that is a
 * bounce every substep: a corner pushed in by the volume or its edges came
 * off at the speed of the push, and soft bodies at rest on a floor rocked
 * and wobbled for good. This is XPBD's velocity pass (Macklin et al.,
 * "Detailed Rigid Body Simulation with Extended Position Based Dynamics",
 * 2020, 3.6). A moving body shares it, by inverse masses, as it shared the push.
 */
function unbounce(world, soft) {
  const v = soft.v, x = soft.x, w = soft.w, pressed = soft.pressed, by = soft.pressedBy, n = soft.n;
  const bv = world._velocity, bw = world._angular, p = world._position;
  for (let i = 0; i < n; i++) {
    const i3 = 3 * i;
    for (let k = 0; k < 2; k++) {
      const j = 6 * i + 3 * k, ox = pressed[j], oy = pressed[j + 1], oz = pressed[j + 2];
      if (ox === 0 && oy === 0 && oz === 0) break;
      const s = by[2 * i + k], s3 = 3 * s;
      let ux = v[i3], uy = v[i3 + 1], uz = v[i3 + 2], rx = 0, ry = 0, rz = 0, wb = 0;
      if (world._type[s] !== STATIC) {
        rx = x[i3] - p[s3]; ry = x[i3 + 1] - p[s3 + 1]; rz = x[i3 + 2] - p[s3 + 2];
        ux -= bv[s3] + bw[s3 + 1] * rz - bw[s3 + 2] * ry;
        uy -= bv[s3 + 1] + bw[s3 + 2] * rx - bw[s3] * rz;
        uz -= bv[s3 + 2] + bw[s3] * ry - bw[s3 + 1] * rx;
        wb = inverseAt(world, s, rx, ry, rz, ox, oy, oz);
      }
      const off = ux * ox + uy * oy + uz * oz;
      if (off <= 0) continue;
      const wi = w[i], l = off / (wi + wb);
      v[i3] -= wi * l * ox; v[i3 + 1] -= wi * l * oy; v[i3 + 2] -= wi * l * oz;
      if (wb > 0) push(world, s, rx, ry, rz, l * ox, l * oy, l * oz);
    }
  }
}

/** Where each attachment's point will be at the end of the substep, into soft.anchors. */
function anchor(world, soft, h) {
  const L = soft.attachLocal, A = soft.anchors;
  for (let k = 0; k < soft.attached; k++) {
    const id = soft.attachBody[k], k3 = 3 * k;
    if (id < 0) { A[k3] = L[k3]; A[k3 + 1] = L[k3 + 1]; A[k3 + 2] = L[k3 + 2]; continue; }
    ahead(world, world._slots.slotOf(id), L, k3, h);
    A[k3] = V[0]; A[k3 + 1] = V[1]; A[k3 + 2] = V[2];
  }
}

/** Each corner pulled in to within its tether of its nearest attachment (TETHERS). */
function tether(soft) {
  const x = soft.x, tie = soft.tether, reach = soft.reach, A = soft.anchors, s = soft.stretch;
  for (let i = 0; i < soft.n; i++) {
    const k = tie[i];
    if (k < 0) continue;
    const i3 = 3 * i, k3 = 3 * k;
    const dx = x[i3] - A[k3], dy = x[i3 + 1] - A[k3 + 1], dz = x[i3 + 2] - A[k3 + 2];
    const most = reach[i] / s, d2 = dx * dx + dy * dy + dz * dz;
    if (d2 <= most * most) continue;
    const f = most / Math.sqrt(d2);
    x[i3] = A[k3] + dx * f; x[i3 + 1] = A[k3 + 1] + dy * f; x[i3 + 2] = A[k3 + 2] + dz * f;
  }
}

/**
 * Each attached corner to its attachment's point, exactly: held in the
 * world, or shared with its body by their inverse masses there, the body
 * pulled back by the same impulse.
 */
function attachments(world, soft, h) {
  const x = soft.x, w = soft.w, A = soft.anchors, L = soft.attachLocal;
  for (let k = 0; k < soft.attached; k++) {
    const i = soft.attachVertex[k], id = soft.attachBody[k], i3 = 3 * i, k3 = 3 * k;
    if (id < 0) { x[i3] = A[k3]; x[i3 + 1] = A[k3 + 1]; x[i3 + 2] = A[k3 + 2]; continue; }
    const dx = x[i3] - A[k3], dy = x[i3 + 1] - A[k3 + 1], dz = x[i3 + 2] - A[k3 + 2], l = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (l === 0) continue;
    const s = world._slots.slotOf(id);
    // From the body's centre as it is now, as ahead() measures it.
    rotate(world._rotation, 4 * s, L[k3], L[k3 + 1], L[k3 + 2], false, U);
    const rx = U[0], ry = U[1], rz = U[2], ux = dx / l, uy = dy / l, uz = dz / l;
    const wi = w[i], wb = inverseAt(world, s, rx, ry, rz, ux, uy, uz);
    const lambda = -l / (wi + wb);
    x[i3] += wi * lambda * ux; x[i3 + 1] += wi * lambda * uy; x[i3 + 2] += wi * lambda * uz;
    if (wb > 0) push(world, s, rx, ry, rz, -lambda * ux / h, -lambda * uy / h, -lambda * uz / h);
  }
}

/**
 * Each corner's velocity drawn toward the soft body's own motion as a rigid
 * body, a velocity and a spin about its centre of mass, by rate `rate` (1/s),
 * implicitly: its wobble and sway die away, and how it falls, slides or
 * spins is left as it is (Müller et al., "Position Based Dynamics", 2007).
 */
function settle(soft, rate, dt) {
  const x = soft.x, v = soft.v, w = soft.w, n = soft.n;
  let m = 0, cx = 0, cy = 0, cz = 0, vx = 0, vy = 0, vz = 0;
  for (let i = 0; i < n; i++) {
    const mi = 1 / w[i], i3 = 3 * i;
    m += mi; cx += mi * x[i3]; cy += mi * x[i3 + 1]; cz += mi * x[i3 + 2];
    vx += mi * v[i3]; vy += mi * v[i3 + 1]; vz += mi * v[i3 + 2];
  }
  cx /= m; cy /= m; cz /= m; vx /= m; vy /= m; vz /= m;
  // Its angular momentum about the centre, and its inertia there.
  let lx = 0, ly = 0, lz = 0, ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
  for (let i = 0; i < n; i++) {
    const mi = 1 / w[i], i3 = 3 * i, rx = x[i3] - cx, ry = x[i3 + 1] - cy, rz = x[i3 + 2] - cz;
    const ux = v[i3], uy = v[i3 + 1], uz = v[i3 + 2];
    lx += mi * (ry * uz - rz * uy); ly += mi * (rz * ux - rx * uz); lz += mi * (rx * uy - ry * ux);
    ixx += mi * (ry * ry + rz * rz); iyy += mi * (rx * rx + rz * rz); izz += mi * (rx * rx + ry * ry);
    ixy -= mi * rx * ry; ixz -= mi * rx * rz; iyz -= mi * ry * rz;
  }
  // Its spin; a body all in a line has no inertia about it, and then no spin is kept.
  let ox = 0, oy = 0, oz = 0;
  if (spinOf(ixx, iyy, izz, ixy, ixz, iyz, lx, ly, lz)) { ox = W[0]; oy = W[1]; oz = W[2]; }
  const k = 1 - 1 / (1 + rate * dt);
  for (let i = 0; i < n; i++) {
    const i3 = 3 * i, rx = x[i3] - cx, ry = x[i3 + 1] - cy, rz = x[i3 + 2] - cz;
    v[i3] += k * (vx + oy * rz - oz * ry - v[i3]);
    v[i3 + 1] += k * (vy + oz * rx - ox * rz - v[i3 + 1]);
    v[i3 + 2] += k * (vz + ox * ry - oy * rx - v[i3 + 2]);
  }
}

/**
 * The air on each triangle: the pressure of a flat plate, ½ ρ C A |u| u
 * along its normal, u the wind's speed through it along the normal, a third
 * to each corner. Taken implicitly, as a body's damping is, so it can slow a
 * light triangle to the wind's speed but never past it, however fast.
 */
function air(soft, h) {
  const x = soft.x, v = soft.v, w = soft.w, idx = soft.indices, wind = soft.wind;
  const k0 = 0.5 * AIR * DRAG * h;
  for (let t = 0; t < idx.length; t += 3) {
    const a = idx[t], b = idx[t + 1], c = idx[t + 2], a3 = 3 * a, b3 = 3 * b, c3 = 3 * c;
    const ux = x[b3] - x[a3], uy = x[b3 + 1] - x[a3 + 1], uz = x[b3 + 2] - x[a3 + 2];
    const vx = x[c3] - x[a3], vy = x[c3 + 1] - x[a3 + 1], vz = x[c3 + 2] - x[a3 + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const twice = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (twice === 0) continue;
    const inv = 1 / twice, mx = nx * inv, my = ny * inv, mz = nz * inv;
    const rx = wind[0] - (v[a3] + v[b3] + v[c3]) * THIRD, ry = wind[1] - (v[a3 + 1] + v[b3 + 1] + v[c3 + 1]) * THIRD, rz = wind[2] - (v[a3 + 2] + v[b3 + 2] + v[c3 + 2]) * THIRD;
    const un = rx * mx + ry * my + rz * mz;
    if (un === 0) continue;
    const k = k0 * twice * 0.5 * Math.abs(un);
    const j = k * un * THIRD / (1 + k * (w[a] + w[b] + w[c]) * NINTH);
    v[a3] += j * w[a] * mx; v[a3 + 1] += j * w[a] * my; v[a3 + 2] += j * w[a] * mz;
    v[b3] += j * w[b] * mx; v[b3 + 1] += j * w[b] * my; v[b3 + 2] += j * w[b] * mz;
    v[c3] += j * w[c] * mx; v[c3 + 1] += j * w[c] * my; v[c3 + 2] += j * w[c] * mz;
  }
}

/**
 * After the substeps: the corners into `positions`, and sleep, by the rule
 * bodies sleep by (World._sleep): still while no corner strays a pixel of
 * the mesh's spacing from where it rested, for SLEEP_TIME, the rest pose
 * taken again each whole SLEEP_TIME.
 */
export function endSoft(world, dt) {
  const list = world._softs.list, sleeping = world._sleeping;
  for (let k = 0; k < list.length; k++) {
    const soft = list[k];
    if (soft === null || soft === undefined || soft.asleep === 1) continue;
    copyOut(soft);
    if (!sleeping) continue;
    const x = soft.x, rest = soft.rest, tolerance = soft.size * PIXEL, t2 = tolerance * tolerance;
    let still = true;
    for (let i = 0; i < x.length && still; i += 3) {
      const dx = x[i] - rest[i], dy = x[i + 1] - rest[i + 1], dz = x[i + 2] - rest[i + 2];
      if (dx * dx + dy * dy + dz * dz >= t2) still = false;
    }
    if (!still) { soft.still = 0; rest.set(x); continue; }
    const before = soft.still;
    soft.still += dt;
    if (Math.floor(soft.still / SLEEP_TIME) !== Math.floor(before / SLEEP_TIME)) rest.set(x);
    if (soft.still >= SLEEP_TIME) {
      soft.asleep = 1;
      soft.v.fill(0);
    }
  }
}
