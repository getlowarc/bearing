// Cylinders (milestone 26): their mass, their contacts against every shape
// held to a brute-force count of points inside both, standing on an end,
// lying on a side, rolling, on meshes and heightfields, as a car's wheels,
// rays, water, compounds, queries and cloth.
// Run: node test/cylinder.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { Shape, Manifold, collide, SPHERE, CAPSULE, BOX, HULL, CYLINDER } from '../src/collide.js';
import { build } from '../src/hull.js';
import { submerged, UNDER } from '../src/fluid.js';
import { sheet } from './scenes.js';
import { suite, test } from './test.js';

suite('cylinder');

const steps = (world, seconds, each) => {
  for (let i = 0; i < Math.round(seconds * 60); i++) { world.step(1 / 60); if (each) each(i); }
};
/** Lying on its side: its own y along the world's x. */
const SIDE = [0, 0, Math.SQRT1_2, Math.SQRT1_2];
const numbers = (seed) => () => (seed = (seed * 16807) % 2147483647) / 2147483647;

test('a cylinder weighs its density times πr²h and turns as one: m r²/2 about its axis, m (h²/12 + r²/4) across', () => {
  const world = new World();
  const c = world.addBody({ shape: 'cylinder', radius: 0.3, height: 0.8, density: 500 });
  const m = 500 * Math.PI * 0.09 * 0.8;
  assert.ok(Math.abs(c.mass / m - 1) < 1e-12);
  const [ix, iy, iz] = c.inertia;
  assert.ok(Math.abs(iy / (m * 0.09 / 2) - 1) < 1e-12);
  assert.ok(Math.abs(ix / (m * (0.64 / 12 + 0.09 / 4)) - 1) < 1e-12 && ix === iz);
  // Sliced into rings and summed, so the formula isn't checking itself.
  let sum = 0;
  const N = 400;
  for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
    const rr = 0.3 * (i + 0.5) / N, y = 0.8 * ((j + 0.5) / N - 0.5), dm = 500 * 2 * Math.PI * rr * (0.3 / N) * (0.8 / N);
    sum += dm * (y * y + rr * rr / 2);   // about x: y² plus half of r² averaged round the ring
  }
  assert.ok(Math.abs(sum / ix - 1) < 1e-4);
  // A compound part, and moved off its centre.
  const wheel = world.addBody({ shape: 'compound', parts: [{ shape: 'cylinder', radius: 0.3, height: 0.2, rotation: SIDE }, { size: [0.1, 0.1, 0.1], position: [0, 0.5, 0] }] });
  assert.ok(Math.abs(wheel.mass - 1000 * (Math.PI * 0.09 * 0.2 + 0.001)) < 1e-9, `the compound weighs ${wheel.mass}`);
  world.addBody({ shape: 'cylinder', centerOfMass: [0, -0.2, 0] });
});

/** Whether point p is inside placed shape s by more than eps. */
function inside(s, x, y, z, eps) {
  const c = s.center, A = s.axes, h = s.half, dx = x - c[0], dy = y - c[1], dz = z - c[2];
  const lx = A[0] * dx + A[1] * dy + A[2] * dz, ly = A[3] * dx + A[4] * dy + A[5] * dz, lz = A[6] * dx + A[7] * dy + A[8] * dz;
  if (s.kind === SPHERE) return Math.hypot(lx, ly, lz) < h[0] - eps;
  if (s.kind === CAPSULE) { const cy = Math.max(-h[1], Math.min(h[1], ly)); return Math.hypot(lx, ly - cy, lz) < h[0] - eps; }
  if (s.kind === BOX) return Math.abs(lx) < h[0] - eps && Math.abs(ly) < h[1] - eps && Math.abs(lz) < h[2] - eps;
  if (s.kind === CYLINDER) return Math.hypot(lx, lz) < h[0] - eps && Math.abs(ly) < h[1] - eps;
  const p = s.hull.planes;
  for (let f = 0; f < p.length; f += 4) if (p[f] * lx + p[f + 1] * ly + p[f + 2] * lz - p[f + 3] > -eps) return false;
  return true;
}

/** How deep b's points reach into a, sampled: the most any is inside both, by depth along nothing in particular; true if any. */
function overlap(a, b, next, eps) {
  // Points inside a, tested against b.
  const r = Math.max(a.half[0], a.half[1], a.half[2]) + 0.5;
  for (let k = 0; k < 6000; k++) {
    const x = a.center[0] + r * (2 * next() - 1), y = a.center[1] + r * (2 * next() - 1), z = a.center[2] + r * (2 * next() - 1);
    if (inside(a, x, y, z, eps) && inside(b, x, y, z, eps)) return true;
  }
  return false;
}

const ROCK = build([0, 0.4, 0, 0.4, -0.2, 0.1, -0.3, -0.2, 0.3, -0.1, -0.2, -0.4, 0.2, 0.1, -0.2, 0.1, 0.3, 0.3]);

test('against every shape, a cylinder found touching overlaps it, and pushed apart along the normal by its depth, no longer does', () => {
  const next = numbers(11), out = new Manifold();
  const make = (kind) => {
    const s = new Shape();
    s.kind = kind;
    if (kind === SPHERE) s.half[0] = 0.2 + 0.3 * next();
    else if (kind === CAPSULE || kind === CYLINDER) { s.half[0] = 0.1 + 0.3 * next(); s.half[1] = 0.05 + 0.4 * next(); }
    else if (kind === BOX) { s.half[0] = 0.1 + 0.4 * next(); s.half[1] = 0.1 + 0.4 * next(); s.half[2] = 0.1 + 0.4 * next(); }
    else s.hull = ROCK;
    const q = [next() - 0.5, next() - 0.5, next() - 0.5, next() - 0.5], l = Math.hypot(...q);
    s.turn = q.map((x) => x / l);
    s.place(0.8 * (next() - 0.5), 0.8 * (next() - 0.5), 0.8 * (next() - 0.5), s.turn);
    return s;
  };
  let met = 0, missed = 0, stuck = 0;
  for (const kind of [SPHERE, CAPSULE, BOX, HULL, CYLINDER]) {
    for (let n = 0; n < 300; n++) {
      const a = make(kind), b = make(CYLINDER), tol = 0.002;
      const count = collide(a, b, 0, tol, out);
      const deep = count > 0 ? Math.min(...out.separations.slice(0, count)) : Infinity;
      const overlaps = overlap(a, b, next, 0.01) || overlap(b, a, next, 0.01);
      if (overlaps) { met++; if (!(count > 0 && deep < 0)) missed++; }
      if (count > 0 && deep < -0.02) {
        // Moved along the normal by its depth and a little, b clears a.
        const d = -deep + 0.01, n2 = out.normal;
        b.place(b.center[0] + n2[0] * d, b.center[1] + n2[1] * d, b.center[2] + n2[2] * d, b.turn);
        if (overlap(a, b, next, 0.01) && overlap(b, a, next, 0.01)) stuck++;
      }
    }
  }
  assert.ok(met > 300, `only ${met} overlapping poses`);
  assert.equal(missed, 0, `${missed} overlaps found no contact`);
  assert.ok(stuck <= 3, `${stuck} still overlapping once moved apart`);
});

test('a cylinder stands on its end, lies on its side and carries others, at rest where it should be, and sleeps', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  const standing = world.addBody({ shape: 'cylinder', radius: 0.3, height: 0.6, position: [0, 0.4, 0] });
  const lying = world.addBody({ shape: 'cylinder', radius: 0.3, height: 0.6, position: [2, 0.4, 0], rotation: SIDE });
  const stack = [0, 1, 2].map((k) => world.addBody({ shape: 'cylinder', radius: 0.25, height: 0.3, position: [-2, 0.16 + 0.31 * k, 0] }));
  const ball = world.addBody({ shape: 'sphere', radius: 0.15, position: [0, 1, 0] });
  const box = world.addBody({ size: [0.3, 0.3, 0.3], position: [2, 1, 0] });
  steps(world, 6);
  const tol = 0.003;
  assert.ok(Math.abs(standing.position[1] - 0.3) < tol && Math.abs(lying.position[1] - 0.3) < tol);
  stack.forEach((c, k) => assert.ok(Math.abs(c.position[1] - (0.15 + 0.3 * k)) < tol && Math.abs(c.position[0] + 2) < tol, `stacked ${k} at ${c.position}`));
  assert.ok(Math.abs(ball.position[1] - 0.75) < tol && Math.abs(box.position[1] - 0.75) < tol);
  for (const b of [standing, lying, ...stack, ball, box]) assert.ok(b.asleep);
});

test('a wheel rolls at its spin times its radius, on a box, a heightfield and a mesh, never leaving the ground', () => {
  for (const ground of ['box', 'heightfield', 'mesh']) {
    for (const [speed, width] of [[5, 0.25], [20, 0.25], [20, 0.5]]) {
      const world = new World();
      if (ground === 'box') world.addBody({ type: 'static', size: [400, 1, 400], position: [0, -0.5, 0] });
      else if (ground === 'heightfield') world.addBody({ shape: 'heightfield', heights: new Array(121 * 121).fill(0), columns: 121, spacing: 1 });
      else world.addBody({ shape: 'mesh', ...sheet(60, 60, 120, 120) });
      const wheel = world.addBody({ shape: 'cylinder', radius: 0.35, height: width, rotation: SIDE, density: 300, position: [0.3, 0.35, -40], velocity: [0, 0, speed], angularVelocity: [speed / 0.35, 0, 0] });
      let lifted = 0, bounce = 0;
      steps(world, 2, () => { if (world.touching(wheel).length === 0) lifted++; bounce = Math.max(bounce, Math.abs(wheel.velocity[1])); });
      assert.equal(lifted, 0, `${ground}, ${speed} m/s, ${width} m wide: off the ground`);
      assert.ok(bounce < 1e-3, `${ground}, ${speed} m/s: bounced at ${bounce} m/s`);
      // Slowed only by rolling resistance, 0.01 of its weight: about 0.2 m/s in 2 s, with its spin to slow too.
      assert.ok(wheel.velocity[2] > speed - 0.25 && wheel.velocity[2] < speed, `${ground}: ${wheel.velocity[2]} m/s`);
      assert.ok(Math.abs(wheel.angularVelocity[0] * 0.35 - wheel.velocity[2]) < 0.01, 'slipping');
    }
  }
});

test('cylinders dropped on rough terrain, mesh and heightfield, none through it', () => {
  const N = 30, ground = (x, z) => 0.02 * (x * x + z * z) + 0.25 * Math.sin(1.3 * x) * Math.cos(0.9 * z);
  const next = numbers(3);
  for (const level of ['mesh', 'heightfield']) {
    let through = 0, asleep = 0, total = 0;
    for (let run = 0; run < 2; run++) {
      const world = new World();
      const v = [], idx = [], heights = [];
      for (let j = 0; j <= N; j++) for (let i = 0; i <= N; i++) { v.push(i - N / 2, ground(i - N / 2, j - N / 2), j - N / 2); heights.push(ground(i - N / 2, j - N / 2)); }
      for (let j = 0; j < N; j++) for (let i = 0; i < N; i++) { const a = j * (N + 1) + i; idx.push(a, a + N + 1, a + 1, a + 1, a + N + 1, a + N + 2); }
      world.addBody(level === 'mesh' ? { shape: 'mesh', vertices: v, indices: idx } : { shape: 'heightfield', heights, columns: N + 1, spacing: 1 });
      const bodies = [];
      for (let i = 0; i < 25; i++) {
        const q = [next() - 0.5, next() - 0.5, next() - 0.5, next() - 0.5];
        bodies.push(world.addBody({ shape: 'cylinder', radius: 0.15 + 0.3 * next(), height: 0.1 + 0.6 * next(), position: [8 * next() - 4, 4 + 3 * next(), 8 * next() - 4], rotation: q, velocity: [2 * next() - 1, 0, 2 * next() - 1] }));
      }
      steps(world, 20);
      for (const b of bodies) {
        total++;
        const [x, y, z] = b.position;
        if (ground(x, z) - y > 0.05) through++;
        if (b.asleep) asleep++;
      }
    }
    assert.equal(through, 0, `${level}: ${through} of ${total} through`);
    // Round things roll about a bowl for a while; most come to rest.
    assert.ok(asleep >= 0.7 * total, `${level}: ${asleep} of ${total} asleep at 20 s`);
  }
});

test('a car on cylinder wheels drives, turns hard and crosses a heightfield at 20 m/s on the ground', () => {
  for (const level of ['box', 'heightfield']) {
    const world = new World();
    world.addBody(level === 'box' ? { type: 'static', size: [400, 1, 400], position: [0, -0.5, 0] } : { shape: 'heightfield', heights: new Array(161 * 161).fill(0), columns: 161, spacing: 1 });
    const chassis = world.addBody({ size: [1.6, 0.4, 3.6], density: 300, position: [0.3, 0.75, -45] });
    const wheels = [], joints = [];
    for (const [x, z] of [[-0.85, 1.3], [0.85, 1.3], [-0.85, -1.3], [0.85, -1.3]]) {
      const p = [0.3 + x, 0.35, -45 + z];
      wheels.push(world.addBody({ shape: 'cylinder', radius: 0.35, height: 0.25, rotation: SIDE, density: 600, friction: 1, position: p }));
      joints.push(world.addJoint({ type: 'wheel', a: wheels.at(-1), b: chassis, anchor: p, axis: [0, 1, 0], frequency: 3, dampingRatio: 0.7, min: -0.2, max: 0.2 }));
    }
    steps(world, 3);
    assert.ok(chassis.asleep, `${level}: parked, awake`);
    for (const j of joints) world.setJoint(j, { motorSpeed: 20 / 0.35, motorForce: 1500 });
    let lifted = 0;
    steps(world, 4.5, () => { if (wheels.some((w) => world.touching(w).length === 0)) lifted++; });
    assert.ok(Math.abs(chassis.velocity[2] - 20) < 0.1, `${level}: ${chassis.velocity[2]} m/s`);
    assert.equal(lifted, 0, `${level}: a wheel left the ground`);
    if (level !== 'box') continue;
    // Steered hard at 20 m/s: it turns, its wheels on the ground and its chassis upright.
    let lean = 0;
    steps(world, 3, () => {
      const [x, y, z, w] = chassis.rotation, s = 0.4;
      const X = [Math.cos(s), 0, -Math.sin(s)], tx = 2 * (y * X[2] - z * X[1]), ty = 2 * (z * X[0] - x * X[2]), tz = 2 * (x * X[1] - y * X[0]);
      const axle = [X[0] + w * tx + (y * tz - z * ty), X[1] + w * ty + (z * tx - x * tz), X[2] + w * tz + (x * ty - y * tx)];
      for (const j of joints.slice(0, 2)) world.setJoint(j, { axle });
      lean = Math.max(lean, Math.acos(Math.min(1, 1 - 2 * (x * x + z * z))));
    });
    assert.ok(lean < 0.2, `the chassis leaned ${lean} rad`);
    assert.ok(Math.abs(chassis.velocity[0]) > 1, 'it didn\'t turn');
  }
});

test('a ray meets a cylinder where its side or ends are, its normal theirs', () => {
  const world = new World();
  world.addBody({ type: 'static', shape: 'cylinder', radius: 0.5, height: 1, position: [1, 2, 3], rotation: [0.3, 0.1, 0.2, 1] });
  const next = numbers(9);
  let hits = 0;
  for (let i = 0; i < 3000; i++) {
    const o = [1 + 4 * next() - 2, 2 + 4 * next() - 2, 3 + 4 * next() - 2], d = [next() - 0.5, next() - 0.5, next() - 0.5];
    const hit = world.raycast(o, d);
    const l = Math.hypot(...d), u = d.map((x) => x / l);
    // Against the same cylinder as a shape, probed by overlap just past and just short of the hit.
    if (hit === null || hit.distance === 0) continue;
    hits++;
    const at = (t) => [o[0] + u[0] * t, o[1] + u[1] * t, o[2] + u[2] * t];
    assert.equal(world.overlap({ shape: 'sphere', radius: 1e-4 }, at(hit.distance - 2e-3)).length, 0, 'something before the hit');
    assert.equal(world.overlap({ shape: 'sphere', radius: 1e-4 }, at(hit.distance + 2e-3)).length > 0 || Math.abs(u[0] * hit.normal[0] + u[1] * hit.normal[1] + u[2] * hit.normal[2]) < 0.05, true, 'nothing past the hit');
    assert.ok(u[0] * hit.normal[0] + u[1] * hit.normal[1] + u[2] * hit.normal[2] < 0, 'the normal faces the ray');
  }
  assert.ok(hits > 100);
});

test('in water a cylinder floats as Archimedes says', () => {
  // Exact cases: lying half under, its centre of buoyancy 4r/3π below its axis; upright.
  const s = new Shape();
  s.kind = CYLINDER; s.half[0] = 0.5; s.half[1] = 1;
  s.place(0, 0, 0, SIDE);
  UNDER.fill(0); submerged(s, 0, 1, 0, 0, UNDER);
  assert.ok(Math.abs(UNDER[0] - Math.PI * 0.25) < 1e-12 && Math.abs(UNDER[2] / UNDER[0] + 2 / (3 * Math.PI)) < 1e-12);
  s.place(0, 0, 0, [0, 0, 0, 1]);
  UNDER.fill(0); submerged(s, 0, 1, 0, 0.3, UNDER);
  assert.ok(Math.abs(UNDER[0] - Math.PI * 0.25 * 1.3) < 1e-12 && Math.abs(UNDER[2] / UNDER[0] + 0.35) < 1e-12);
  // Any pose: against a count of points, to the count's own error.
  const next = numbers(4);
  for (let n = 0; n < 40; n++) {
    s.half[0] = 0.2 + 0.5 * next(); s.half[1] = 0.1 + 0.6 * next();
    const q = [next() - 0.5, next() - 0.5, next() - 0.5, next() - 0.5], l = Math.hypot(...q);
    s.place(0, 0, 0, q.map((x) => x / l));
    const c = (next() - 0.5) * 1.2, r = s.half[0], h = s.half[1], A = s.axes, M = 80;
    UNDER.fill(0); submerged(s, 0, 1, 0, c, UNDER);
    let vol = 0;
    const dv = (2 * r / M) * (2 * r / M) * (2 * h / M);
    for (let i = 0; i < M; i++) for (let j = 0; j < M; j++) {
      const x = -r + (i + 0.5) * 2 * r / M, z = -r + (j + 0.5) * 2 * r / M;
      if (x * x + z * z > r * r) continue;
      for (let k = 0; k < M; k++) { const y = -h + (k + 0.5) * 2 * h / M; if (A[1] * x + A[4] * y + A[7] * z < c) vol += dv; }
    }
    assert.ok(Math.abs(UNDER[0] - vol) < 0.002 * Math.PI * r * r * 2 * h, `${UNDER[0]} against ${vol}`);
  }
  // A log of half water's density floats on its side with its axis at the surface, and sleeps.
  const world = new World();
  world.addBody({ type: 'static', fluid: true, size: [20, 4, 20], position: [0, -2, 0] });
  const log = world.addBody({ shape: 'cylinder', radius: 0.3, height: 2, density: 500, rotation: SIDE, position: [0, 1, 0] });
  steps(world, 20);
  assert.ok(Math.abs(log.position[1]) < 0.002, `its axis at ${log.position[1]}`);
  assert.ok(log.asleep);
});

test('queries, characters and cloth meet cylinders; a character can\'t be one', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  const drum = world.addBody({ type: 'static', shape: 'cylinder', radius: 0.5, height: 1, position: [0, 0.5, 0] });
  assert.equal(world.overlap({ shape: 'sphere', radius: 0.1 }, [0.55, 0.5, 0])[0].id, drum.id);
  assert.equal(world.overlap({ shape: 'sphere', radius: 0.1 }, [0.65, 0.5, 0]).length, 0);
  const cast = world.shapeCast({ size: [0.2, 0.2, 0.2] }, [3, 0.5, 0], [-1, 0, 0]);
  assert.equal(cast.body.id, drum.id);
  assert.ok(Math.abs(cast.distance - 2.4) < 0.01, `cast ${cast.distance}`);
  const walker = world.addBody({ type: 'character', shape: 'capsule', radius: 0.3, height: 1.8, position: [3, 0.9, 0], velocity: [-2, 0, 0] });
  steps(world, 2, () => walker.setVelocity(-2, walker.velocity[1], 0));
  assert.ok(walker.position[0] > 0.79, `the character walked into the drum to ${walker.position[0]}`);
  const cloth = world.addSoftBody({ ...sheet(16, 16, 2, 2), position: [0, 1.5, 0] });
  steps(world, 6);
  const p = cloth.positions;
  for (let i = 0; i < p.length; i += 3) {
    assert.ok(!(Math.hypot(p[i], p[i + 2]) < 0.5 - 1e-3 && p[i + 1] < 1 - 1e-3 && p[i + 1] > 0), 'a corner in the drum');
  }
  assert.throws(() => world.addBody({ type: 'character', shape: 'cylinder' }), /^Error: addBody: type must be 'dynamic', 'static' or 'kinematic' for a cylinder, got 'character'$/);
  // Any height: a disc as well as a log.
  world.addBody({ shape: 'cylinder', radius: 1, height: 0.01, position: [5, 1, 5] });
  const read = world.bodyOf(drum);
  assert.equal(read.radius, 0.5);
  assert.equal(read.height, 1);
});

test('a saved world with cylinders on terrain steps on the same', () => {
  const world = new World({ workers: 0 });
  world.addBody({ shape: 'heightfield', heights: [0, 0.3, 0, 0.2, 0.5, 0.1, 0, 0.2, 0], columns: 3, spacing: 3 });
  for (let i = 0; i < 6; i++) world.addBody({ shape: 'cylinder', radius: 0.2, height: 0.3, position: [i - 2.5, 2, 0], rotation: [0.2 * i, 0.1, 0, 1] });
  steps(world, 0.5);
  const copy = World.load(JSON.parse(JSON.stringify(world.save())));
  steps(world, 1);
  steps(copy, 1);
  assert.deepEqual(Array.from(copy.transforms), Array.from(world.transforms));
});
