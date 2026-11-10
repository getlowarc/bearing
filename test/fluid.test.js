// Water (milestone 22): every shape's part under a surface held to a brute-force
// count, bodies floating where Archimedes puts them, righting themselves,
// settling and sleeping, carried by a current, woken when the water changes,
// and nothing done in a world with no water.
// Run: node test/fluid.test.js

import assert from 'node:assert/strict';
import { World } from '../src/bearing.js';
import { submerged, UNDER } from '../src/fluid.js';
import { Shape, BOX, SPHERE, CAPSULE, HULL } from '../src/collide.js';
import { build } from '../src/hull.js';
import { suite, test } from './test.js';

suite('fluid');

const steps = (world, seconds, each) => {
  for (let i = 0; i < Math.round(seconds * 60); i++) { world.step(1 / 60); if (each) each(i); }
};
const ROCK = [0, 0.4, 0, 0.4, -0.2, 0.1, -0.3, -0.2, 0.3, -0.1, -0.2, -0.4, 0.2, 0.1, -0.2];
/** A pool 40 m across, its surface at y = 0, 6 m deep. */
const pool = (world, options = {}) => world.addBody({ type: 'static', fluid: true, size: [40, 6, 40], position: [0, -3, 0], ...options });

/** A seeded number from 0 to 1, the same everywhere. */
function numbers(seed) {
  return () => (seed = (seed * 16807) % 2147483647) / 2147483647;
}

/** A unit quaternion from three numbers from 0 to 1 (Shoemake's). */
function turn(a, b, c) {
  const s = Math.sqrt(1 - a), t = Math.sqrt(a);
  return [s * Math.sin(2 * Math.PI * b), s * Math.cos(2 * Math.PI * b), t * Math.sin(2 * Math.PI * c), t * Math.cos(2 * Math.PI * c)];
}

/** Whether a point is inside placed shape s: in its own frame, against its own sizes. */
function inside(s, x, y, z) {
  const A = s.axes, o = s.center, dx = x - o[0], dy = y - o[1], dz = z - o[2];
  const lx = A[0] * dx + A[1] * dy + A[2] * dz, ly = A[3] * dx + A[4] * dy + A[5] * dz, lz = A[6] * dx + A[7] * dy + A[8] * dz;
  if (s.kind === BOX) return Math.abs(lx) <= s.half[0] && Math.abs(ly) <= s.half[1] && Math.abs(lz) <= s.half[2];
  if (s.kind === SPHERE) return lx * lx + ly * ly + lz * lz <= s.half[0] * s.half[0];
  if (s.kind === CAPSULE) {
    const c = Math.max(-s.half[1], Math.min(s.half[1], ly));
    return lx * lx + (ly - c) * (ly - c) + lz * lz <= s.half[0] * s.half[0];
  }
  const p = s.hull.planes;
  for (let f = 0; f < p.length; f += 4) if (p[f] * lx + p[f + 1] * ly + p[f + 2] * lz > p[f + 3]) return false;
  return true;
}

test('the part of every shape under a surface is what a count of points under it finds', () => {
  // Points on a grid of 64 a side through each shape's box, each counted if it
  // is inside the shape and under the plane: the grid's error is about a
  // point's share of the surface, so 0.6% of the volume is what it can tell.
  const next = numbers(11), s = new Shape();
  const shapes = [
    () => { s.kind = BOX; s.half.set([0.3 + next(), 0.3 + next(), 0.3 + next()]); },
    () => { s.kind = SPHERE; s.half[0] = 0.3 + next(); },
    () => { s.kind = CAPSULE; s.half[0] = 0.2 + 0.5 * next(); s.half[1] = next(); },
    () => { s.kind = HULL; s.hull = build(ROCK.map((v) => v * (1 + next()))); },
  ];
  for (const make of shapes) {
    for (let k = 0; k < 6; k++) {
      make();
      s.place(next() - 0.5, next() - 0.5, next() - 0.5, turn(next(), next(), next()));
      let nx = next() - 0.5, ny = next() - 0.5, nz = next() - 0.5;
      const l = Math.hypot(nx, ny, nz); nx /= l; ny /= l; nz /= l;
      const c = (next() - 0.5) * 1.2;
      UNDER.fill(0);
      submerged(s, nx, ny, nz, c, UNDER);
      const R = s.kind === HULL ? s.hull.reach : s.kind === CAPSULE ? s.half[0] + s.half[1] : s.kind === SPHERE ? s.half[0] : Math.hypot(...s.half);
      const N = 64, d = 2 * R / N;
      let count = 0, all = 0, mx = 0, my = 0, mz = 0;
      for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) for (let m = 0; m < N; m++) {
        const x = s.center[0] - R + (i + 0.5) * d, y = s.center[1] - R + (j + 0.5) * d, z = s.center[2] - R + (m + 0.5) * d;
        if (!inside(s, x, y, z)) continue;
        all++;
        if (nx * x + ny * y + nz * z > c) continue;
        count++; mx += x; my += y; mz += z;
      }
      const volume = all * d * d * d, want = count * d * d * d;
      assert.ok(Math.abs(UNDER[0] - want) < 0.006 * volume, `kind ${s.kind}: ${UNDER[0]} under, the count ${want}`);
      if (count > N * N) {
        const reach = Math.cbrt(volume);
        for (const [got, sum] of [[UNDER[1], mx], [UNDER[2], my], [UNDER[3], mz]]) {
          assert.ok(Math.abs(got / UNDER[0] - sum / count) < 0.01 * reach, `kind ${s.kind}: centre ${got / UNDER[0]}, the count's ${sum / count}`);
        }
      }
    }
  }
});

test('a box, a ball and a capsule wholly under, half under and upright are exact', () => {
  const s = new Shape();
  s.kind = BOX; s.half.set([0.5, 1, 1.5]); s.place(0, 0, 0, [0, 0, 0, 1]);
  UNDER.fill(0); submerged(s, 0, 1, 0, 0, UNDER);
  assert.equal(UNDER[0], 3);
  assert.equal(UNDER[2] / UNDER[0], -0.5);
  s.kind = SPHERE; s.half[0] = 1;
  UNDER.fill(0); submerged(s, 0, 1, 0, 0, UNDER);
  assert.ok(Math.abs(UNDER[0] - 2 * Math.PI / 3) < 1e-15 && Math.abs(UNDER[2] / UNDER[0] + 3 / 8) < 1e-15);
  // Upright, a capsule's discs are each wholly under or not: its half is exact.
  s.kind = CAPSULE; s.half[0] = 0.5; s.half[1] = 1;
  const full = Math.PI * 0.25 * 2 + (4 / 3) * Math.PI * 0.125;
  UNDER.fill(0); submerged(s, 0, 1, 0, 0, UNDER);
  assert.ok(Math.abs(UNDER[0] - full / 2) < 1e-15 * full, `${UNDER[0]} against ${full / 2}`);
  UNDER.fill(0); submerged(s, 0, 1, 0, 10, UNDER);
  assert.equal(UNDER[0], full);
});

test('a body of half water\'s density floats half under, and one of a quarter a quarter under', () => {
  const world = new World();
  pool(world);
  const box = world.addBody({ size: [1, 1, 1], density: 500, position: [0, 1, 0] });
  const ball = world.addBody({ shape: 'sphere', radius: 0.5, density: 500, position: [3, 1, 0] });
  // A slab, wide and flat: a light cube floats on a corner (see the timber below), a slab on its face.
  const light = world.addBody({ size: [1, 0.4, 1], density: 250, position: [-3, 1, 0] });
  steps(world, 30);
  // At rest within the sleep rule's tolerance: 5 mm for a 1 m body.
  assert.ok(Math.abs(box.position[1]) < 0.005, `the box's centre at ${box.position[1]}`);
  assert.ok(Math.abs(ball.position[1]) < 0.005, `the ball's centre at ${ball.position[1]}`);
  assert.ok(Math.abs(light.position[1] - 0.1) < 0.002, `the slab's centre at ${light.position[1]}`);
  for (const b of [box, ball, light]) assert.ok(b.asleep);
});

test('a plank floats flat, a square timber corner down, and a body denser than water sinks', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [40, 1, 40], position: [0, -6.5, 0] });
  pool(world);
  const plank = world.addBody({ size: [3, 0.2, 0.6], density: 600, position: [0, 2, 0], rotation: [0.3, 0.2, 0.1, 0.927] });
  const timber = world.addBody({ size: [3, 0.4, 0.4], density: 600, position: [0, 2, 4], rotation: [0.3, 0.2, 0.1, 0.927] });
  const rock = world.addBody({ shape: 'hull', vertices: ROCK, density: 2500, position: [4, 1, 0] });
  steps(world, 30);
  const axis = (b, k) => {
    const [x, y, z, w] = b.rotation;
    return [[1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)], [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)]][k];
  };
  // Flat on its face, 0.6 of its 0.2 m under: its centre 0.02 m down.
  assert.ok(Math.abs(axis(plank, 1)[1]) > 0.999, `the plank's thin axis leans to ${axis(plank, 1)}`);
  assert.ok(Math.abs(plank.position[1] + 0.02) < 0.002, `the plank's centre at ${plank.position[1]}`);
  // A square timber between about 0.21 and 0.79 of water's density floats on a corner, not a face:
  // its length level, its sides at 45°, and 0.6 of the diamond under puts its centre 30.2 mm down.
  const long = axis(timber, 0), side = axis(timber, 1);
  assert.ok(Math.abs(long[1]) < 0.01, `the timber's length leans ${long[1]} out of the water`);
  assert.ok(Math.abs(Math.abs(side[1]) - Math.SQRT1_2) < 0.01, `the timber's side leans ${side[1]}`);
  const d = 0.2 * Math.SQRT2, h = 2 * d - Math.sqrt(0.16 * 0.4);
  assert.ok(Math.abs(timber.position[1] - (d - h)) < 0.002, `the timber's centre at ${timber.position[1]}, not ${d - h}`);
  assert.ok(rock.position[1] < -5.5, `the rock at ${rock.position[1]}`);
});

test('dropped in, every shape bobs, settles and sleeps with the water as it comes', () => {
  const world = new World();
  pool(world);
  const bodies = [
    world.addBody({ size: [1, 1, 1], density: 500, position: [0, 2, 0] }),
    world.addBody({ shape: 'sphere', radius: 0.5, density: 500, position: [3, 2, 0] }),
    world.addBody({ shape: 'capsule', radius: 0.3, height: 1.6, density: 300, position: [-3, 2, 0], rotation: [0, 0, 0.3826834, 0.9238795] }),
    world.addBody({ size: [3, 0.4, 0.4], density: 600, position: [0, 3, 4], rotation: [0.3, 0.2, 0.1, 0.927] }),
    world.addBody({ shape: 'hull', vertices: ROCK, density: 700, position: [3, 2, 4] }),
    world.addBody({ shape: 'compound', parts: [{ size: [1, 0.2, 1] }, { shape: 'sphere', radius: 0.3, position: [0, 0.4, 0] }], density: 400, position: [-3, 2, 4] }),
    world.addBody({ size: [0.1, 0.1, 0.1], density: 400, position: [-3, 1, -4] }),
  ];
  steps(world, 20);
  bodies.forEach((b, i) => assert.ok(b.asleep, `body ${i} awake at 20 s, at ${b.position}`));
});

test('rocking with no damping neither grows nor fades', () => {
  // The push is worked out from where each body is turned on every substep. From
  // the turn as the step began, the push lagged the rocking and fed it: this
  // plank went from 0.89 rad/s to 5.5 in 30 s at 8 substeps.
  const world = new World({ sleeping: false });
  pool(world, { damping: 0, angularDamping: 0 });
  const plank = world.addBody({ size: [1.6, 0.6, 0.6], density: 300, position: [0, 0.0916, 0], rotation: [0, 0, Math.sin(0.025), Math.cos(0.025)] });
  const most = [];
  for (let k = 0; k < 4; k++) {
    let top = 0;
    steps(world, 7.5, () => { top = Math.max(top, Math.abs(plank.angularVelocity[2])); });
    most.push(top);
  }
  for (const m of most) assert.ok(Math.abs(m / most[0] - 1) < 0.01, `rocking at most ${most.join(', ')} rad/s`);
});

test('a current carries what floats in it', () => {
  const world = new World();
  pool(world, { flow: [1, 0, 0] });
  const raft = world.addBody({ size: [2, 0.3, 1], density: 500, position: [0, 0, 0] });
  steps(world, 10);
  assert.ok(Math.abs(raft.velocity[0] - 1) < 0.01 && Math.abs(raft.velocity[2]) < 1e-9, `the raft at ${raft.velocity} m/s`);
});

test('water added over a sleeping body lifts it, raised by hand lifts it again, and taken away lets it fall', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  // A slab, which floats flat: 0.09 of its 0.3 m under, its centre 0.06 m over the surface.
  const box = world.addBody({ size: [1, 0.3, 1], density: 300, position: [0, 0.15, 0] });
  steps(world, 4);
  assert.ok(box.asleep);
  const water = world.addBody({ type: 'static', fluid: true, size: [20, 3, 20], position: [0, 1.5, 0] });
  steps(world, 15);
  assert.ok(Math.abs(box.position[1] - 3.06) < 0.002, `after water was added, at ${box.position[1]}`);
  water.setPosition(0, 2.5, 0);
  steps(world, 20);
  assert.ok(Math.abs(box.position[1] - 4.06) < 0.002, `after it rose, at ${box.position[1]}`);
  world.remove(water);
  steps(world, 5);
  assert.ok(Math.abs(box.position[1] - 0.15) < 0.002, `after it went, at ${box.position[1]}`);
});

test('a tide wakes what sleeps under it, and lifts it as it rises', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  const tide = world.addBody({ type: 'kinematic', fluid: true, size: [20, 2, 20], position: [0, -1.2, 0] });
  const box = world.addBody({ size: [1, 0.3, 1], density: 300, position: [0, 0.15, 0] });
  steps(world, 4);
  assert.ok(box.asleep);
  tide.setVelocity(0, 0.5, 0);
  steps(world, 6);
  tide.setVelocity(0, 0, 0);
  steps(world, 15);
  assert.ok(Math.abs(box.position[1] - (tide.position[1] + 1 + 0.06)) < 0.002, `the slab at ${box.position[1]}, the tide's top at ${tide.position[1] + 1}`);
});

test('water is a sensor: it reports what goes in, reads back, and costs nothing where there is none', () => {
  const world = new World();
  const water = pool(world);
  const ball = world.addBody({ shape: 'sphere', radius: 0.5, density: 500, position: [0, 2, 0] });
  let began = false;
  steps(world, 3, () => { for (const e of world.contacts(water)) if (e.type === 'begin' && e.b.id === ball.id) began = true; });
  assert.ok(began, 'no begin when the ball went in');
  const spec = world.bodyOf(water);
  assert.equal(spec.fluid, true);
  assert.equal(spec.sensor, true);
  assert.deepEqual(spec.flow, [0, 0, 0]);
  assert.equal(spec.damping, 3);
  assert.equal(world.bodyOf(ball).fluid, undefined);
  const copy = world.addBody(spec);
  assert.deepEqual(world.bodyOf(copy), spec);
  // Off means off: no water, no substep asks.
  const dry = new World();
  dry.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
  dry.addBody({ position: [0, 3, 0] });
  dry.addBody({ type: 'static', sensor: true, size: [4, 4, 4] });
  steps(dry, 2);
  assert.equal(dry._floatSteps, 0);
  assert.ok(world._floatSteps > 0);
});
