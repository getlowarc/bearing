// Pieces (milestone 24): a box or a hull cut into pieces that fill it exactly,
// face to face, the same for a seed on every machine, and resting where the
// whole did.
// Run: node test/fracture.test.js

import assert from 'node:assert/strict';
import { World, fracture } from '../src/bearing.js';
import { build } from '../src/hull.js';
import { hullMass } from '../src/shapes.js';
import { suite, test } from './test.js';

suite('fracture');

const ROCK = [0, 1, 0, 1, -0.5, 0.3, -0.8, -0.5, 0.6, -0.2, -0.4, -1, 0.6, 0.2, -0.5];
const CASES = [
  [{ size: [2, 1, 2] }, { count: 12, seed: 7 }],
  [{ shape: 'box', size: [0.3, 3, 0.5] }, { count: 9, seed: 2 }],
  [{ shape: 'hull', vertices: ROCK }, { count: 20, seed: 3 }],
  [{}, { count: 40, seed: 11 }],
];

/** A piece's hull, placed where it is in the shape's frame. */
const placed = (p) => build(p.vertices.map((v, i) => v + p.position[i % 3]));
const volumeOf = (shape) => hullMass(build(shape.shape === 'hull' ? shape.vertices : (() => {
  const [x, y, z] = (shape.size ?? [1, 1, 1]).map((v) => v / 2);
  return [-x, -y, -z, x, -y, -z, -x, y, -z, x, y, -z, -x, -y, z, x, -y, z, -x, y, z, x, y, z];
})())).volume;

test('the pieces fill the shape: there are as many as asked, each a solid, their volumes summing to its', () => {
  for (const [shape, options] of CASES) {
    const pieces = fracture(shape, options);
    assert.equal(pieces.length, options.count);
    let sum = 0;
    for (const p of pieces) {
      const h = build(p.vertices);
      assert.ok(h !== null, 'a piece that isn\'t a solid');
      // Its corners are about its own centre of mass.
      const m = hullMass(h);
      for (const c of m.center) assert.ok(Math.abs(c) < 1e-12, `a piece's centre at ${m.center}`);
      sum += m.volume;
    }
    const whole = volumeOf(shape);
    assert.ok(Math.abs(sum / whole - 1) < 1e-12, `${sum} m³ of pieces in ${whole}`);
  }
});

test('every point of the shape is in exactly one piece', () => {
  // Points spread through the shape's box, each tested against every piece's
  // faces: inside one, or on a face between two to rounding, never in two.
  let seed = 5;
  const next = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (const [shape, options] of CASES) {
    const pieces = fracture(shape, options).map(placed);
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (const h of pieces) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], h.lo[k]); hi[k] = Math.max(hi[k], h.hi[k]); }
    const size = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]), eps = 1e-9 * size;
    let inside = 0;
    for (let n = 0; n < 4000; n++) {
      const p = [0, 1, 2].map((k) => lo[k] + (hi[k] - lo[k]) * next());
      let deep = 0, near = 0;
      for (const h of pieces) {
        let most = -Infinity;
        for (let f = 0; f < h.planes.length; f += 4) most = Math.max(most, h.planes[f] * p[0] + h.planes[f + 1] * p[1] + h.planes[f + 2] * p[2] - h.planes[f + 3]);
        if (most < -eps) deep++;
        else if (most <= eps) near++;
      }
      assert.ok(deep <= 1, `a point inside ${deep} pieces`);
      assert.ok(deep === 1 || near > 0 || !inShape(shape, p, eps), `a point of the shape in no piece: ${p}`);
      if (deep === 1) inside++;
    }
    assert.ok(inside > 0);
  }
});

/** Whether p is inside the shape itself, by more than eps. */
function inShape(shape, p, eps) {
  if (shape.shape === 'hull') {
    const h = build(shape.vertices);
    for (let f = 0; f < h.planes.length; f += 4) if (h.planes[f] * p[0] + h.planes[f + 1] * p[1] + h.planes[f + 2] * p[2] - h.planes[f + 3] > -eps) return false;
    return true;
  }
  const s = shape.size ?? [1, 1, 1];
  return [0, 1, 2].every((k) => Math.abs(p[k]) < s[k] / 2 - eps);
}

test('a seed gives the same pieces every time, and another seed other ones', () => {
  const a = fracture({ shape: 'hull', vertices: ROCK }, { count: 15, seed: 42 });
  assert.deepEqual(fracture({ shape: 'hull', vertices: ROCK }, { count: 15, seed: 42 }), a);
  assert.notDeepEqual(fracture({ shape: 'hull', vertices: ROCK }, { count: 15, seed: 43 }), a);
  // Left out: eight pieces, of a 1 m cube, from seed 0.
  assert.deepEqual(fracture(), fracture({ shape: 'box', size: [1, 1, 1] }, { count: 8, seed: 0 }));
  // One piece is the whole.
  const [one] = fracture({ size: [2, 1, 3] }, { count: 1 });
  assert.deepEqual(one.position.map((v) => Math.abs(v) < 1e-15), [true, true, true]);
  assert.ok(Math.abs(hullMass(build(one.vertices)).volume - 6) < 1e-12);
});

test('broken where it lay, a slab\'s pieces rest where they were', () => {
  for (const [count, seed] of [[6, 4], [12, 7]]) {
    const world = new World();
    world.addBody({ type: 'static', size: [20, 1, 20], position: [0, -0.5, 0] });
    const bodies = fracture({ size: [2, 1, 2] }, { count, seed })
      .map((p) => world.addBody({ shape: 'hull', vertices: p.vertices, position: [p.position[0], p.position[1] + 0.5, p.position[2]], density: 500 }));
    const start = bodies.map((b) => b.position.slice());
    for (let i = 0; i < 300; i++) world.step(1 / 60);
    bodies.forEach((b, i) => {
      const moved = Math.hypot(b.position[0] - start[i][0], b.position[1] - start[i][1], b.position[2] - start[i][2]);
      assert.ok(moved < 0.002, `${count} pieces: piece ${i} moved ${moved} m`);
      assert.ok(b.asleep, `${count} pieces: piece ${i} awake`);
    });
  }
});
