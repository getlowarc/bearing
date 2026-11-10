// Soft bodies (milestone 25): cloth that hangs its own length and stretches as
// `stretch` says, a flag that waves on its pole, cloth that drapes over a box
// and sleeps, a ball that keeps its volume as it lands, soft and rigid bodies
// pushing each other both ways with momentum kept, attachments that let go
// with their bodies, the setters and readers, saving, and nothing done in a
// world with none.
// Run: node test/soft.test.js

import assert from 'node:assert/strict';
import { World, SoftBody } from '../src/bearing.js';
import { sheet, ball } from './scenes.js';
import { suite, test } from './test.js';

suite('soft');

const steps = (world, seconds, each) => {
  for (let i = 0; i < Math.round(seconds * 60); i++) { world.step(1 / 60); if (each) each(i); }
};
/** Stood up: the sheet's z becomes -y, so row 0 is the top. */
const UPRIGHT = [Math.SQRT1_2, 0, 0, Math.SQRT1_2];
const lowest = (p) => { let y = Infinity; for (let i = 1; i < p.length; i += 3) y = Math.min(y, p[i]); return y; };
const volume = (p, idx) => {
  let six = 0;
  for (let t = 0; t < idx.length; t += 3) {
    const a = 3 * idx[t], b = 3 * idx[t + 1], c = 3 * idx[t + 2];
    six += p[a] * (p[b + 1] * p[c + 2] - p[b + 2] * p[c + 1]) + p[a + 1] * (p[b + 2] * p[c] - p[b] * p[c + 2]) + p[a + 2] * (p[b] * p[c + 1] - p[b + 1] * p[c]);
  }
  return six / 6;
};
/** A 1 m cloth hung by its top edge from the world, its top at y = 2.5. */
function hung(world, columns, options = {}) {
  const top = [];
  for (let i = 0; i <= columns; i++) top.push({ vertex: i });
  return world.addSoftBody({ ...sheet(columns, columns, 1, 1), rotation: UPRIGHT, position: [0, 2, 0], attach: top, ...options });
}

test('a cloth hung by its top edge hangs its own length, however fine, and stretches as `stretch` says', () => {
  for (const columns of [10, 20, 40]) {
    const world = new World();
    const cloth = hung(world, columns);
    steps(world, 10);
    // Its length to within its thickness, at 40 edges as at 10 (the tethers).
    assert.ok(Math.abs(2.5 - lowest(cloth.positions) - 1) < 0.001, `${columns} edges: hangs ${2.5 - lowest(cloth.positions)} m`);
    assert.ok(cloth.asleep, `${columns} edges: awake`);
  }
  // Stretched by (1 - s) / s at the top: 1% and 11%, to a tenth of that.
  for (const [stretch, top] of [[0.99, 0.0101], [0.9, 0.111]]) {
    const world = new World();
    const cloth = hung(world, 20, { stretch });
    steps(world, 15);
    const p = cloth.positions, strain = (p[1] - p[3 * 21 + 1]) * 20 - 1;
    assert.ok(Math.abs(strain / top - 1) < 0.1, `stretch ${stretch}: ${strain} at the top`);
  }
});

test('a flag on a pole waves in the wind, held to the pole', () => {
  const world = new World();
  const pole = world.addBody({ type: 'static', shape: 'capsule', radius: 0.03, height: 3, position: [0, 1.5, 0] });
  const columns = 15, rows = 10, attach = [];
  for (let j = 0; j <= rows; j++) attach.push({ vertex: j * (columns + 1), body: pole });
  const flag = world.addSoftBody({ ...sheet(columns, rows, 1.5, 1), rotation: UPRIGHT, position: [0.78, 2.5, 0], wind: [8, 0, 1.3], attach });
  const p = flag.positions, held = attach.map(({ vertex }) => Array.from(p.slice(3 * vertex, 3 * vertex + 3)));
  steps(world, 3);
  let moved = 0;
  const last = Float32Array.from(p);
  steps(world, 3, () => {
    for (let i = 0; i < p.length; i++) moved += Math.abs(p[i] - last[i]);
    last.set(p);
  });
  // It flies out downwind, and keeps moving: a corner a few millimetres a step.
  let out = 0;
  for (let j = 0; j <= rows; j++) out += p[3 * (j * (columns + 1) + columns)] / (rows + 1);
  assert.ok(out > 0.3, `its free edge at x = ${out}`);
  assert.ok(moved / (180 * flag.positions.length / 3) > 0.002, `${moved} m of motion in 3 s`);
  assert.ok(!flag.asleep);
  // The edge on the pole hasn't moved from it: a static body holds it exactly.
  attach.forEach(({ vertex }, k) => {
    for (let c = 0; c < 3; c++) assert.ok(Math.abs(p[3 * vertex + c] - held[k][c]) < 1e-6, `corner ${vertex} left the pole`);
  });
});

test('a cloth dropped over a box drapes it, stays out of it and the floor, and sleeps', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  const box = world.addBody({ size: [0.6, 0.6, 0.6], position: [0, 0.3, 0], density: 200 });
  const cloth = world.addSoftBody({ ...sheet(20, 20, 2, 2), position: [0, 1, 0] });
  let t = 0;
  while (!cloth.asleep && t < 900) { world.step(1 / 60); t++; }
  assert.ok(cloth.asleep, 'still awake at 15 s');
  assert.ok(t < 600, `asleep at ${t / 60} s`);
  // Within a contact's tolerance of the box and the floor, which give that much (a pixel of 1 mm).
  const p = cloth.positions, r = 0.0005, slack = 2e-4, top = box.position[1] + 0.3;
  let over = 0;
  for (let i = 0; i < p.length; i += 3) {
    assert.ok(p[i + 1] > r - slack, `a corner in the floor at y = ${p[i + 1]}`);
    const inside = Math.abs(p[i]) < 0.3 + r - slack && Math.abs(p[i + 2]) < 0.3 + r - slack && p[i + 1] < top + r - slack;
    assert.ok(!inside, `a corner in the box at ${[p[i], p[i + 1], p[i + 2]]}`);
    if (Math.abs(p[i]) < 0.3 && Math.abs(p[i + 2]) < 0.3) { over++; assert.ok(Math.abs(p[i + 1] - top - r) < 0.002, 'the top not lying on the box'); }
  }
  assert.ok(over > 20, `${over} corners on top of the box`);
  // Held down by the cloth, held up by the floor: the box hasn't moved.
  const [x, y, z] = box.position;
  assert.ok(Math.hypot(x, y - 0.3, z) < 0.005, `the box moved to ${box.position}`);
});

test('a ball keeps its volume as it lands', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  const shape = ball(2, 0.25), soft = world.addSoftBody({ ...shape, pressure: 1, position: [0, 2, 0] });
  const V0 = volume(soft.positions, shape.indices);
  let worst = 0;
  steps(world, 3, () => { worst = Math.max(worst, Math.abs(volume(soft.positions, shape.indices) / V0 - 1)); });
  assert.ok(worst < 0.015, `${worst * 100}% off its volume landing`);
  assert.ok(Math.abs(volume(soft.positions, shape.indices) / V0 - 1) < 0.002, 'off its volume at rest');
  assert.ok(Math.abs(lowest(soft.positions) - 0.0005) < 0.0002, `its bottom at ${lowest(soft.positions)}`);
  // Pumped up, with a skin that gives, it swells; without pressure, a bag of nothing slumps.
  const swell = new World({ gravity: [0, 0, 0] });
  const big = swell.addSoftBody({ ...shape, pressure: 1.1, stretch: 0.95 });
  steps(swell, 2);
  assert.ok(volume(big.positions, shape.indices) / V0 > 1.05);
});

test('a soft body pushes a rigid body and is pushed back, keeping their momentum', () => {
  // A ball thrown at a box floating free: what one loses the other gains,
  // but for the air's drag on the ball, which a dense one hardly feels (0.3%).
  const world = new World({ gravity: [0, 0, 0] });
  const box = world.addBody({ size: [0.5, 0.5, 0.5], position: [1, 0, 0], density: 4000 });
  const thrown = world.addSoftBody({ ...ball(2, 0.2), pressure: 1, density: 2000, velocity: [3, 0, 0] });
  // The ball's corners' momentum, from inside: the handle gives positions only.
  const s = world._softs.list[world._softs.slots.slotOf(thrown.id)];
  const P = () => { let p = box.mass * box.velocity[0]; for (let i = 0; i < s.n; i++) p += s.v[3 * i] / s.w[i]; return p; };
  const before = P();
  steps(world, 1);
  assert.ok(box.velocity[0] > 0.3, `the box moving at ${box.velocity[0]}`);
  assert.ok(Math.abs(P() / before - 1) < 0.01, `momentum ${before} -> ${P()}`);
  // A hammock holds a ball five times its weight off the ground.
  const yard = new World();
  const net = sheet(16, 16, 2, 2), corners = [0, 16, 272, 288];
  for (const c of corners) { net.vertices[3 * c] *= 0.8; net.vertices[3 * c + 2] *= 0.8; }
  yard.addSoftBody({ ...net, position: [0, 2, 0], thickness: 0.002, attach: corners.map((vertex) => ({ vertex })) });
  const rock = yard.addBody({ shape: 'sphere', radius: 0.3, position: [0, 3, 0], mass: 40 });
  steps(yard, 8);
  assert.ok(rock.position[1] > 1 && rock.position[1] < 1.6, `the ball at y = ${rock.position[1]}`);
});

test('a body hung from a cloth hangs from it, and lets go when removed', () => {
  const world = new World();
  const top = [];
  for (let i = 0; i <= 10; i++) top.push({ vertex: i });
  const cloth = world.addSoftBody({ ...sheet(10, 10, 1, 1), rotation: UPRIGHT, position: [0, 2, 0], attach: top });
  const box = world.addBody({ size: [1, 0.2, 0.2], position: [0, 1.4, 0], mass: 2 });
  world.setSoftBody(cloth, { attach: [...top, { vertex: 110, body: box }, { vertex: 120, body: box }] });
  steps(world, 5);
  assert.ok(Math.abs(box.position[1] - 1.4) < 0.03, `the box at y = ${box.position[1]}`);
  assert.equal(world.softBodyOf(cloth).attach.length, 13);
  world.remove(box);
  assert.equal(world.softBodyOf(cloth).attach.length, 11);
  assert.ok(!cloth.asleep);
  // And the world's pins let go when the cloth is set with none: it falls.
  world.setSoftBody(cloth, { attach: [] });
  steps(world, 0.5);
  assert.ok(lowest(cloth.positions) < 1.5 - 0.5 * 9.81 * 0.25 * 0.8);
});

test('a sleeping cloth wakes when what it lies on moves or goes, or something lands on it', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  const box = world.addBody({ size: [0.6, 0.6, 0.6], position: [0, 0.3, 0], density: 200 });
  const cloth = world.addSoftBody({ ...sheet(16, 16, 2, 2), position: [0, 0.8, 0] });
  steps(world, 10);
  assert.ok(cloth.asleep && box.asleep);
  world.addBody({ shape: 'sphere', radius: 0.1, position: [0.3, 1.2, 0.3] });
  steps(world, 0.5);
  assert.ok(!cloth.asleep, 'a ball landed on it');
  steps(world, 12);
  assert.ok(cloth.asleep);
  world.remove(box);
  world.step(1 / 60);
  assert.ok(!cloth.asleep, 'what it lay on was removed');
  steps(world, 1);
  assert.ok(lowest(cloth.positions) < 0.1, 'it fell');
  // Off, nothing sleeps.
  world.sleeping = false;
  assert.ok(!cloth.asleep);
});

test('layers and mask choose what a soft body touches, as for bodies', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0], layers: 2 });
  const through = world.addSoftBody({ ...sheet(4, 4, 1, 1), position: [0, 0.5, 0], mask: ~2 });
  const onIt = world.addSoftBody({ ...sheet(4, 4, 1, 1), position: [3, 0.5, 0] });
  steps(world, 1);
  assert.ok(lowest(through.positions) < -1);
  assert.ok(lowest(onIt.positions) > 0);
});

test('softBodyOf gives what addSoftBody takes; setSoftBody moves, spins, reshapes and reattaches', () => {
  const world = new World();
  const pole = world.addBody({ type: 'static', size: [0.1, 3, 0.1], position: [0, 1.5, 0] });
  const flag = world.addSoftBody({ ...sheet(4, 3, 1, 0.6), rotation: UPRIGHT, position: [0.55, 2.5, 0], density: 300, thickness: 0.002,
    bend: 0.2, friction: 0.3, damping: 0.1, wind: [1, 2, 3], layers: 4, mask: 5, velocity: [1, 0, 0], attach: [{ vertex: 0, body: pole }, { vertex: 5 }] });
  const o = world.softBodyOf(flag);
  assert.deepEqual(o.vertices, sheet(4, 3, 1, 0.6).vertices);
  assert.equal(o.density, 300);
  assert.equal(o.thickness, 0.002);
  assert.deepEqual(o.wind, [1, 2, 3]);
  assert.equal(o.attach.length, 2);
  assert.equal(o.attach[0].body.id, pole.id);
  assert.equal(o.attach[1].body, null);
  assert.ok(Math.abs(o.velocity[0] - 1) < 1e-12);
  const copy = world.addSoftBody(o);
  assert.deepEqual(Array.from(copy.positions), Array.from(flag.positions));
  // Moved by hand: every corner by the same move, the handle's array the same.
  const before = Array.from(flag.positions), array = flag.positions;
  world.setSoftBody(flag, { position: [1.55, 2.5, 0] });
  assert.equal(flag.positions, array);
  for (let i = 0; i < before.length; i += 3) assert.ok(Math.abs(flag.positions[i] - before[i] - 1) < 1e-6);
  // Reshaped: a fresh array, the corners as given, its attachments gone.
  world.setSoftBody(flag, { ...sheet(2, 2, 1, 1) });
  assert.equal(flag.positions.length, 27);
  assert.equal(world.softBodyOf(flag).attach.length, 0);
  // Removed: its handle is dead, and removing it twice does nothing.
  world.remove(flag);
  assert.equal(flag.alive, false);
  flag.destroy();
  assert.throws(() => flag.positions, /^Error: softBody\.positions: this soft body was removed$/);
  assert.ok(world.softBody(copy.id) instanceof SoftBody);
  assert.equal(world.softBody(copy.id).id, copy.id);
});

test('a soft body steps the same after a save and load', () => {
  const world = new World({ workers: 0 });
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  world.addBody({ size: [0.6, 0.6, 0.6], position: [0, 0.3, 0], density: 200 });
  const cloth = world.addSoftBody({ ...sheet(8, 8, 1.6, 1.6), position: [0, 1, 0], wind: [1, 0, 0] });
  world.addSoftBody({ ...ball(1, 0.2), pressure: 1, position: [1, 1, 0] });
  steps(world, 0.5);
  const copy = World.load(JSON.parse(JSON.stringify(world.save())));
  const again = copy.softBody(cloth.id);
  steps(world, 1);
  steps(copy, 1);
  assert.deepEqual(Array.from(again.positions), Array.from(cloth.positions));
  assert.deepEqual(Array.from(copy.transforms), Array.from(world.transforms));
});

test('off means off: a world with no soft bodies does no soft-body work', () => {
  const world = new World();
  world.addBody({ type: 'static', size: [10, 1, 10], position: [0, -0.5, 0] });
  world.addBody({ position: [0, 2, 0] });
  steps(world, 1);
  assert.equal(world._softSteps, 0);
  const cloth = world.addSoftBody({});
  steps(world, 1);
  assert.equal(world._softSteps, 60);
  world.remove(cloth);
  steps(world, 1);
  assert.equal(world._softSteps, 60);
});
