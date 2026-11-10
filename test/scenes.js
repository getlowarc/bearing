// Scenes more than one test file steps.

import { World } from '../src/bearing.js';

/**
 * A world with a bit of everything: a pile that falls asleep, bouncing
 * balls, capsules, a hull and a compound, a chain of joints, a character
 * walking, a sensor, a kinematic platform, a mesh and a heightfield, a fast
 * rod, a force waiting, and joints driven to targets: a limb, a hand after a
 * moving angle, a post kept upright, and a crate whose turn is held; water;
 * a cart on wheels; and soft bodies: a flag in the wind, a ball, and a cloth
 * thrown over a crate.
 */
export function mixed(options = {}) {
  const world = new World(options);
  world.addBody({ type: 'static', size: [30, 1, 30], position: [0, -0.5, 0] });
  world.addBody({ shape: 'mesh', vertices: [20, 0, -5, 30, 2, -5, 30, 2, 5, 20, 0, 5], indices: [0, 1, 2, 0, 2, 3], position: [-4, 0.01, 0] });
  world.addBody({ shape: 'heightfield', heights: [0, 0.2, 0, 0.3, 0.5, 0.1, 0, 0.2, 0], columns: 3, spacing: 2, position: [-10, 0.01, 8] });
  for (let i = 0; i < 12; i++) world.addBody({ position: [3, 0.5 + i * 1.01, 3] });
  for (let i = 0; i < 6; i++) world.addBody({ shape: 'sphere', radius: 0.3, restitution: 0.7, position: [-3 + i, 3 + i, -2], velocity: [1, 0, 0.5] });
  world.addBody({ shape: 'capsule', radius: 0.25, height: 1.2, position: [0, 4, 0], angularVelocity: [2, 0, 1] });
  world.addBody({ shape: 'hull', vertices: [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0.6, 0.6, 0.6], position: [5, 2, -4] });
  world.addBody({ shape: 'compound', parts: [{ size: [0.6, 0.2, 0.2] }, { shape: 'sphere', radius: 0.2, position: [0.4, 0, 0] }], position: [-5, 3, 5], angularVelocity: [0, 3, 1] });
  let last = null;
  for (let i = 0; i < 6; i++) {
    const link = world.addBody({ size: [0.1, 0.4, 0.1], position: [8, 6 - 0.4 * (i + 0.5), 0] });
    world.addJoint(last ? { type: 'ball', a: link, b: last, anchor: [8, 6 - 0.4 * i, 0] } : { type: 'hinge', a: link, anchor: [8, 6, 0], axis: [0, 0, 1], motorSpeed: 1, motorForce: 5 });
    last = link;
  }
  const arm = world.addBody({ size: [0.6, 0.15, 0.15], position: [10.3, 4, 0] });
  world.addJoint({ type: 'ball', a: arm, anchor: [10, 4, 0], target: [0, 0, 0.38, 0.92], motorForce: 40, frequency: 3, cone: 1 });
  const hand = world.addBody({ size: [0.4, 0.1, 0.1], position: [10.8, 4, 0] });
  world.addJoint({ type: 'hinge', a: hand, b: arm, anchor: [10.6, 4, 0], axis: [0, 0, 1], target: 0.5, motorSpeed: 1, motorForce: 10 });
  const post = world.addBody({ size: [0.3, 1.2, 0.3], position: [10, 0.6, 3], velocity: [0, 0, -1] });
  world.addJoint({ type: 'rotation', a: post, target: [0.1, 0, 0, 1], motorForce: 200, frequency: 2, dampingRatio: 0.5, cone: 0.3 });
  const crate = world.addBody({ position: [12, 3, 3], angularVelocity: [0, 2, 0] });
  world.addJoint({ type: 'rotation', a: crate });
  const walker = world.addBody({ type: 'character', shape: 'capsule', radius: 0.3, height: 1.6, position: [-6, 0.8, -6], velocity: [1.5, 0, 0] });
  world.addBody({ type: 'static', sensor: true, size: [2, 2, 2], position: [-3, 1, -6] });
  world.addBody({ type: 'kinematic', size: [2, 0.2, 2], position: [6, 0.5, 6], velocity: [0, 0, -1] });
  world.addBody({ size: [0.1, 0.02, 0.02], position: [-12, 1, 12], velocity: [200, 0, 0], rotation: [0.2, 0.3, 0.1, 1] });
  world.addBody({ type: 'static', size: [0.01, 4, 4], position: [-8, 2, 12] });
  // A cart on two wheel joints, driven, its suspension sprung.
  const cart = world.addBody({ size: [0.8, 0.2, 1.2], density: 300, position: [14, 0.55, -10] });
  for (const z of [-0.45, 0.45]) {
    const tyre = world.addBody({ shape: 'sphere', radius: 0.25, density: 300, position: [14, 0.25, -10 + z] });
    world.addJoint({ type: 'wheel', a: tyre, b: cart, anchor: [14, 0.25, -10 + z], frequency: 3, dampingRatio: 0.7, min: -0.1, max: 0.1, motorSpeed: 4, motorForce: 30 });
  }
  // Water, with a current, and a raft, a ball and a rock dropped in.
  world.addBody({ type: 'static', fluid: true, size: [4, 1.5, 4], position: [-12, 0.75, -12], flow: [0.3, 0, 0] });
  world.addBody({ size: [1, 0.3, 0.6], density: 500, position: [-12, 2.5, -12], rotation: [0.2, 0.1, 0, 1] });
  world.addBody({ shape: 'sphere', radius: 0.3, density: 400, position: [-13, 3, -11] });
  world.addBody({ shape: 'hull', vertices: [0, 0.3, 0, 0.3, -0.15, 0.1, -0.2, -0.15, 0.2, -0.1, -0.1, -0.3], density: 1500, position: [-11, 3, -13] });
  // A flag on a pole in the wind, a soft ball, and a cloth over a crate.
  const pole = world.addBody({ type: 'static', shape: 'capsule', radius: 0.05, height: 3, position: [16, 1.5, 6] });
  const flag = sheet(8, 5, 1.2, 0.75);
  world.addSoftBody({ ...flag, rotation: [Math.SQRT1_2, 0, 0, Math.SQRT1_2], position: [16.65, 2.6, 6], wind: [6, 0, 1],
    attach: [0, 9, 18, 27, 36, 45].map((vertex) => ({ vertex, body: pole })) });
  world.addSoftBody({ ...ball(1, 0.3), pressure: 1, stretch: 0.98, bend: 0.1, density: 200, position: [16, 2, 2], velocity: [0, 0, 1] });
  world.addBody({ size: [0.6, 0.6, 0.6], density: 200, position: [16, 0.3, -2] });
  world.addSoftBody({ ...sheet(8, 8, 1.6, 1.6), position: [16, 1.2, -2], bend: 0.05 });
  return { world, walker };
}

/** A flat sheet in x and z, `columns` by `rows` squares, each cut corner to corner, centred: corners row by row. */
export function sheet(columns, rows, width, depth) {
  const vertices = [], indices = [];
  for (let j = 0; j <= rows; j++) for (let i = 0; i <= columns; i++) vertices.push(width * (i / columns - 0.5), 0, depth * (j / rows - 0.5));
  for (let j = 0; j < rows; j++) {
    for (let i = 0; i < columns; i++) {
      const a = j * (columns + 1) + i, b = a + 1, c = a + columns + 1, d = c + 1;
      indices.push(a, c, b, b, c, d);
    }
  }
  return { vertices, indices };
}

/** A ball of radius r: an icosahedron, its triangles split k times, wound counter-clockwise seen from outside. */
export function ball(k, r) {
  const t = (1 + Math.sqrt(5)) / 2;
  let v = [[-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0], [0, -1, t], [0, 1, t], [0, -1, -t], [0, 1, -t], [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1]];
  let f = [[0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11], [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8],
    [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9], [4, 9, 5], [2, 4, 11], [6, 2, 10], [8, 6, 7], [9, 8, 1]];
  for (let s = 0; s < k; s++) {
    const mid = new Map(), next = [];
    const m = (a, b) => {
      const key = a < b ? `${a},${b}` : `${b},${a}`;
      if (!mid.has(key)) { mid.set(key, v.length); v.push(v[a].map((x, i) => (x + v[b][i]) / 2)); }
      return mid.get(key);
    };
    for (const [a, b, c] of f) { const ab = m(a, b), bc = m(b, c), ca = m(c, a); next.push([a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]); }
    f = next;
  }
  const vertices = v.flatMap((p) => { const l = Math.hypot(...p); return p.map((x) => (x / l) * r); });
  return { vertices, indices: f.flat() };
}
