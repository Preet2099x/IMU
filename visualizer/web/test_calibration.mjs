// Run with: node visualizer/web/test_calibration.mjs
import assert from 'node:assert/strict';
import { fitAccelCal, applyCal, faceOf } from './calibration.js';

let seed = 5;
const uniform = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
const gauss = () => Math.sqrt(-2 * Math.log(uniform() + 1e-12)) * Math.cos(2 * Math.PI * uniform());

// What a real sensor does to a true direction of gravity.
const truth = { offset: [0.006, -0.010, 0.014], scale: [1.004, 0.995, 0.979] };
const measure = (dir, noise = 0.001) => dir.map((d, i) => d / truth.scale[i] + truth.offset[i] + noise * gauss());
const unit = (v) => { const n = Math.hypot(...v); return v.map((c) => c / n); };
const randomDir = () => unit([gauss(), gauss(), gauss()]);

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: expected ${b}, got ${a}`);

// Six sides, the usual way to do it.
{
  const sides = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
  const fit = fitAccelCal(sides.map((d) => measure(d)));
  for (let i = 0; i < 3; i++) {
    near(fit.offset[i], truth.offset[i], 0.003, `offset ${i}`);
    near(fit.scale[i], truth.scale[i], 0.003, `scale ${i}`);
  }
  assert.ok(fit.rms < 0.003, `rms ${fit.rms}`);
}

// Any mix of directions works, they do not have to be exact sides.
{
  const fit = fitAccelCal(Array.from({ length: 14 }, () => measure(randomDir())));
  for (let i = 0; i < 3; i++) {
    near(fit.offset[i], truth.offset[i], 0.004, `random offset ${i}`);
    near(fit.scale[i], truth.scale[i], 0.004, `random scale ${i}`);
  }
  // The corrected readings of directions it has never seen are 1 g long.
  for (let k = 0; k < 50; k++) {
    const c = applyCal(measure(randomDir()), fit);
    near(Math.hypot(...c), 1, 0.006, 'held-out length');
  }
}

// Too few, or all the same side: refuse instead of returning nonsense.
assert.throws(() => fitAccelCal([[0, 0, 1], [0, 0, -1]]), /at least 6/);
assert.throws(() => fitAccelCal(Array.from({ length: 8 }, () => measure([0.1, 0.05, 0.99]))), /too alike|do not fit/);

// Which side is up.
assert.equal(faceOf([0.98, 0.05, 0.1]), 0);
assert.equal(faceOf([-0.97, 0.05, 0.1]), 1);
assert.equal(faceOf([0.02, 1.01, 0.03]), 2);
assert.equal(faceOf([0.02, -0.99, 0.03]), 3);
assert.equal(faceOf([0.02, 0.05, 0.98]), 4);
assert.equal(faceOf([0.02, 0.05, -0.99]), 5);
assert.equal(faceOf([0.6, 0.0, 0.8]), -1, 'tilted between sides');

console.log('calibration tests passed');
