import assert from 'node:assert/strict';
import { Lift } from './lift.js';
import { mul, conj, eulerToQuat } from './orientation.js';

const G = 9.80665, DT = 0.02;
const rotate = (q, [x, y, z]) => { const t = mul(mul(q, [0, x, y, z]), conj(q)); return [t[1], t[2], t[3]]; };
let seed = 7;
const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };

// Board with a gravity error (reads 1.5% low) and sensor noise; optional hand tremor.
function run(tilt, script, seconds, hand) {
  const lift = new Lift();
  const q = eulerToQuat(tilt[0], tilt[1], 40);
  let r, ms = 0;
  for (let t = 0; t < seconds; t += DT, ms += 20) {
    const az = script(t); // true vertical acceleration, m/s^2
    const tremor = hand ? 0.5 * Math.sin(2 * Math.PI * 9 * t) : 0;
    const w = [0.4 * tremor * 0.3, 0.3 * tremor * 0.3, (az + tremor) / G + 0.985];
    const inv = rotate(conj(q), w);
    const f = inv.map((v) => v + 0.01 * rnd());
    const g = [1.5 * rnd() + (hand ? 2 * Math.sin(2 * Math.PI * 9 * t) : 0), 1.5 * rnd(), 1.5 * rnd()];
    r = lift.update({ ms, q, f, g });
  }
  return r;
}
// minimum-jerk move of height h over T s starting at t0
const move = (t0, T, h) => (t) => {
  const s = (t - t0) / T;
  if (s < 0 || s > 1) return 0;
  return (h / (T * T)) * (60 * s - 180 * s * s + 120 * s ** 3) * 1;
};

for (const [hand, tol] of [[false, 0.07], [true, 0.12]]) {
  for (const tilt of [[0, 0], [30, -20], [-71, 0]]) {
    for (const h of [0.3, -0.3, 0.15]) {
      const r = run(tilt, move(3, 1.0, h), 6, hand);
      assert.ok(Math.abs(r.pos[2] - h) < Math.max(tol, 0.4 * Math.abs(h)), `tilt ${tilt} hand ${hand} lift ${h}: showed ${r.pos[2].toFixed(3)}`);
      assert.ok(Math.sign(r.pos[2]) === Math.sign(h), 'wrong direction');
    }
  }
}
// still stays still
const rest = run([10, 5], () => 0, 60, false);
assert.ok(Math.abs(rest.pos[2]) < 0.01, `crept ${rest.pos[2]}`);
// up then down returns near the start
const both = run([0, 0], (t) => move(3, 1, 0.3)(t) + move(6, 1, -0.3)(t), 10, true);
assert.ok(Math.abs(both.pos[2]) < 0.12, `up-down ended at ${both.pos[2]}`);
console.log('lift tests passed');
