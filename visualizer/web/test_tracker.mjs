// Run with: node visualizer/web/test_tracker.mjs
// Feeds the tracker simulated 50 Hz data from a sensor with realistic errors
// (per-axis scale and offset, noise) and checks where it ends up.
import assert from 'node:assert/strict';
import { Tracker } from './tracker.js';
import { eulerToQuat, mul, conj } from './orientation.js';
import { applyCal } from './calibration.js';

const G = 9.80665, DT = 0.02;
const rot = (q, v) => { const t = mul(mul(q, [0, ...v]), conj(q)); return [t[1], t[2], t[3]]; };
const s2 = (tau) => 60 * tau - 180 * tau * tau + 120 * tau ** 3; // 2nd derivative of a smooth 0..1 move
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

class Sim {
  constructor({ scale = [1.004, 0.995, 0.979], offset = [0.006, -0.010, 0.014], perfectCal = false, noise = 0.002, seed = 1, tracker = new Tracker(), tremorAcc = 0.5, tremorGyro = 2 } = {}) {
    this.tremorAcc = tremorAcc; this.tremorGyro = tremorGyro; this.t = 0; this.hand = false;
    this.scale = scale; this.offset = offset; this.noise = noise; this.tracker = tracker;
    this.cal = perfectCal ? { offset, scale: scale.map((s) => 1 / s) } : null;
    this.ms = 0; this.seed = seed; this.pos = [0, 0, 0]; this.q = [1, 0, 0, 0];
  }
  gauss() {
    const u = () => { this.seed = (this.seed * 1664525 + 1013904223) >>> 0; return this.seed / 4294967296; };
    return Math.sqrt(-2 * Math.log(u() + 1e-12)) * Math.cos(2 * Math.PI * u());
  }
  step(q, aTrue = [0, 0, 0], gyro = [0, 0, 0]) {
    // In a hand the board trembles at 8-10 Hz.
    this.t += DT;
    const tremble = (amp, f0) => [0, 1, 2].map((i) => (this.hand ? amp * Math.sin(2 * Math.PI * (f0 + i) * this.t + i * 2.1) : 0));
    const ta = tremble(this.tremorAcc, 8), tg = tremble(this.tremorGyro, 9);
    const fTrue = rot([q[0], -q[1], -q[2], -q[3]], [(aTrue[0] + ta[0]) / G, (aTrue[1] + ta[1]) / G, (aTrue[2] + ta[2]) / G + 1]);
    const raw = fTrue.map((v, i) => v * this.scale[i] + this.offset[i] + this.noise * this.gauss());
    const g = gyro.map((v, i) => v + tg[i] + 0.25 * this.gauss());
    this.q = q;
    const r = this.tracker.update({ ms: this.ms, q, f: this.cal ? applyCal(raw, this.cal) : raw, g });
    this.ms += DT * 1000;
    return r;
  }
  rest(secs, q = this.q) { let r; for (let t = 0; t < secs; t += DT) r = this.step(q); return r; }
  moveTo(target, T = 1.2, q = this.q) {
    for (let t = 0; t < T; t += DT) this.step(q, target.map((c, i) => ((c - this.pos[i]) * s2(t / T)) / (T * T)));
    this.pos = target;
  }
  // Turns about one axis ('roll' | 'pitch' | 'yaw'), degrees, without moving.
  turn(axis, from, to, T = 0.5, base = [0, 0, 0]) {
    const idx = { roll: 0, pitch: 1, yaw: 2 }[axis];
    let r;
    for (let t = 0; t < T; t += DT) {
      const ang = [...base]; ang[idx] = from + ((to - from) * t) / T;
      const gyro = [0, 0, 0]; gyro[idx] = (to - from) / T;
      r = this.step(eulerToQuat(...ang), [0, 0, 0], gyro);
    }
    return r;
  }
}

// 1. A minute at rest does not wander, and settles quickly.
{
  const sim = new Sim();
  const r = sim.rest(60);
  assert.ok(Math.hypot(...r.pos) < 0.005 && r.still && !r.settling, `wandered ${r.pos}`);
}

// 2. One move forward, with the sensor uncorrected.
{
  const sim = new Sim();
  sim.rest(2);
  sim.moveTo([0.5, 0, 0]);
  const r = sim.rest(1.5);
  assert.ok(dist(r.pos, [0.5, 0, 0]) < 0.06, `forward: ${r.pos.map((v) => v.toFixed(3))}`);
}

// 3. Front, left, up, then back to the start, for both uncorrected and calibrated.
for (const perfectCal of [false, true]) {
  const sim = new Sim({ perfectCal, seed: 3 });
  sim.rest(2);
  const wp = [[0.5, 0, 0], [0.5, 0.5, 0], [0, 0.5, 0.3], [0, 0, 0]];
  wp.forEach((w, i) => {
    sim.moveTo(w);
    const r = sim.rest(1.5);
    assert.ok(dist(r.pos, w) < (perfectCal ? 0.03 : 0.09), `route ${perfectCal ? 'calibrated' : 'raw'} ${i}: wanted ${w}, got ${r.pos.map((v) => v.toFixed(3))}`);
  });
}

// 4. Held tilted 30 degrees.
{
  const sim = new Sim({ seed: 5 });
  const q = eulerToQuat(0, 30, 0);
  sim.rest(2, q);
  sim.moveTo([0.4, 0, 0], 1.2, q);
  sim.moveTo([0.4, -0.4, 0], 1.2, q);
  const r = sim.rest(1.5, q);
  assert.ok(dist(r.pos, [0.4, -0.4, 0]) < 0.09, `tilted: ${r.pos.map((v) => v.toFixed(3))}`);
}

// 5. THE REGRESSION: turn the board over (or on its side) and set it down. It
// must settle and stay put instead of creeping away.
for (const [axis, angle] of [['roll', 45], ['roll', 90], ['roll', 180], ['pitch', -90], ['pitch', 60]]) {
  const sim = new Sim({ seed: 7 });
  sim.rest(2);
  sim.turn(axis, 0, angle);
  const end = { roll: [angle, 0, 0], pitch: [0, angle, 0] }[axis];
  const q = eulerToQuat(...end);
  const early = sim.rest(2, q);
  assert.ok(early.still, `${axis} ${angle}: still moving 2 s after being put down`);
  const late = sim.rest(10, q);
  assert.ok(late.still && dist(late.pos, early.pos) < 0.005, `${axis} ${angle}: crept ${(dist(late.pos, early.pos) * 100).toFixed(1)} cm while resting`);
  assert.ok(Math.hypot(...late.pos) < 0.2, `${axis} ${angle}: ended ${late.pos.map((v) => v.toFixed(2))} m from where it started`);
}

// 6. Being handled: twenty random turns and set-downs must never run away.
{
  const sim = new Sim({ seed: 11 });
  sim.rest(2);
  let ang = [0, 0, 0];
  const pick = () => [(sim.gauss() * 60) % 180, (sim.gauss() * 40) % 80, ang[2]];
  for (let i = 0; i < 20; i++) {
    const next = pick();
    for (let t = 0; t < 0.6; t += DT) {
      const u = t / 0.6, cur = ang.map((a, k) => a + (next[k] - a) * u);
      sim.step(eulerToQuat(...cur), [0, 0, 0], ang.map((a, k) => (next[k] - a) / 0.6));
    }
    ang = next;
    const r = sim.rest(1.5, eulerToQuat(...ang));
    assert.ok(r.still, `turn ${i}: never settled`);
    assert.ok(Math.hypot(...r.pos) < 0.6, `turn ${i}: ran off to ${r.pos.map((v) => v.toFixed(2))}`);
  }
  const start = sim.rest(0.1, eulerToQuat(...ang)).pos;
  const end = sim.rest(20, eulerToQuat(...ang)).pos;
  assert.ok(dist(start, end) < 0.005, 'kept creeping after the last set-down');
}

// 7. Connecting while the board is already moving: nothing is tracked until it is still.
{
  const sim = new Sim({ seed: 13 });
  sim.moveTo([0.6, 0, 0]);
  assert.deepEqual(sim.tracker.pos, [0, 0, 0], 'moved before the board had been still');
  sim.rest(2);
  sim.pos = [0, 0, 0];
  sim.moveTo([0.5, 0, 0]);
  const r = sim.rest(1.5);
  assert.ok(dist(r.pos, [0.5, 0, 0]) < 0.06, `after late start: ${r.pos.map((v) => v.toFixed(3))}`);
}

// 8. Forward means the way the nose pointed when zeroed.
{
  const q = eulerToQuat(0, 0, 90); // nose along the earth's +y
  const tracker = new Tracker();
  tracker.zeroHeading(q);
  const sim = new Sim({ seed: 9, tracker });
  sim.rest(2, q);
  sim.moveTo([0, 0.5, 0], 1.2, q); // the earth's +y is the board's forward
  const r = sim.rest(1.5, q);
  assert.ok(Math.abs(r.pos[0] - 0.5) < 0.06 && Math.abs(r.pos[1]) < 0.05, `heading zero: ${r.pos.map((v) => v.toFixed(3))}`);
}

// 9. The still detector reads about 1 at rest and far above the limit when moved.
{
  const sim = new Sim({ seed: 17 });
  const rest = sim.rest(2).detector;
  assert.ok(rest < 3, `detector at rest ${rest}`);
  let peak = 0;
  for (let t = 0; t < 1.2; t += DT) peak = Math.max(peak, sim.step(sim.q, [0.5 * s2(t / 1.2) / 1.44, 0, 0]).detector);
  assert.ok(peak > 100, `detector while moving peaked at ${peak}`);
}

// 10. Moved by hand: held in the hand between moves, with tremor. Every pause
// must lock, and sixteen moves in a row must not drift away.
{
  const sim = new Sim({ seed: 21 });
  sim.rest(2);
  sim.hand = true;
  const legs = [[0.3, 0, 0], [0.3, 0.3, 0], [0, 0.3, 0.2], [0, 0, 0]];
  let n = 0, worst = 0;
  for (let round = 0; round < 4; round++) {
    for (const w of legs) {
      sim.moveTo(w, 0.8);
      const r = sim.rest(3);
      n++;
      assert.ok(r.still && !r.lost, `hand move ${n}: not locked (${r.still ? 'lost track' : 'still moving'}), detector ${r.detector.toFixed(0)}`);
      worst = Math.max(worst, dist(r.pos, w));
      assert.ok(dist(r.pos, w) < 0.08, `hand move ${n}: wanted ${w}, got ${r.pos.map((v) => v.toFixed(3))}`);
    }
  }
  assert.ok(worst < 0.08, `worst hand error ${worst}`);
}

// 11. Picked up and held in a hand from the very start: it can still begin tracking.
{
  const sim = new Sim({ seed: 23 });
  sim.hand = true;
  const r = sim.rest(4);
  assert.ok(!r.settling && r.still, 'never began tracking a board held in a hand');
  sim.moveTo([0.4, 0, 0], 0.8);
  const end = sim.rest(3);
  assert.ok(dist(end.pos, [0.4, 0, 0]) < 0.08, `first move from a hand: ${end.pos.map((v) => v.toFixed(3))}`);
}

// 12. A whole minute in a hand: no creep.
{
  const sim = new Sim({ seed: 25 });
  sim.rest(2);
  sim.hand = true;
  sim.moveTo([0.3, 0.2, 0.1], 0.8);
  const a = sim.rest(3);
  const b = sim.rest(60);
  assert.ok(b.still && dist(a.pos, b.pos) < 0.03, `crept ${(dist(a.pos, b.pos) * 100).toFixed(1)} cm in a minute in a hand`);
}

// 13. A knock on the desk does not move it.
{
  const sim = new Sim({ seed: 27 });
  sim.rest(2);
  for (const knock of [[15, 0, 0], [0, -20, 0], [0, 0, 25]]) {
    sim.step(sim.q, knock);
    sim.step(sim.q, knock.map((v) => -v));
    sim.rest(1.5);
  }
  const r = sim.rest(2);
  assert.ok(r.still && Math.hypot(...r.pos) < 0.03, `knocks moved it ${r.pos.map((v) => v.toFixed(3))}`);
}

// 14. A long, violent shake: it must never fly off, it says the tracking is
// approximate, and once the shaking stops the position locks and tracking works again.
{
  const sim = new Sim({ seed: 29 });
  sim.rest(2);
  sim.moveTo([0.3, 0, 0]);
  const before = sim.rest(1.5).pos;
  assert.ok(dist(before, [0.3, 0, 0]) < 0.06, `the move before the shake was not tracked: ${before.map((v) => v.toFixed(3))}`);
  let sawFree = false, worst = 0;
  for (let t = 0; t < 12; t += DT) {
    const r = sim.step(sim.q, [12 * Math.sin(2 * Math.PI * 2.5 * t), 9 * Math.sin(2 * Math.PI * 1.7 * t), 6 * Math.sin(2 * Math.PI * 3.1 * t)], [40 * Math.sin(2 * Math.PI * 3 * t), 0, 0]);
    sawFree = sawFree || r.free;
    worst = Math.max(worst, dist(r.pos, before));
  }
  assert.ok(sawFree, 'a twelve second shake was never marked as approximate');
  assert.ok(worst <= 1.51, `flew ${worst.toFixed(2)} m away`); // the hard limit on one move is 1.5 m
  const after = sim.rest(4);
  assert.ok(after.still && !after.free, 'did not lock after the shake');
  sim.pos = [...after.pos];
  const target = [after.pos[0] + 0.3, after.pos[1], after.pos[2]];
  sim.moveTo(target);
  const next = sim.rest(1.5);
  assert.ok(dist(next.pos, target) < 0.08, `could not track a move after the shake: ${next.pos.map((v) => v.toFixed(3))}`);
}

// 15. WAVING THE BOARD AROUND without ever pausing, turning it as it goes, for
// forty seconds, with a shaky hand. The picture must keep following the movement
// (not freeze and not run away), and lock again when the board is put down.
{
  const sim = new Sim({ seed: 31 });
  sim.rest(2);
  sim.hand = true;
  const A = 0.25, w = 2 * Math.PI * 0.4;              // 25 cm swing, 0.4 Hz
  const truth = [], seen = [];
  let worst = 0;
  for (let t = 0; t < 40; t += DT) {
    // starts from rest: the swing grows over the first 3 s (env is the ramp, envD its rate)
    const env = Math.min(1, t / 3), envD = t < 3 ? 1 / 3 : 0;
    const wave = (amp, f, ph) => {
      const th = f * w * t + ph, s = Math.sin(th), c = Math.cos(th), W = f * w;
      const e = env, e1 = envD;
      return [amp * e * s, amp * (e * -W * W * s + 2 * e1 * W * c)];
    };
    const wx = wave(A, 1, 0), wy = wave(0.6 * A, 0.7, 1), wz = wave(0.4 * A, 0.5, 0);
    const x = wx[0], y = wy[0], z = wz[0];
    const acc = [wx[1], wy[1], wz[1]];
    const roll = 70 * Math.sin(0.3 * 2 * Math.PI * t), rollRate = 70 * 0.3 * 2 * Math.PI * Math.cos(0.3 * 2 * Math.PI * t);
    const r = sim.step(eulerToQuat(roll, 10 * Math.sin(0.2 * 2 * Math.PI * t), 0), acc, [rollRate, 0, 0]);
    if (t > 8) { truth.push([x, y, z]); seen.push(r.pos); }
    worst = Math.max(worst, Math.hypot(...r.pos));
  }
  assert.ok(worst < 1.2, `ran off to ${worst.toFixed(2)} m while being waved around`);
  // Does the picture follow the waving? Compare the swing of each axis with the true one.
  const swing = (rows, i) => { const v = rows.map((r) => r[i]); return Math.max(...v) - Math.min(...v); };
  const corr = (i) => {
    const a = truth.map((r) => r[i]), b = seen.map((r) => r[i]);
    const ma = a.reduce((x, y) => x + y) / a.length, mb = b.reduce((x, y) => x + y) / b.length;
    let sab = 0, saa = 0, sbb = 0;
    a.forEach((v, k) => { sab += (v - ma) * (b[k] - mb); saa += (v - ma) ** 2; sbb += (b[k] - mb) ** 2; });
    return sab / Math.sqrt(saa * sbb + 1e-12);
  };
  const ratio = swing(seen, 0) / swing(truth, 0);
  assert.ok(ratio > 0.25 && ratio < 2, `front-back swing was ${(ratio * 100).toFixed(0)}% of the real one`);
  assert.ok(corr(0) > 0.4, `front-back does not follow the waving (correlation ${corr(0).toFixed(2)})`);
  const end = sim.rest(4, eulerToQuat(0, 0, 0));
  assert.ok(end.still && !end.free, 'did not lock once the board was put down');
}

// 16. Waving that starts with the board already turned and held sideways.
{
  const sim = new Sim({ seed: 33 });
  sim.rest(2, eulerToQuat(-71, 0, 0));
  sim.hand = true;
  let moved = 0;
  for (let t = 0; t < 20; t += DT) {
    const acc = [0, 0.2 * (2 * Math.PI * 0.5) ** 2 * Math.cos(2 * Math.PI * 0.5 * t) * Math.min(1, t / 2), 0]; // starts from rest
    const r = sim.step(eulerToQuat(-71 + 30 * Math.sin(2 * Math.PI * 0.4 * t), 0, 0), acc, [30 * 2 * Math.PI * 0.4 * Math.cos(2 * Math.PI * 0.4 * t), 0, 0]);
    moved = Math.max(moved, Math.hypot(...r.pos));
  }
  assert.ok(moved > 0.05, `frozen: never moved more than ${(moved * 100).toFixed(1)} cm while being waved sideways`);
  assert.ok(moved < 1.2, `ran off to ${moved.toFixed(2)} m`);
}

console.log('tracker tests passed');
