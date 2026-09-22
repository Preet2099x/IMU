// Estimates where the board is by integrating its acceleration.
//
// Integrating acceleration twice drifts quickly, so this only works for short
// moves, and it leans on the board pausing between them:
//   - Still or moving is decided from AVERAGES over the last 0.3 s: the average
//     rotation rate and the average leftover acceleration (what is left after
//     taking off gravity and the error measured at the last rest). A hand's
//     tremor averages out, so a board held in a hand still counts as still,
//     while a steady push doesn't. A board that has been turned since the last
//     rest can't use the leftover acceleration (the gravity error moved with it),
//     so it only needs to stop turning.
//   - While still, position is frozen and speed is forced to zero (zero-velocity
//     update), and the leftover gravity error is measured for the next move.
//   - When a move ends, the speed error it built up is spread back over it.
//   - A short move is tracked precisely. If the board keeps moving for longer
//     than that (waved around, turned as it goes), the error can no longer be
//     kept small, so tracking switches to "free motion": speed fades and the
//     position is drawn gently back toward where the move began. It stays
//     responsive and can't run away, it just isn't exact. When the board stops,
//     the position locks wherever it has got to.
//
// Inputs are 50 Hz samples: device time (ms), orientation quaternion [w,x,y,z]
// (sensor to earth), the accelerometer in g in the sensor frame (calibrated),
// and the rotation rate in deg/s. Position is in metres: x front, y left, z up,
// measured from the heading the board had when zeroed.
import { mul, conj, normalize, eulerToQuat, quatToEuler } from './orientation.js';

const G = 9.80665;

const DEFAULTS = {
  window: 15, // samples (0.3 s) the still tests average over
  sigmaAcc: 0.05, // m/s^2, expected noise of a still accelerometer (for the variation figure)
  sigmaGyro: 1.0, // deg/s, same for the gyro
  gyroQuiet: 3, // deg/s: average rotation below this counts as still
  gyroMove: 6, // ... and above this as moving
  accQuiet: 0.2, // m/s^2: average leftover acceleration below this counts as still
  accQuietDesk: 0.06, // ... and for a board that was resting on a table (its start threshold is lower, so this must be too)
  accMove: 0.3, // ... and above this as pushed (in a hand, where sway adds a little)
  accMoveDesk: 0.12, // ... and above this for a board resting on a table, where there is far less noise
  jolt: 25, // variation figure that, for a board that had been resting on a table, means it has been picked up
  deskTime: 0.5, // s of proper stillness that makes a board count as resting on a table
  shake: 400, // variation figure above which nothing could be resting, not even in a hand
  speedQuiet: 0.25, // m/s: a move only ends once the estimated speed is this low ...
  speedQuietDesk: 0.06, // ... and this low if it began from a table, where slow careful moves peak below 0.25
  longQuiet: 0.8, // ... or it has been not turning with a steady average acceleration this long (s), whatever the speed says
  steadyTol: 0.2, // m/s^2: how little the average acceleration may change over 0.4 s to count as steady
  stillTime: 0.15, // s of quiet before a move counts as over
  turnedDeg: 5, // turned by more than this since the last rest: that rest's gravity error is out of date
  settleTime: 0.3, // s of stillness before the leftover gravity is measured
  strict: 6, // variation figure of a board resting on a table
  armTime: 0.6, // s of that needed at the start before tracking begins ...
  handArmTime: 2, // ... or this long (s) of the board just not turning, if it is in a hand
  backfill: 0.6, // s of samples replayed when a move starts
  biasTau: 0.5, // s, how fast the leftover-gravity estimate follows
  biasLag: 0.8, // s, when a move starts the estimate goes back to what it was this long ago
  maxSpeed: 3, // m/s, faster than this is a glitch
  freeAfter: 4, // s of continuous movement after which it is approximate (free motion)
  slowTau: 0.8, // s, in free motion the slow average of the acceleration is treated as error
  leak: 1.2, // s, in free motion the speed fades with this time constant ...
  spring: 1.5, // 1/s^2, ... and the position is drawn back toward where the move began
  maxReach: 1.5, // m, a single move never takes the position further than this from where it began
  driftFix: true,
};

const rotate = ([w, x, y, z], [vx, vy, vz]) => {
  const t = mul(mul([w, x, y, z], [0, vx, vy, vz]), conj([w, x, y, z]));
  return [t[1], t[2], t[3]];
};
const norm = (v) => Math.hypot(v[0], v[1], v[2]);

export class Tracker {
  constructor(options = {}) {
    this.o = { ...DEFAULTS, ...options };
    this.yaw0 = 0;
    this.reset();
  }

  reset() {
    this.pos = [0, 0, 0];
    this.vel = [0, 0, 0];
    // Start by waiting for the board to be still: its speed is unknown until then.
    this.state = 'settling';
    this.free = false; // approximate tracking of long continuous movement
    this.quietT = 0;
    this.handT = 0;
    this.longQuietT = 0;
    this.meanTrail = []; // recent window averages of the acceleration, oldest first
    this.deskT = 0;
    this.stillT = 0;
    this.moveT = 0;
    this.posBefore = [0, 0, 0]; // where the current move began
    this.moveOnDesk = false;
    this.sinceStart = 0; // time since the move was noticed
    this.biasW = [0, 0, 0]; // leftover gravity in the earth frame, measured at rest
    this.haveBias = false;
    this.biasTrail = []; // recent values of the estimate, oldest first
    this.qBias = [1, 0, 0, 0]; // orientation the gravity error was measured in
    this.win = [];
    this.history = [];
    this.lastMs = null;
  }

  // Puts the board back at the middle, keeping what it has learned.
  zeroPosition() {
    this.pos = [0, 0, 0];
    this.vel = [0, 0, 0];
    this.posBefore = [0, 0, 0];
  }

  // Forward is wherever the nose points right now.
  zeroHeading(q) {
    this.yaw0 = quatToEuler(q).yaw;
  }

  // Earth-frame acceleration turned so that x is the zeroed heading.
  toScene(a) {
    const c = Math.cos(-this.yaw0 * Math.PI / 180), s = Math.sin(-this.yaw0 * Math.PI / 180);
    return [c * a[0] - s * a[1], s * a[0] + c * a[1], a[2]];
  }

  // Orientation to draw: tilt as measured, heading measured from the zeroed one.
  displayQuat(q) {
    return normalize(mul(eulerToQuat(0, 0, -this.yaw0), q));
  }

  // Averages over the window, and a variation figure (about 1 for a board on a
  // table, hundreds when it is shaken) that also tells the desk from a hand.
  windowStats() {
    const n = this.win.length;
    const meanA = [0, 0, 0], meanF = [0, 0, 0], meanG = [0, 0, 0];
    for (const s of this.win) {
      for (let i = 0; i < 3; i++) {
        meanA[i] += s.a[i] / n;
        meanF[i] += s.f[i] / n;
        meanG[i] += s.g[i] / n;
      }
    }
    let variation = 0;
    for (const s of this.win) {
      const dx = s.f[0] - meanF[0], dy = s.f[1] - meanF[1], dz = s.f[2] - meanF[2];
      variation += (dx * dx + dy * dy + dz * dz) / (this.o.sigmaAcc ** 2) + (s.g[0] ** 2 + s.g[1] ** 2 + s.g[2] ** 2) / (this.o.sigmaGyro ** 2);
    }
    return { meanA, meanRate: norm(meanG), variation: variation / n };
  }

  turnedSinceRest(q) {
    const d = Math.abs(q[0] * this.qBias[0] + q[1] * this.qBias[1] + q[2] * this.qBias[2] + q[3] * this.qBias[3]);
    return 2 * Math.acos(Math.min(1, d)) * (180 / Math.PI) > this.o.turnedDeg;
  }

  learnGravityError(a, dt, q) {
    this.qBias = q;
    if (!this.haveBias) {
      this.biasW = [...a];
      this.haveBias = true;
      this.biasTrail = []; // whatever it was before is out of date
      return;
    }
    const k = Math.min(1, dt / this.o.biasTau);
    this.biasW = this.biasW.map((b, i) => b + k * (a[i] - b));
  }

  // In free motion the speed fades and the position is pulled back toward where
  // the move began, so an error can never build up without limit.
  // Returns true if the speed had to be cut to the limit (which means it went wrong).
  integrate(a, dt, free = false) {
    if (!free) this.aSlow = null;
    else if (!this.aSlow) this.aSlow = [0, 0, 0];
    else this.aSlow = this.aSlow.map((m, i) => m + Math.min(1, dt / this.o.slowTau) * (a[i] - m));
    for (let i = 0; i < 3; i++) {
      // a steady push over seconds is far more likely a tilt/gravity error than real, so drop it
      const acc = free ? a[i] - this.aSlow[i] - this.o.spring * (this.pos[i] - this.posBefore[i]) : a[i];
      this.vel[i] += acc * dt;
      if (free) this.vel[i] *= Math.exp(-dt / this.o.leak);
      this.pos[i] += this.vel[i] * dt;
    }
    let wentWrong = false;
    const speed = norm(this.vel);
    if (speed > this.o.maxSpeed) {
      this.vel = this.vel.map((v) => (v * this.o.maxSpeed) / speed);
      wentWrong = true;
    }
    // A hard limit on how far one move can go, whatever the estimate says.
    const away = [0, 1, 2].map((i) => this.pos[i] - this.posBefore[i]);
    const reach = norm(away);
    if (reach > this.o.maxReach) {
      const out = away.map((v) => v / reach);
      this.pos = this.posBefore.map((b, i) => b + out[i] * this.o.maxReach);
      const outward = this.vel[0] * out[0] + this.vel[1] * out[1] + this.vel[2] * out[2];
      if (outward > 0) this.vel = this.vel.map((v, i) => v - outward * out[i]);
      wentWrong = true;
    }
    return wentWrong;
  }

  update({ ms, q, f, g }) {
    const o = this.o;
    let dt = this.lastMs === null ? 0 : (ms - this.lastMs) / 1000;
    this.lastMs = ms;
    if (!(dt > 0 && dt < 0.2)) dt = 0;

    // Acceleration in the earth frame, gravity taken off (still including the
    // sensor's gravity error, which is measured at rest).
    const w = rotate(q, f);
    const a = [w[0] * G, w[1] * G, (w[2] - 1) * G];
    this.win.push({ f: f.map((v) => v * G), g, a });
    if (this.win.length > o.window) this.win.shift();
    const full = this.win.length >= o.window;
    const st = this.windowStats();

    this.biasTrail.push({ b: [...this.biasW], dt });
    let trailT = 0, cut = this.biasTrail.length;
    while (cut > 0 && trailT < o.biasLag) trailT += this.biasTrail[--cut].dt;
    this.biasTrail = this.biasTrail.slice(cut);

    this.meanTrail.push({ m: [...st.meanA], dt });
    let spanT = 0, drop = this.meanTrail.length;
    while (drop > 0 && spanT < 0.4) spanT += this.meanTrail[--drop].dt;
    this.meanTrail = this.meanTrail.slice(drop);
    const older = this.meanTrail[0].m;
    // A resting board's average acceleration holds still; a move's keeps changing.
    const steady = full && spanT >= 0.35 && norm([st.meanA[0] - older[0], st.meanA[1] - older[1], st.meanA[2] - older[2]]) < o.steadyTol;

    const left = [a[0] - this.biasW[0], a[1] - this.biasW[1], a[2] - this.biasW[2]];
    const scene = this.toScene(left);
    const avgLeft = norm([st.meanA[0] - this.biasW[0], st.meanA[1] - this.biasW[1], st.meanA[2] - this.biasW[2]]);
    const turned = this.turnedSinceRest(q);

    // Not turning, and not shaken beyond what a hand could hold still.
    const rotQuiet = full && st.meanRate < o.gyroQuiet && st.variation < o.shake;

    if (this.state === 'settling') {
      // Not turning proves nothing about being pushed, so a board on a table has
      // to be properly quiet, and one in a hand has to stay unturned for longer.
      this.quietT = rotQuiet && st.variation < o.strict ? this.quietT + dt : 0;
      this.handT = rotQuiet && steady ? this.handT + dt : 0;
      if (this.quietT >= o.armTime || this.handT >= o.handArmTime) {
        this.biasW = [...st.meanA]; // what the window has been reading is the gravity error
        this.haveBias = true;
        this.qBias = q;
        this.biasTrail = [];
        this.state = 'still';
        this.stillT = 0;
      }
    } else if (this.state === 'still') {
      this.stillT += dt;
      const onDesk = this.deskT >= o.deskTime;
      const pushLimit = onDesk ? o.accMoveDesk : o.accMove;
      const hadBias = this.haveBias; // a fresh measurement below can't be judged against the old one
      if (rotQuiet && this.stillT > o.settleTime && (!this.haveBias || avgLeft < pushLimit)) this.learnGravityError(st.meanA, dt, q); // the average, so a hand's tremor cancels out
      // Keep the last few samples so a move that just began can be replayed.
      this.history.push({ a, dt });
      let total = 0, from = this.history.length;
      while (from > 0 && total < o.backfill) total += this.history[--from].dt;
      this.history = this.history.slice(from);

      // A board on a table shows almost no variation, so any sudden variation
      // means it was picked up or pushed. In a hand there is always some, so
      // there only a real average push or turn counts.
      if (st.variation < o.strict) this.deskT += dt;
      else if (st.variation > o.jolt) this.deskT = 0;

      const spun = full && st.meanRate > o.gyroMove;
      const pushed = hadBias && this.stillT > o.settleTime && avgLeft > pushLimit;
      const jolted = full && onDesk && st.variation > o.jolt;
      if (spun || pushed || jolted || (full && st.variation > o.shake)) {
        this.state = 'moving';
        this.startReason = spun ? 'turned' : pushed ? `pushed ${avgLeft.toFixed(2)} > ${pushLimit}` : jolted ? 'jolted' : 'shaken';
        this.deskT = 0;
        // The estimate was still following the start of the push: go back to
        // what it was before that, and redo the replay with it.
        if (this.haveBias && this.biasTrail.length) this.biasW = [...this.biasTrail[0].b];
        this.posBefore = [...this.pos];
        this.moveOnDesk = onDesk;
        this.moveT = 0;
        this.quietT = 0;
        this.longQuietT = 0;
        this.free = false;
        this.sinceStart = 0;
        this.vel = [0, 0, 0];
        for (const h of this.history) {
          this.free = this.integrate(this.toScene([h.a[0] - this.biasW[0], h.a[1] - this.biasW[1], h.a[2] - this.biasW[2]]), h.dt) || this.free;
          this.moveT += h.dt;
        }
        this.history = [];
      } else {
        this.vel = [0, 0, 0];
      }
    } else {
      if (this.integrate(scene, dt, this.free)) this.free = true;
      this.moveT += dt;
      this.sinceStart += dt;
      if (this.sinceStart > o.freeAfter) this.free = true;
      {
        // A move ends once the board has stopped turning, no push is left over and
        // the speed says it can have stopped. If a push is left over that never
        // changes, the gravity error measured at the last rest is out of date: that
        // is a board at rest too, just noticed more slowly.
        const slow = norm(this.vel) < (this.moveOnDesk ? o.speedQuietDesk : o.speedQuiet);
        const noPush = avgLeft < (this.moveOnDesk ? o.accQuietDesk : o.accQuiet) || turned || !this.haveBias;
        this.quietT = rotQuiet && slow && noPush ? this.quietT + dt : 0;
        this.longQuietT = rotQuiet && steady ? this.longQuietT + dt : 0;
        if (this.quietT >= o.stillTime || this.longQuietT >= o.longQuiet) {
          if (turned || this.free || this.quietT < o.stillTime) this.haveBias = false; // measure it afresh
          if (o.driftFix && !this.free) {
            // A steady error makes the speed error grow linearly and the position
            // error quadratically, so half of the speed error times the move time
            // is what to take back.
            const moved = Math.max(0, this.moveT - Math.max(this.quietT, this.longQuietT));
            for (let i = 0; i < 3; i++) this.pos[i] -= 0.5 * this.vel[i] * moved;
          }
          this.vel = [0, 0, 0];
          this.free = false;
          this.state = 'still';
          this.stillT = Math.max(this.quietT, this.longQuietT);
          this.deskT = 0;
          this.history = [];
        }
      }
    }
    return {
      pos: [...this.pos], vel: [...this.vel], still: this.state !== 'moving',
      settling: this.state === 'settling', free: this.free, detector: st.variation, full,
    };
  }
}
