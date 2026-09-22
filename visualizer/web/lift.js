// Follows how far the board has been lifted or lowered. Only height is tracked:
// gravity gives a solid up direction, so up and down are the one axis that can be
// followed well from an accelerometer alone. Sideways position is not attempted.
import { mul, conj, normalize, eulerToQuat, quatToEuler } from './orientation.js';

const G = 9.80665;

const DEFAULTS = {
  window: 15, // samples (0.3 s at 50 Hz) used to decide the board is still
  gyroQuiet: 6, // dps
  accQuiet: 0.5, // m/s^2, spread of the vertical acceleration when still
  handSpread: 2.5, // m/s^2, largest jitter that still counts as a hand holding the board
  handMean: 0.25, // m/s^2, but its average must be this close to the resting offset
  handTime: 0.8, // s, a shaky hand has to hold this long before it counts as still
  handSpeed: 0.1, // m/s, and it must not be going anywhere
  deadband: 0.1, // m/s^2, smaller pushes are treated as noise
  stillTime: 0.3, // s the board must be quiet before it counts as still
  leak: 30, // s, speed fades very slowly so an error cannot last forever
  biasWindow: 0.1, // m/s^2, the resting offset only follows readings this close to it
  biasTau: 0.4, // s, how fast the resting offset is learned
  maxHeight: 0.75, // m, the room's half height
};

const rotate = (q, [vx, vy, vz]) => {
  const t = mul(mul(q, [0, vx, vy, vz]), conj(q));
  return [t[1], t[2], t[3]];
};

export class Lift {
  constructor(options = {}) {
    this.o = { ...DEFAULTS, ...options };
    this.yaw0 = 0;
    this.reset();
  }

  reset() {
    this.z = 0;
    this.v = 0;
    this.bias = 0;
    this.haveBias = false;
    this.win = [];
    this.stillT = 0;
    this.handT = 0;
    this.moveT = 0;
    this.hist = [];
    this.wasStill = false;
    this.lastMs = null;
  }

  zeroPosition() {
    this.z = 0;
    this.v = 0;
  }

  zeroHeading(q) {
    this.yaw0 = quatToEuler(q).yaw;
  }

  displayQuat(q) {
    return normalize(mul(eulerToQuat(0, 0, -this.yaw0), q));
  }

  update({ ms, q, f, g }) {
    const o = this.o;
    let dt = this.lastMs === null ? 0 : (ms - this.lastMs) / 1000;
    this.lastMs = ms;
    if (!(dt > 0 && dt < 0.2)) dt = 0;

    const az = (rotate(q, f)[2] - 1) * G; // vertical acceleration, gravity taken off
    this.win.push({ az, gyro: Math.hypot(g[0], g[1], g[2]) });
    if (this.win.length > o.window) this.win.shift();
    const full = this.win.length >= o.window;

    let mean = 0, lo = Infinity, hi = -Infinity, gyro = 0;
    for (const s of this.win) {
      mean += s.az / this.win.length;
      gyro += s.gyro / this.win.length;
      lo = Math.min(lo, s.az);
      hi = Math.max(hi, s.az);
    }
    // A hand's tremor makes the readings jump but averages out, so once the offset
    // is known, a small average counts as still even if the readings are shaky.
    const spread = hi - lo;
    const calm = full && gyro < o.gyroQuiet && spread < o.accQuiet;
    const quiet = full && gyro < o.gyroQuiet && (spread < o.accQuiet || (spread < o.handSpread && (!this.haveBias || (Math.abs(mean - this.bias) < o.handMean && Math.abs(this.v) < o.handSpeed))));
    this.stillT = calm ? this.stillT + dt : 0;
    this.handT = quiet ? this.handT + dt : 0;
    const still = this.stillT >= o.stillTime || this.handT >= o.handTime;

    if (still) {
      // Resting: speed is zero. The speed left over at the end of a move is the
      // error that built up during it, growing steadily, so half of it times the
      // duration of the move is taken off the height.
      if (this.moveT > 0.2 && this.moveT < 8) this.z -= 0.5 * this.v * Math.max(0, this.moveT - o.stillTime);
      this.moveT = 0;
      this.v = 0;
      this.hist.push({ a: az - this.bias, dt });
      if (this.hist.length > 20) this.hist.shift();
      this.wasStill = true;
      if (!this.haveBias) this.bias = mean;
      else if (Math.abs(mean - this.bias) < o.biasWindow) this.bias += Math.min(1, dt / o.biasTau) * (mean - this.bias);
      this.haveBias = true;
    } else if (this.haveBias && dt > 0) {
      if (this.wasStill) {
        // The move was under way a little before it could be told from rest: catch up.
        for (const h of this.hist) {
          this.v += (Math.abs(h.a) < o.deadband ? 0 : h.a) * h.dt;
          this.z += this.v * h.dt;
          this.moveT += h.dt;
        }
        this.hist = [];
        this.wasStill = false;
      }
      this.moveT += dt;
      // Integrates even during the first moments of a move, before it is known to be one.
      let a = az - this.bias;
      if (Math.abs(a) < o.deadband) a = 0;
      this.v = (this.v + a * dt) * Math.exp(-dt / o.leak);
      this.z += this.v * dt;
      if (Math.abs(this.z) > o.maxHeight) {
        this.z = Math.sign(this.z) * o.maxHeight;
        this.v = 0;
      }
    }
    return {
      pos: [0, 0, this.z], vel: [0, 0, this.v], still, settling: !this.haveBias,
      free: false, detector: gyro, full,
    };
  }
}
