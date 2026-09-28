// 3D position tracking from a gyro and an accelerometer, built for short moves
// with pauses in between (slide, stop, slide back, stop).
//
// Why it is built this way (the measurements are in analysis/report.html):
//   * Position comes from adding the accelerometer up twice, so any error in
//     "which way is down" becomes a fake push that grows with time squared
//     (1 degree of tilt error = 0.17 m/s^2 = 0.86 m after 10 s).
//   * An accelerometer cannot tell a push from a tilt, so while the board is
//     moving it is NOT used for tilt at all. Tilt is carried by the gyro alone,
//     which on this board drifts about 0.01 deg/s: about 0.1 deg over a 10 s move.
//   * The accelerometer is trusted only when the board is clearly still. Then
//     tilt is re-measured from gravity, the local gravity size and the gyro bias
//     are re-learned, and the speed is set to zero (zero-velocity update).
//   * When a move ends, the speed that is left over is pure error. If it grew
//     steadily through the move, the position error it caused is half of
//     (leftover speed x move time), and that is taken back off once, at the stop.
//
// Units: time in microseconds (uint32, wrap-safe differences), accelerometer in
// m/s^2 (specific force, sensor axes), gyro in rad/s (sensor axes). The navigation
// frame is x/y horizontal, z up. No Arduino code here, so it can be replayed on a
// PC (visualizer/tracker3d.py is a line-by-line Python copy).
#pragma once
#include <math.h>
#include <stdint.h>

namespace trk {

struct V3 {
  float x, y, z;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float norm(V3 a) { return sqrtf(dot(a, a)); }

struct Q {
  float w, x, y, z;
};
inline Q qmul(Q a, Q b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline Q qnorm(Q q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1e-12f) return {1, 0, 0, 0};
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}
inline Q qconj(Q q) { return {q.w, -q.x, -q.y, -q.z}; }
// Rotation vector (axis x angle, radians) to quaternion.
inline Q qexp(V3 r) {
  float a = norm(r);
  if (a < 1e-9f) return qnorm({1.0f, 0.5f * r.x, 0.5f * r.y, 0.5f * r.z});
  float s = sinf(0.5f * a) / a;
  return {cosf(0.5f * a), r.x * s, r.y * s, r.z * s};
}
// Sensor axes -> navigation axes.
inline V3 rotate(Q q, V3 v) {
  V3 u{q.x, q.y, q.z};
  V3 t = cross(u, v) * 2.0f;
  return v + t * q.w + cross(u, t);
}
inline V3 rotateInv(Q q, V3 v) { return rotate(qconj(q), v); }
inline float yawOf(Q q) { return atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)); }
inline Q yawQuat(float yaw) { return {cosf(0.5f * yaw), 0, 0, sinf(0.5f * yaw)}; }

struct Params {
  // Still: over the last 0.1 s the accelerometer wobbles less than accStill (rms of
  // the vector, m/s^2) and the gyro turns slower than gyroStill on average and
  // gyroStillMax at most (rad/s). Loose enough that a board held steady in a hand
  // counts (a real hand measured 0.1-0.3 m/s^2 and 2-8 deg/s), since a hand never
  // becomes table-still and without stops the position runs away.
  float accStill = 0.35f;
  float gyroStill = 6.0f * 0.0174533f;
  float gyroStillMax = 15.0f * 0.0174533f;
  // While still, a steady push this big (m/s^2, the last 20 ms against the resting
  // reading) also counts as the start of a move, for pushes too smooth to wobble.
  float accShift = 0.40f;
  // A move ends after this long of stillness (s); longer if the speed estimate is
  // still high, so a smooth part of a real move is not mistaken for a stop.
  float holdSlow = 0.08f;
  float holdFast = 0.40f;
  float vFast = 0.25f;  // m/s
  // Re-learning at rest (time constants, s).
  float tauTilt = 0.10f;
  float tauG = 0.30f;
  float tauRef = 0.50f;
  float tauBias = 2.0f;
  // The gyro bias is learned only when the board is this still (on a table, not in a hand).
  float biasAcc = 0.05f;
  float biasGyro = 1.0f * 0.0174533f;
  float biasAfter = 0.5f;  // s of stillness first
  float settleTime = 1.0f;  // s of stillness before tracking starts
  float backfill = 0.15f;   // s of samples re-added when a move is noticed
  // Past this long (s) the leftover speed no longer grew steadily, so taking half of
  // it off would only make the position jump; longer moves are left as they are.
  float maxCorrect = 4.0f;
  // Past this long (s) of moving without a stop the position is paused ("lost")
  // rather than left to run off by metres; it carries on from there at the next stop.
  float maxMove = 3.0f;
};

enum State : uint8_t { SETTLING = 0, STILL = 1, MOVING = 2 };

class Tracker {
 public:
  static const int WIN = 40;    // samples in the stillness window (0.1 s at 400 Hz)
  static const int RECENT = 8;  // samples in the steady-push check (20 ms)
  static const int HIST = 96;   // samples kept for backfill (0.24 s)

  Params P;
  Q q{1, 0, 0, 0};       // sensor -> navigation
  V3 bg{0, 0, 0};        // gyro bias, rad/s
  V3 p{0, 0, 0}, v{0, 0, 0};
  float gLocal = 9.80665f;
  V3 fRef{0, 0, 9.80665f};  // resting accelerometer reading (sensor axes)
  State state = SETTLING;
  bool lost = false;  // the current move has gone on too long; position paused
  uint32_t moves = 0;
  // the last finished move, for display and tuning
  float lastMoveTime = 0;
  V3 lastMoveVres{0, 0, 0};
  V3 lastMoveCorr{0, 0, 0};
  // detector values from the latest accelerometer sample
  float accStd = 0, gyroMean = 0, gyroMax = 0, shift = 0;

  void gyro(uint32_t t, V3 wRaw) {
    float dt = haveG ? (int32_t)(t - tG) * 1e-6f : 0.0f;
    tG = t;
    haveG = true;
    if (dt < 0.0f || dt > 0.05f) dt = 0.0f;
    V3 w = wRaw - bg;
    if (state != MOVING) {
      // At rest any turn about the vertical is gyro bias, not motion: hold heading.
      V3 up = rotateInv(q, V3{0, 0, 1});
      w = w - up * dot(w, up);
    }
    gRaw[gi] = wRaw;
    gMag[gi] = norm(w);
    gi = (gi + 1) % WIN;
    if (gN < WIN) gN++;
    q = qnorm(qmul(q, qexp(w * dt)));
    wLast = w;
  }

  void accel(uint32_t t, V3 f) {
    float dt = haveA ? (int32_t)(t - tA) * 1e-6f : 0.0f;
    tA = t;
    haveA = true;
    if (dt < 0.0f || dt > 0.05f) dt = 0.0f;
    aWin[ai] = f;
    ai = (ai + 1) % WIN;
    if (aN < WIN) aN++;

    // Orientation at this sample's time: the latest gyro orientation, carried on by
    // the latest turn rate for the few milliseconds between the two streams.
    Q qa = q;
    if (haveG) {
      float d = (int32_t)(t - tG) * 1e-6f;
      if (d > 0.005f) d = 0.005f;
      if (d < -0.005f) d = -0.005f;
      qa = qnorm(qmul(q, qexp(wLast * d)));
    }
    V3 an = rotate(qa, f) - V3{0, 0, gLocal};
    hist[hi] = {t, an, dt};
    hi = (hi + 1) % HIST;
    if (hN < HIST) hN++;

    // Stillness detector.
    V3 fMean{0, 0, 0}, wRawMean{0, 0, 0}, recent{0, 0, 0};
    for (int k = 0; k < aN; k++) fMean = fMean + aWin[k];
    fMean = fMean * (1.0f / aN);
    float var = 0;
    for (int k = 0; k < aN; k++) {
      V3 d = aWin[k] - fMean;
      var += dot(d, d);
    }
    accStd = sqrtf(var / aN);
    gyroMean = 0;
    gyroMax = 0;
    for (int k = 0; k < gN; k++) {
      gyroMean += gMag[k];
      if (gMag[k] > gyroMax) gyroMax = gMag[k];
      wRawMean = wRawMean + gRaw[k];
    }
    if (gN > 0) {
      gyroMean /= gN;
      wRawMean = wRawMean * (1.0f / gN);
    }
    int nr = aN < RECENT ? aN : RECENT;
    for (int k = 1; k <= nr; k++) recent = recent + aWin[(ai - k + WIN) % WIN];
    recent = recent * (1.0f / nr);
    shift = norm(recent - fRef);
    bool full = aN >= WIN && gN >= WIN;
    bool quiet = full && accStd < P.accStill && gyroMean < P.gyroStill && gyroMax < P.gyroStillMax;

    if (state == SETTLING) {
      quietT = quiet ? quietT + dt : 0.0f;
      if (quietT >= P.settleTime) {
        bg = wRawMean;
        q = Q{1, 0, 0, 0};
        alignTilt(fMean, 1.0f);
        q = qnorm(qmul(yawQuat(-yawOf(q)), q));  // start facing heading 0
        gLocal = norm(fMean);
        fRef = fMean;
        v = V3{0, 0, 0};
        state = STILL;
        stillT = 0;
      }
    } else if (state == STILL) {
      if (!full) {
        // windows still filling (only after a restart of the tracker): wait
      } else if (!quiet || shift > P.accShift) {
        startMove(t);
      } else {
        stillT += dt;
        v = V3{0, 0, 0};
        alignTilt(fMean, fminf(1.0f, dt / P.tauTilt));
        gLocal += fminf(1.0f, dt / P.tauG) * (norm(fMean) - gLocal);
        fRef = fRef + (fMean - fRef) * fminf(1.0f, dt / P.tauRef);
        if (stillT > P.biasAfter && accStd < P.biasAcc && gyroMean < P.biasGyro)
          bg = bg + (wRawMean - bg) * fminf(1.0f, dt / P.tauBias);
      }
    } else {
      if (moveT > P.maxMove) {
        lost = true;
        v = V3{0, 0, 0};
      } else {
        integrate(an, dt);
      }
      moveT += dt;
      quietT = quiet ? quietT + dt : 0.0f;
      float hold = norm(v) < P.vFast ? P.holdSlow : P.holdFast;
      if (quietT >= hold) endMove(fMean);
    }
  }

 private:
  struct Hist {
    uint32_t t;
    V3 a;
    float dt;
  };
  V3 aWin[WIN], gRaw[WIN];
  float gMag[WIN];
  Hist hist[HIST];
  int ai = 0, aN = 0, gi = 0, gN = 0, hi = 0, hN = 0;
  uint32_t tA = 0, tG = 0;
  bool haveA = false, haveG = false;
  V3 wLast{0, 0, 0};
  float quietT = 0, stillT = 0, moveT = 0;

  // Turns the orientation so that "up" in sensor axes moves a fraction k of the
  // way toward the measured resting reading. Only tilt changes, never heading.
  void alignTilt(V3 fMean, float k) {
    float n = norm(fMean);
    if (n < 1e-3f) return;
    V3 meas = fMean * (1.0f / n);
    V3 pred = rotateInv(q, V3{0, 0, 1});
    V3 e = cross(meas, pred);
    float s = norm(e);
    if (s < 1e-9f) return;
    float angle = atan2f(s, dot(meas, pred));
    q = qnorm(qmul(q, qexp(e * (angle * k / s))));
  }

  void integrate(V3 a, float dt) {
    V3 v0 = v;
    v = v + a * dt;
    p = p + (v0 + v) * (0.5f * dt);
  }

  // The window only notices a move after it has begun, so the last backfill
  // seconds are added again from the kept samples.
  void startMove(uint32_t t) {
    state = MOVING;
    lost = false;
    v = V3{0, 0, 0};
    moveT = 0;
    quietT = 0;
    uint32_t from = t - (uint32_t)(P.backfill * 1e6f);
    for (int k = hN; k >= 1; k--) {
      const Hist &h = hist[(hi - k + HIST) % HIST];
      if ((int32_t)(h.t - from) < 0) continue;
      integrate(h.a, h.dt);
      moveT += h.dt;
    }
  }

  void endMove(V3 fMean) {
    lastMoveVres = v;
    lastMoveTime = moveT;
    lastMoveCorr = (!lost && moveT <= P.maxCorrect) ? v * (-0.5f * moveT) : V3{0, 0, 0};
    lost = false;
    p = p + lastMoveCorr;
    v = V3{0, 0, 0};
    moves++;
    state = STILL;
    stillT = quietT;
    fRef = fMean;
  }
};

}  // namespace trk
