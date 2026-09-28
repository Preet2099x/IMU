"""The RIDI tracker on the live board, drawn in the same 3D window as visualizer/track3d.py.

    python ridi_adapter/ridi_live.py                      # live: finds the ESP32, RIDI + rest detection
    python ridi_adapter/ridi_live.py --pure               # RIDI alone (drifts on its own when the board rests)
    python ridi_adapter/ridi_live.py --offline LOG --save out.png   # run it on a saved recording

RIDI is made for CONTINUOUS movement: someone walking steadily while holding the device. Hold the
board like a phone lying flat (top face up, nose pointing the way you walk), keep it still for one
second at the start so it can settle, then walk. It reports the speed of a walker, so it does not suit
short hand slides.

How it works (RIDI: Robust IMU Double Integration, Yan, Shan, Furukawa 2018):
  1. Every 0.15 s, RIDI's trained model looks at the last second of the IMU (gyro and linear
     acceleration, 200 Hz, gravity-aligned) and says how fast the board is moving, sideways and
     forward, in a gravity-stabilized frame. That becomes a room-frame velocity.
  2. The accelerometer is integrated to speed as usual, and RIDI's correction step then solves for
     the accelerometer bias (a piecewise-linear bias over a rolling 5 s window) that makes the integrated
     speed match the model's speed at those moments. Position integrates the corrected speed.
Orientation comes from the board's own gyro tracker (src/tracker3d.h, copied in visualizer/tracker3d.py).

One thing is added to RIDI: while our rest detector says the board is resting, its speed is pinned to zero
(and the model's target speed is zero). RIDI alone reports a walking speed even for a board sitting still:
measured on this board, it wandered 8 m in 29 s of rest. Walking is unaffected: a walking hand is never
detected as resting. --pure switches the pin off. See ridi_adapter/README.md for how the trained model
behaves on the board's movements.
"""
import argparse
import collections
import functools
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for p in (HERE, ROOT, ROOT / "visualizer"):
    sys.path.insert(0, str(p))
import ridi_features as rf  # noqa: E402
import ridi_model as rm  # noqa: E402
import tracker3d as t3  # noqa: E402


class RidiEngine:
    """Feeds on 200 Hz frames (device-frame gyro, accelerometer, orientation, gravity) and keeps
    RIDI's speed, corrected bias and position."""

    def __init__(self, cls=0, pure=False, window=1000, knot=50, opt_every=100, ridge=1e-2):
        self.cls, self.pure = cls, pure
        self.window, self.knot, self.opt_every, self.ridge = window, knot, opt_every, ridge
        self.reg = [rm.load_regressor(cls, ch, verbose=True) for ch in (0, 1)]
        self.t, self.a, self.bias, self.v, self.p = [], [], [], [], []
        self.held = []           # per frame: True while the board rests and its speed is pinned to zero
        self.feat = collections.deque(maxlen=rf.WINDOW)
        self.cons = []            # (frame index, target world velocity)
        self.n = 0
        self.last_opt = 0
        self.last_speed_model = np.zeros(2)   # the model's last (x, z) answer, for display

    # ------------------------------------------------------------------ per frame
    def frame(self, t, gyro, acc, q, grav, still):
        lin = acc - grav
        gg = rf.align_with_gravity(gyro[None], grav[None])[0]
        lg = rf.align_with_gravity(lin[None], grav[None])[0]
        aw = rf.quat_rotate(q[None], lin[None])[0]
        if self.n == 0:
            v = np.zeros(3)
            p = np.zeros(3)
            b = np.zeros(3)
        else:
            dt = t - self.t[-1]
            if not 0 < dt < 0.05:
                dt = 0.005
            b = self.bias[-1]
            v = self.v[-1] + (self.a[-1] - b) * dt
            p = self.p[-1] + self.v[-1] * dt
        hold = still and not self.pure
        if hold:
            v = np.zeros(3)
        self.t.append(t)
        self.a.append(aw)
        self.bias.append(b.copy())
        self.held.append(hold)
        self.v.append(v)
        self.p.append(p)
        self.feat.append(np.concatenate([gg, lg]))
        self.n += 1

        if len(self.feat) == rf.WINDOW and self.n % rf.STEP == 0:
            X = rf.gaussian_smooth(np.array(self.feat)).reshape(1, -1)
            xz = np.array([r.predict(X)[0] for r in self.reg])
            self.last_speed_model = xz
            target = rf.local_speed_to_world(xz[None], grav[None], q[None])[0]
            if hold:
                target = np.zeros(3)
            self.cons.append((self.n - 1, target))
        if self.cons and self.n - self.last_opt >= self.opt_every:
            self.last_opt = self.n
            self.optimize()

    # ------------------------------------------------------------------ RIDI's correction step
    def optimize(self):
        s = max(0, self.n - self.window)
        e = self.n - 1
        self.cons = [(i, vt) for (i, vt) in self.cons if i > s]
        if len(self.cons) < 2 or e - s < 20:
            return
        nw = e - s + 1
        knots = list(range(s, e + 1, self.knot))
        if knots[-1] != e:
            knots.append(e)
        K = len(knots)
        Wm = np.zeros((nw, K))                           # piecewise-linear interpolation weights
        for m in range(K - 1):
            lo, hi = knots[m], knots[m + 1]
            j = np.arange(lo, hi + 1)
            alpha = (j - lo) / (hi - lo)
            Wm[j - s, m] += 1 - alpha
            Wm[j - s, m + 1] += alpha
        t = np.array(self.t[s:e + 1])
        a = np.array(self.a[s:e + 1])
        dts = np.diff(t)
        dts = np.where((dts > 0) & (dts < 0.05), dts, 0.005)
        # v[i] = v[s] + sum_{k=s}^{i-1} (a[k] - b[k]) dt_{k+1}    with b[k] = Wm[k] . B
        cum_w = np.vstack([np.zeros(K), np.cumsum(dts[:, None] * Wm[:-1], axis=0)])   # (nw, K)
        cum_a = np.vstack([np.zeros(3), np.cumsum(dts[:, None] * a[:-1], axis=0)])    # (nw, 3)
        v_s = self.v[s]
        rows = np.array([i - s for i, _ in self.cons])
        target = np.array([vt for _, vt in self.cons])
        A = cum_w[rows]                                   # v_int - A B = target  ->  A B = v_int - target
        rhs = v_s + cum_a[rows] - target
        B = np.linalg.solve(A.T @ A + self.ridge * np.eye(K), A.T @ rhs)
        bias = Wm @ B                                     # (nw, 3)
        for j in range(nw):
            self.bias[s + j] = bias[j]
        for j in range(s + 1, e + 1):                     # integrate again with the corrected acceleration
            dt = self.t[j] - self.t[j - 1]
            if not 0 < dt < 0.05:
                dt = 0.005
            self.v[j] = np.zeros(3) if self.held[j] else self.v[j - 1] + (self.a[j - 1] - self.bias[j - 1]) * dt
            self.p[j] = self.p[j - 1] + self.v[j - 1] * dt

    @property
    def position(self):
        return self.p[-1] if self.p else np.zeros(3)

    @property
    def velocity(self):
        return self.v[-1] if self.v else np.zeros(3)


class FrameMaker:
    """Raw 400 Hz A/G samples -> 200 Hz frames for the engine, using the board's own gyro tracker for
    orientation and gravity. Feed it lines; it calls `emit(t_s, gyro, acc, q, grav, still, tracker)`."""

    def __init__(self, emit):
        self.tr = t3.Tracker()
        self.emit = emit
        self.pending = None

    def line(self, text):
        f = text.split(",")
        try:
            if f[0] == "G" and len(f) == 5:
                self.tr.gyro(int(f[1]), np.array([float(x) for x in f[2:5]]))
            elif f[0] == "INIT" and len(f) == 20:
                self.tr.load_init([float(x) for x in f[1:]])
                self.pending = None
            elif f[0] == "A" and len(f) == 5:
                ts = int(f[1])
                a = np.array([float(x) for x in f[2:5]])
                self.tr.accel(ts, a)
                if self.tr.state == t3.SETTLING:
                    return
                grav = t3.rotate_inv(self.tr.q, np.array([0.0, 0.0, 1.0])) * self.tr.gLocal
                sample = (ts, self.tr.wLast.copy(), a, self.tr.q.copy(), grav)
                if self.pending is None:
                    self.pending = sample
                else:                                                 # average a pair: 400 Hz -> 200 Hz
                    p0 = self.pending
                    self.pending = None
                    q = p0[3] + sample[3]
                    q /= np.linalg.norm(q)
                    self.emit((p0[0] + sample[0]) / 2e6, (p0[1] + sample[1]) / 2, (p0[2] + sample[2]) / 2, q,
                              (p0[4] + sample[4]) / 2, self.tr.state == t3.STILL, self.tr)
        except ValueError:
            pass


def make_link_class(cls_index, pure):
    import track3d

    class RidiLink(track3d.Link):
        def __init__(self, port, raw, log_path):
            super().__init__(port, True, log_path)        # RIDI needs the raw samples
            self.engine = RidiEngine(cls_index, pure)
            self.frames = 0
            self.maker = FrameMaker(self._frame)

        def _frame(self, t, gyro, acc, q, grav, still, tr):
            self.engine.frame(t, gyro, acc, q, grav, still)
            self.frames += 1
            if self.frames % 4 == 0:                          # 50 Hz to the window
                self.last_data = __import__("time").time()
                self.events.put(("trk", {"ms": t * 1000.0, "q": q, "p": self.engine.position.copy(),
                                         "v": self.engine.velocity.copy(),
                                         "state": 1 if still else 2, "moves": tr.moves}))

        def on_line(self, text, now):
            if text[:2] in ("A,", "G,") or text.startswith("INIT,"):
                self.maker.line(text)
            elif not text.startswith(("TRK,", "STAT,")):
                self.events.put(("text", text))

    return RidiLink


def run_offline(log, cls_index, pure, out_trk):
    """The engine over a saved recording; writes TRK-format lines the viewer can replay."""
    eng = RidiEngine(cls_index, pure)
    lines = []
    count = [0]

    def emit(t, gyro, acc, q, grav, still, tr):
        eng.frame(t, gyro, acc, q, grav, still)
        count[0] += 1
        if count[0] % 4 == 0:
            p, v = eng.position, eng.velocity
            lines.append("TRK,%.0f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%d\n" % (
                t * 1000, q[0], q[1], q[2], q[3], p[0], p[1], p[2], v[0], v[1], v[2], 1 if still else 2, tr.moves))

    maker = FrameMaker(emit)
    for text in open(log, errors="replace"):
        maker.line(text.strip())
    Path(out_trk).write_text("".join(lines))
    return eng


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pure", action="store_true", help="RIDI alone: no rest detection (it then drifts while resting)")
    ap.add_argument("--style", type=int, default=0, choices=[0, 1, 2, 3],
                    help="which RIDI model: 0 handheld (default), 1 leg, 2 bag, 3 body")
    ap.add_argument("--offline", metavar="LOG", help="run on a saved recording instead of the live board")
    args, rest = ap.parse_known_args()
    import track3d

    pure = args.pure
    track3d.MOVE_WARNING = False  # long continuous movement is what RIDI is for, not a fault
    track3d.TITLE_PREFIX = "RIDI tracker (%s model%s)" % (rm.CLASS_NAMES[args.style], ", alone" if pure else " + rest detection")
    if args.offline:
        out = Path(args.offline).with_suffix(".ridi.trk")
        eng = run_offline(args.offline, args.style, pure, out)
        p = eng.position
        print(f"RIDI ended at x {p[0]:+.2f} y {p[1]:+.2f} z {p[2]:+.2f} m after {eng.n / 200:.0f} s")
        sys.argv = [sys.argv[0], "--replay", str(out)] + rest
    else:
        track3d.Link = make_link_class(args.style, pure)
        sys.argv = [sys.argv[0]] + rest
    track3d.main()


if __name__ == "__main__":
    main()
