"""A line-by-line Python copy of src/tracker3d.h, for replaying recordings offline.

The board sends every sample it tracked (A/G lines, in the order it used them)
when the viewer asks for the raw stream, so a recording can be run through this
copy with different settings and compared with what the board itself drew.

    python visualizer/tracker3d.py visualizer/recordings/track_....log

prints where each move ended, from the board and from this copy.
"""
import math
import sys
from dataclasses import dataclass

import numpy as np

DEG = math.pi / 180


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def qnorm(q):
    n = np.linalg.norm(q)
    return np.array([1.0, 0, 0, 0]) if n < 1e-12 else q / n


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qexp(r):
    a = np.linalg.norm(r)
    if a < 1e-9:
        return qnorm(np.array([1.0, 0.5 * r[0], 0.5 * r[1], 0.5 * r[2]]))
    s = math.sin(0.5 * a) / a
    return np.array([math.cos(0.5 * a), r[0] * s, r[1] * s, r[2] * s])


def rotate(q, v):
    u = q[1:]
    t = np.cross(u, v) * 2.0
    return v + t * q[0] + np.cross(u, t)


def rotate_inv(q, v):
    return rotate(qconj(q), v)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def yaw_quat(yaw):
    return np.array([math.cos(0.5 * yaw), 0, 0, math.sin(0.5 * yaw)])


def i32(d):
    """Wrap-safe difference of two uint32 microsecond stamps."""
    d &= 0xFFFFFFFF
    return d - (1 << 32) if d >= (1 << 31) else d


@dataclass
class Params:
    accStill: float = 0.35
    gyroStill: float = 6.0 * DEG
    gyroStillMax: float = 15.0 * DEG
    accShift: float = 0.40
    holdSlow: float = 0.08
    holdFast: float = 0.40
    vFast: float = 0.25
    tauTilt: float = 0.10
    tauG: float = 0.30
    tauRef: float = 0.50
    tauBias: float = 2.0
    biasAcc: float = 0.05
    biasGyro: float = 1.0 * DEG
    biasAfter: float = 0.5
    settleTime: float = 1.0
    backfill: float = 0.15
    maxCorrect: float = 4.0
    maxMove: float = 3.0


SETTLING, STILL, MOVING = 0, 1, 2
WIN, RECENT, HIST = 40, 8, 96


class Tracker:
    def __init__(self, params=None):
        self.P = params or Params()
        self.q = np.array([1.0, 0, 0, 0])
        self.bg = np.zeros(3)
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.gLocal = 9.80665
        self.fRef = np.array([0, 0, 9.80665])
        self.state = SETTLING
        self.lost = False
        self.moves = 0
        self.lastMoveTime = 0.0
        self.lastMoveVres = np.zeros(3)
        self.lastMoveCorr = np.zeros(3)
        self.accStd = self.gyroMean = self.gyroMax = self.shift = 0.0
        self.aWin = np.zeros((WIN, 3))
        self.gRaw = np.zeros((WIN, 3))
        self.gMag = np.zeros(WIN)
        self.hist = [None] * HIST
        self.ai = self.aN = self.gi = self.gN = self.hi = self.hN = 0
        self.tA = self.tG = 0
        self.haveA = self.haveG = False
        self.wLast = np.zeros(3)
        self.quietT = self.stillT = self.moveT = 0.0

    def load_init(self, f):
        """INIT line fields (after 'INIT'): the board's state when the recording began."""
        self.q = np.array(f[0:4])
        self.bg = np.array(f[4:7])
        self.p = np.array(f[7:10])
        self.gLocal = f[10]
        self.state = int(f[11])
        self.fRef = np.array(f[12:15])
        self.v = np.zeros(3)

    def gyro(self, t, wRaw):
        dt = i32(t - self.tG) * 1e-6 if self.haveG else 0.0
        self.tG, self.haveG = t, True
        if dt < 0 or dt > 0.05:
            dt = 0.0
        w = wRaw - self.bg
        if self.state != MOVING:
            up = rotate_inv(self.q, np.array([0, 0, 1.0]))
            w = w - up * np.dot(w, up)
        self.gRaw[self.gi] = wRaw
        self.gMag[self.gi] = np.linalg.norm(w)
        self.gi = (self.gi + 1) % WIN
        self.gN = min(self.gN + 1, WIN)
        self.q = qnorm(qmul(self.q, qexp(w * dt)))
        self.wLast = w

    def accel(self, t, f):
        P = self.P
        dt = i32(t - self.tA) * 1e-6 if self.haveA else 0.0
        self.tA, self.haveA = t, True
        if dt < 0 or dt > 0.05:
            dt = 0.0
        self.aWin[self.ai] = f
        self.ai = (self.ai + 1) % WIN
        self.aN = min(self.aN + 1, WIN)
        qa = self.q
        if self.haveG:
            d = max(-0.005, min(0.005, i32(t - self.tG) * 1e-6))
            qa = qnorm(qmul(self.q, qexp(self.wLast * d)))
        an = rotate(qa, f) - np.array([0, 0, self.gLocal])
        self.hist[self.hi] = (t, an, dt)
        self.hi = (self.hi + 1) % HIST
        self.hN = min(self.hN + 1, HIST)

        A = self.aWin[:self.aN]
        fMean = A.mean(0)
        self.accStd = math.sqrt(((A - fMean) ** 2).sum(1).mean())
        if self.gN:
            self.gyroMean = self.gMag[:self.gN].mean()
            self.gyroMax = self.gMag[:self.gN].max()
            wRawMean = self.gRaw[:self.gN].mean(0)
        else:
            wRawMean = np.zeros(3)
        nr = min(self.aN, RECENT)
        recent = np.mean([self.aWin[(self.ai - k) % WIN] for k in range(1, nr + 1)], axis=0)
        self.shift = np.linalg.norm(recent - self.fRef)
        full = self.aN >= WIN and self.gN >= WIN
        quiet = full and self.accStd < P.accStill and self.gyroMean < P.gyroStill and self.gyroMax < P.gyroStillMax

        if self.state == SETTLING:
            self.quietT = self.quietT + dt if quiet else 0.0
            if self.quietT >= P.settleTime:
                self.bg = wRawMean.copy()
                self.q = np.array([1.0, 0, 0, 0])
                self._align_tilt(fMean, 1.0)
                self.q = qnorm(qmul(yaw_quat(-yaw_of(self.q)), self.q))
                self.gLocal = np.linalg.norm(fMean)
                self.fRef = fMean.copy()
                self.v = np.zeros(3)
                self.state = STILL
                self.stillT = 0.0
        elif self.state == STILL:
            if not full:
                pass  # windows still filling (only after a restart of the tracker): wait
            elif not quiet or self.shift > P.accShift:
                self._start_move(t)
            else:
                self.stillT += dt
                self.v = np.zeros(3)
                self._align_tilt(fMean, min(1.0, dt / P.tauTilt))
                self.gLocal += min(1.0, dt / P.tauG) * (np.linalg.norm(fMean) - self.gLocal)
                self.fRef = self.fRef + (fMean - self.fRef) * min(1.0, dt / P.tauRef)
                if self.stillT > P.biasAfter and self.accStd < P.biasAcc and self.gyroMean < P.biasGyro:
                    self.bg = self.bg + (wRawMean - self.bg) * min(1.0, dt / P.tauBias)
        else:
            if self.moveT > P.maxMove:
                self.lost = True
                self.v = np.zeros(3)
            else:
                self._integrate(an, dt)
            self.moveT += dt
            self.quietT = self.quietT + dt if quiet else 0.0
            hold = P.holdSlow if np.linalg.norm(self.v) < P.vFast else P.holdFast
            if self.quietT >= hold:
                self._end_move(fMean)

    def _align_tilt(self, fMean, k):
        n = np.linalg.norm(fMean)
        if n < 1e-3:
            return
        meas = fMean / n
        pred = rotate_inv(self.q, np.array([0, 0, 1.0]))
        e = np.cross(meas, pred)
        s = np.linalg.norm(e)
        if s < 1e-9:
            return
        angle = math.atan2(s, np.dot(meas, pred))
        self.q = qnorm(qmul(self.q, qexp(e * (angle * k / s))))

    def _integrate(self, a, dt):
        v0 = self.v
        self.v = self.v + a * dt
        self.p = self.p + (v0 + self.v) * (0.5 * dt)

    def _start_move(self, t):
        self.state = MOVING
        self.lost = False
        self.v = np.zeros(3)
        self.moveT = 0.0
        self.quietT = 0.0
        frm = (t - int(self.P.backfill * 1e6)) & 0xFFFFFFFF
        for k in range(self.hN, 0, -1):
            ht, a, dt = self.hist[(self.hi - k) % HIST]
            if i32(ht - frm) < 0:
                continue
            self._integrate(a, dt)
            self.moveT += dt

    def _end_move(self, fMean):
        self.lastMoveVres = self.v.copy()
        self.lastMoveTime = self.moveT
        self.lastMoveCorr = (self.v * (-0.5 * self.moveT) if not self.lost and self.moveT <= self.P.maxCorrect
                             else np.zeros(3))
        self.lost = False
        self.p = self.p + self.lastMoveCorr
        self.v = np.zeros(3)
        self.moves += 1
        self.state = STILL
        self.stillT = self.quietT
        self.fRef = fMean.copy()


def replay(path, params=None):
    """Runs the raw samples of a recording through the tracker. Returns a list of
    (t_us, position in the board's display frame, state) after every accel sample,
    and the board's own TRK rows [(ms, p, state, moves)]."""
    tr = Tracker(params)
    yaw0, origin = 0.0, np.zeros(3)
    out, board = [], []
    for line in open(path, encoding="utf-8", errors="replace"):
        f = line.strip().split(",")
        try:
            if f[0] == "A" and len(f) == 5:
                t = int(f[1])
                tr.accel(t, np.array([float(x) for x in f[2:5]]))
                rz = yaw_quat(-yaw0)
                out.append((t, rotate(rz, tr.p - origin), tr.state))
            elif f[0] == "G" and len(f) == 5:
                tr.gyro(int(f[1]), np.array([float(x) for x in f[2:5]]))
            elif f[0] == "INIT" and len(f) == 20:
                v = [float(x) for x in f[1:]]
                tr.load_init(v)
                yaw0, origin = v[15], np.array(v[16:19])
            elif f[0] == "TRK" and len(f) == 14:
                board.append((float(f[1]), np.array([float(x) for x in f[6:9]]), int(f[12]), int(f[13])))
        except ValueError:
            continue
    return out, board


def stops(rows, state_index=2, pos_index=1):
    """Positions where the board had just come to rest after a move."""
    res, prev = [], None
    for r in rows:
        st = r[state_index]
        if prev == MOVING and st == STILL:
            res.append(r[pos_index])
        prev = st
    return res


if __name__ == "__main__":
    mine, board = replay(sys.argv[1])
    a, b = stops(mine), stops(board)
    print(f"{len(mine)} accel samples replayed; {len(a)} stops here, {len(b)} stops from the board")
    for k in range(max(len(a), len(b))):
        pa = "  ".join(f"{x:+.3f}" for x in a[k]) if k < len(a) else "-"
        pb = "  ".join(f"{x:+.3f}" for x in b[k]) if k < len(b) else "-"
        print(f"stop {k + 1:2d}   replay {pa:30s}   board {pb}")
