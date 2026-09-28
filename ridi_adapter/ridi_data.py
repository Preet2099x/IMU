"""Loads a recording made by visualizer/track3d.py (the ESP32's raw 400 Hz accelerometer and
gyro samples) and turns it into RIDI-style streams at 200 Hz: gyro, linear acceleration,
gravity and orientation, all in the device's own axes, like an Android phone log.

The orientation comes from the same gyro-only tracker the board uses (visualizer/tracker3d.py),
which is also what it would use live.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "visualizer"))
import tracker3d as t3  # noqa: E402


def load_track_log(path, max_seconds=None):
    """Returns a dict of 200 Hz arrays:
      t (s), gyro (rad/s), acc (m/s^2, what the accelerometer reads), lin (acc minus gravity),
      grav (gravity as the device feels it, m/s^2), q (device -> room, [w,x,y,z]),
      v_ref / state (our own tracker's speed and still/moving state, for comparison)."""
    tr = t3.Tracker()
    t, gyro, acc, grav, quat, vref, state = [], [], [], [], [], [], []
    t0 = None
    for line in open(path, errors="replace"):
        f = line.strip().split(",")
        if f[0] == "G" and len(f) == 5:
            try:
                tr.gyro(int(f[1]), np.array([float(x) for x in f[2:5]]))
            except ValueError:
                pass
        elif f[0] == "INIT" and len(f) == 20:
            tr.load_init([float(x) for x in f[1:]])
        elif f[0] == "A" and len(f) == 5:
            try:
                ts = int(f[1])
                a = np.array([float(x) for x in f[2:5]])
            except ValueError:
                continue
            tr.accel(ts, a)
            if tr.state == t3.SETTLING:
                continue  # the tracker has no orientation or gravity size yet
            t0 = t0 if t0 is not None else ts
            if max_seconds and (ts - t0) / 1e6 > max_seconds:
                break
            t.append((ts - t0) / 1e6)
            gyro.append(tr.wLast.copy())  # turn rate with the learned gyro bias taken off
            acc.append(a)
            up_dev = t3.rotate_inv(tr.q, np.array([0.0, 0.0, 1.0]))
            grav.append(up_dev * tr.gLocal)
            quat.append(tr.q.copy())
            vref.append(tr.v.copy() if tr.state == t3.MOVING else np.zeros(3))
            state.append(tr.state)
    if not t:
        raise ValueError(f"{path}: no tracked samples (is it a track3d.py recording with raw A/G lines?)")
    t, gyro, acc, grav, quat, vref, state = (np.array(x) for x in (t, gyro, acc, grav, quat, vref, state))
    lin = acc - grav

    # 400 Hz -> 200 Hz by averaging neighbouring pairs (a simple anti-alias filter)
    n = len(t) // 2 * 2

    def pair(x):
        return x[:n].reshape(n // 2, 2, -1).mean(1)

    q2 = pair(quat)
    q2 /= np.linalg.norm(q2, axis=1, keepdims=True)
    return dict(t=t[:n:2] + 0.0025, gyro=pair(gyro), acc=pair(acc), lin=pair(lin), grav=pair(grav), q=q2,
                v_ref=pair(vref), state=state[:n:2])
