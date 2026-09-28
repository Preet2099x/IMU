"""Live 3D orientation of the board in matplotlib, plus front/back and left/right movement.

    python visualizer/orient3d.py                     # the board over WiFi
    python visualizer/orient3d.py --port COM6         # over USB
    python visualizer/orient3d.py --replay FILE.viz   # a saved session

Same conventions as the mapping viewer: the board is drawn with its own axes,
X red, Y green (the nose), Z blue, and the numbers are the firmware's roll, pitch
and heading. By default the first reading counts as "flat, heading 0" (like the
mapping viewer's "Start = flat"); press A to see the angles exactly as the
board reports them.

Movement: the board also moves through the 3D space as you move it front/back (X)
and left/right (Y), along the directions of its own X and Y axes when zeroed. Only
those two are tracked, so up/down is ignored; gravity is simply taken off.
The acceleration is added up to speed and then position; when the board is held
still its speed is set to zero and, if drift correction is on, the speed error the
move built up is taken back off the position (the method from the reference
script). Expect about centimetres for short moves with a pause at each end, and
worse for long continuous movement: an accelerometer alone drifts.

Every live session is logged to recordings/orient_<time>.viz.

Keys: Z re-zero (the current pose becomes flat, heading 0, position 0),
      P zero the position, D drift correction on/off, K switch tracker
      (simple ZUPT / ZUPT Kalman filter),
      A absolute/relative angles, Q quit.
Needs matplotlib, numpy, pyserial. Only one program may use the board's WiFi
port, so close the other viewers first."""
import argparse
import collections
import queue
import sys
import time
from pathlib import Path

import matplotlib
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from bridge import CalStore, Hub, SerialLink, parse_viz  # noqa: E402

HERE = Path(__file__).parent
DEFAULT_PORT = "auto"

# --- quaternion helpers, [w, x, y, z]; q rotates the sensor frame into the earth frame ---


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qnorm(q):
    return q / (np.linalg.norm(q) or 1.0)


def euler_to_quat(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_euler(q):
    """Roll, pitch, yaw in degrees: the same extraction the firmware prints."""
    w, x, y, z = q
    sinp = np.clip(2 * (w * y - z * x), -1, 1)
    return (
        np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
        np.degrees(np.arcsin(sinp)),
        np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
    )


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def relative_to(cur, ref):
    """Orientation of `cur` measured from the pose `ref`, which counts as flat with
    heading 0 whichever way it was facing (the same maths as the mapping viewer)."""
    face_ref = euler_to_quat(0.0, 0.0, np.radians(quat_to_euler(ref)[2]))
    return qnorm(qmul(qmul(qmul(qconj(face_ref), cur), qconj(ref)), face_ref))


G = 9.80665


class Slide:
    """Horizontal position [x front, y left] in metres from the accelerometer."""

    def __init__(self, window=15, gyro_quiet=3.0, acc_tol=0.3, acc_move=0.6, settle=0.5):
        self.window, self.gyro_quiet, self.acc_tol, self.acc_move, self.settle = window, gyro_quiet, acc_tol, acc_move, settle
        self.g_vec = np.array([0.0, 0.0, G])
        self.correct = True
        self.reset()

    def reset(self):
        self.pos = np.zeros(2)
        self.vel = np.zeros(2)
        self.bias = np.zeros(3)  # what the accelerometer reads beyond gravity when still (sensor axes, m/s^2)
        self.win = collections.deque(maxlen=self.window)
        self.still_t = 0.0
        self.ready = False
        self.moving = False
        self.move_t = 0.0
        self.last_ms = None

    def update(self, ms, q, f, gyro, yaw0):
        """q sensor->earth; f accelerometer in g (sensor axes, calibrated); gyro deg/s;
        yaw0 radians: the heading that counts as forward."""
        dt = 0.0 if self.last_ms is None else (ms - self.last_ms) / 1000.0
        self.last_ms = ms
        if not 0 < dt < 0.2:
            dt = 0.0
        R = quat_to_mat(q)
        c, s_ = np.cos(-yaw0), np.sin(-yaw0)
        Rs = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]]) @ R  # sensor -> room (y = left of the zeroed heading)
        fb = np.asarray(f, float) * G
        self.win.append((fb, np.asarray(gyro, float)))
        still = False
        if len(self.win) == self.window:
            F = np.array([w[0] for w in self.win])
            Gy = np.array([w[1] for w in self.win])
            spread = np.sqrt(((F - F.mean(0)) ** 2).sum(1).mean())
            still = np.linalg.norm(Gy.mean(0)) < self.gyro_quiet and spread < self.acc_tol
        a = Rs @ (fb - self.bias) - self.g_vec
        if np.hypot(a[0], a[1]) > self.acc_move:
            still = False  # a clear push: don't wait for the window to notice
        self.still_t = self.still_t + dt if still else 0.0
        was_moving = self.moving
        if still:
            if was_moving and self.correct and self.move_t > 0:
                # a steady error grows the speed error linearly, so the position error
                # is half of (speed error x move time)
                self.pos = self.pos - 0.5 * self.vel * self.move_t
            self.vel = np.zeros(2)
            self.moving = False
            self.move_t = 0.0
            if self.still_t >= self.settle:
                self.bias = np.mean([w[0] for w in self.win], axis=0) - R.T @ self.g_vec
                self.ready = True
        elif self.ready and dt > 0:
            self.moving = True
            self.move_t += dt
            self.pos = self.pos + self.vel * dt + 0.5 * a[:2] * dt * dt
            self.vel = self.vel + a[:2] * dt
        return self.pos


class ZuptKF:
    """Zero-velocity-update error-state Kalman filter, the standard gait-tracking method.

    Runs the board's acceleration (gravity taken off with the firmware's orientation)
    to speed and position, and in parallel a Kalman filter over the ERRORS in position,
    speed, tilt and accelerometer bias. Whenever the board is detected still, "speed is
    zero" is fed in as a measurement: that removes the speed error, pulls the position
    back by the share of drift built up during the move, and learns the bias and tilt
    error for the next move. The still test looks at a window centred on the sample
    being processed, so everything runs about 0.15 s behind."""

    def __init__(self, window=15, sigma_acc=0.05, sigma_gyro=1.0, gamma=12.0, gyro_quiet=4.0,
                 sigma_v=0.02, q_vel=0.08, q_tilt=0.002, q_bias=0.004, settle=0.5, max_tilt=6.0, max_bias=0.6):
        self.window, self.sigma_acc, self.sigma_gyro, self.gamma, self.gyro_quiet = window, sigma_acc, sigma_gyro, gamma, gyro_quiet
        self.sigma_v, self.q_vel, self.q_tilt, self.q_bias, self.settle = sigma_v, q_vel, q_tilt, q_bias, settle
        self.max_tilt, self.max_bias = max_tilt, max_bias
        self.reset()

    def reset(self):
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.bias = np.zeros(3)  # accelerometer bias, sensor axes, m/s^2
        self.corr = np.array([1.0, 0, 0, 0])  # tilt correction on top of the firmware's orientation
        self.origin = np.zeros(3)
        self.P = np.zeros((12, 12))
        self.P[3:6, 3:6] = np.eye(3) * 0.5 ** 2
        self.P[6:9, 6:9] = np.eye(3) * 0.05 ** 2
        self.P[9:12, 9:12] = np.eye(3) * 0.15 ** 2
        self.win = collections.deque(maxlen=self.window)
        self.still_t = 0.0
        self.ready = False
        self.moving = False
        self.last_ms = None
        self.out = np.zeros(2)

    def zero(self):
        self.origin = self.p.copy()

    def update(self, ms, q, f, gyro, yaw0):
        dt = 0.0 if self.last_ms is None else (ms - self.last_ms) / 1000.0
        self.last_ms = ms
        if not 0 < dt < 0.2:
            dt = 0.0
        self.win.append({"f": np.asarray(f, float) * G, "g": np.asarray(gyro, float), "q": np.asarray(q, float), "dt": dt})
        still = False
        if len(self.win) == self.window:
            F = np.array([w["f"] for w in self.win])
            Gy = np.array([w["g"] for w in self.win])
            variation = (((F - F.mean(0)) ** 2).sum(1) / self.sigma_acc ** 2 + (Gy ** 2).sum(1) / self.sigma_gyro ** 2).mean()
            still = variation < self.gamma and np.linalg.norm(Gy.mean(0)) < self.gyro_quiet
            centre = self.win[self.window // 2]
            self.still_t = self.still_t + centre["dt"] if still else 0.0
            if not self.ready and self.still_t >= self.settle:
                self.ready = True
            if self.ready and centre["dt"] > 0:
                self._predict(qnorm(qmul(self.corr, centre["q"])), centre["f"], centre["dt"])
            if self.ready and still:
                self._zero_velocity()
        self.moving = self.ready and not still
        c, s_ = np.cos(-yaw0), np.sin(-yaw0)
        d = self.p - self.origin
        self.out = np.array([c * d[0] - s_ * d[1], s_ * d[0] + c * d[1]])
        return self.out

    def _predict(self, q, fm, dt):
        R = quat_to_mat(q)
        fe = R @ (fm - self.bias)
        a = fe - np.array([0.0, 0.0, G])
        self.p = self.p + self.v * dt + 0.5 * a * dt * dt
        self.v = self.v + a * dt
        skew = np.array([[0, -fe[2], fe[1]], [fe[2], 0, -fe[0]], [-fe[1], fe[0], 0]])
        Fm = np.eye(12)
        Fm[0:3, 3:6] = np.eye(3) * dt
        Fm[3:6, 6:9] = skew * dt
        Fm[3:6, 9:12] = R * dt
        Fm[0:3, 6:9] = 0.5 * skew * dt * dt
        Fm[0:3, 9:12] = 0.5 * R * dt * dt
        P = Fm @ self.P @ Fm.T
        P[3:6, 3:6] += np.eye(3) * self.q_vel ** 2 * dt
        P[6:9, 6:9] += np.eye(3) * self.q_tilt ** 2 * dt
        P[9:12, 9:12] += np.eye(3) * self.q_bias ** 2 * dt
        self.P = P

    def _zero_velocity(self):
        P = self.P
        S = P[3:6, 3:6] + np.eye(3) * self.sigma_v ** 2
        K = P[:, 3:6] @ np.linalg.inv(S)
        x = K @ self.v
        Pn = P - K @ P[3:6, :]
        self.P = 0.5 * (Pn + Pn.T)
        self.p = self.p - x[0:3]
        self.bias = np.clip(self.bias + x[9:12], -self.max_bias, self.max_bias)
        dq = qnorm(np.array([1.0, *(x[6:9] / 2)]))
        corr = qnorm(qmul(dq, self.corr))
        if corr[0] < 0:
            corr = -corr
        angle = 2 * np.degrees(np.arccos(min(1.0, corr[0])))
        if angle > self.max_tilt:
            axis = corr[1:] / (np.linalg.norm(corr[1:]) or 1.0)
            half = np.radians(self.max_tilt) / 2
            corr = np.array([np.cos(half), *(axis * np.sin(half))])
        self.corr = corr
        self.v = np.zeros(3)


class Tracks:
    """Both trackers run on every sample, so switching between them is instant."""

    def __init__(self):
        self.simple = Slide()
        self.kf = ZuptKF()
        self.use_kf = False

    def update(self, ms, q, f, gyro, yaw0):
        self.simple.update(ms, q, f, gyro, yaw0)
        self.kf.update(ms, q, f, gyro, yaw0)

    @property
    def active(self):
        return self.kf if self.use_kf else self.simple

    @property
    def pos(self):
        return self.kf.out if self.use_kf else self.simple.pos

    @property
    def ready(self):
        return self.active.ready

    @property
    def moving(self):
        return self.active.moving

    @property
    def correct(self):
        return self.simple.correct

    @correct.setter
    def correct(self, value):
        self.simple.correct = value

    def zero(self):
        self.simple.pos, self.simple.vel = np.zeros(2), np.zeros(2)
        self.kf.zero()


# --- the board model: 4.2 x 6.4 x 0.2, long along Y (the nose), like the mapping viewer ---
BOARD_SCALE = 0.1  # drawn about 14 cm long, so it is to scale with the metre axes
SIZE = np.array([4.2, 6.4, 0.2]) / 6.4 * 1.4  # about 1.4 long
CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * SIZE / 2
FACES = [  # corner indices, and colour: sides dark green, top green, underside slate
    ([4, 5, 7, 6], "#0b5c3c"), ([0, 1, 3, 2], "#0b5c3c"),  # +x, -x
    ([2, 3, 7, 6], "#0b5c3c"), ([0, 1, 5, 4], "#0b5c3c"),  # +y, -y
    ([1, 3, 7, 5], "#14a06b"), ([0, 2, 6, 4], "#2b3a52"),  # +z (top), -z (under)
]
AXIS_COLORS = ("#ff5c5c", "#5cd67c", "#5c9bff")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=DEFAULT_PORT, help="COM port, socket://host:port, or 'auto' (default): find the board on USB or any WiFi")
    ap.add_argument("--replay", metavar="FILE", help="play back a saved session instead of a live board")
    ap.add_argument("--skip-cal", action="store_true", help="skip the board's 30 s magnetometer calibration")
    ap.add_argument("--save", metavar="PNG", help="with --replay: draw the last pose, save it and exit")
    args = ap.parse_args()
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    state = {"q": None, "ref": None, "absolute": False, "status": {"state": "waiting", "message": "connecting..."}}
    cal = CalStore(HERE / "accel_cal.json").get()
    off, scale = np.array(cal["offset"]), np.array(cal["scale"])
    slide = Tracks()
    link = sub = None
    replay_events = []
    if args.replay:
        replay_events = [e for e in (parse_viz(line) for line in Path(args.replay).read_text().splitlines()) if e]
        state["status"] = {"state": "replay", "message": f"replaying {Path(args.replay).name}"}
    else:
        hub = Hub()
        port = None if args.port == "auto" else args.port  # None: the link finds the board itself, every time it connects
        log_path = HERE / "recordings" / time.strftime("orient_%Y%m%d_%H%M%S.viz")
        log_path.parent.mkdir(exist_ok=True)
        print("logging raw board data to", log_path)
        link = SerialLink(hub, CalStore(HERE / "accel_cal.json"), port, skip_cal=args.skip_cal,
                          record=open(log_path, "w", encoding="utf-8", buffering=1))
        link.start()
        sub = hub.subscribe()

    trail = [np.zeros(2)]  # [x, y] position over time
    marks = []  # (position, board-to-room matrix) every 0.4 s, for the arrows

    def shown(q):
        return q if state["absolute"] else relative_to(q, state["ref"])

    def feed(ev):
        state["q"] = qnorm(np.array(ev["q"], float))
        if state["ref"] is None:
            state["ref"] = state["q"].copy()
        f = (np.array(ev["f"]) - off) * scale
        slide.update(ev["ms"], state["q"], f, ev["g"], np.radians(quat_to_euler(state["ref"])[2]))
        trail.append(slide.pos.copy())
        if len(trail) % 20 == 0:
            marks.append((slide.pos.copy(), quat_to_mat(shown(state["q"]))))

    def pump():
        if sub is not None:
            while True:
                try:
                    ev = sub.get_nowait()
                except queue.Empty:
                    break
                if ev["type"] == "status":
                    state["status"] = {"state": ev["state"], "message": ev["message"]}
                elif ev["type"] == "viz":
                    feed(ev)
        for ev in replay_events[:40]:
            feed(ev)
        del replay_events[:40]

    fig = plt.figure(figsize=(8, 8))
    fig.patch.set_facecolor("#e8e8e8")
    ax = fig.add_subplot(111, projection="3d")
    fig.patch.set_facecolor("#c9c9c9")
    ax.set_xlabel("X (m)  front +"); ax.set_ylabel("Y (m)  left +"); ax.set_zlabel("Z (m)")
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis._axinfo["grid"].update(linestyle=":", color="0.55", linewidth=0.8)
    ax.view_init(elev=24, azim=-35)
    (trail_line,) = ax.plot([], [], [], color="k", lw=1.6)
    hist_arrows = [ax.plot([], [], [], color=c, lw=1.0)[0] for c in AXIS_COLORS]
    board = Poly3DCollection([], edgecolor="k", linewidths=0.5)
    ax.add_collection3d(board)
    axes_lines = [ax.plot([], [], [], color=c, lw=3)[0] for c in AXIS_COLORS]
    (nose,) = ax.plot([], [], [], color="#ffb020", lw=5, solid_capstyle="round")
    title = ax.set_title("")
    fig.subplots_adjust(bottom=0.16)
    numbers = fig.text(0.5, 0.02, "", ha="center", fontsize=15, family="monospace")

    def draw(_=None):
        pump()
        q = state["q"]
        if q is None:
            q_show = np.array([1.0, 0, 0, 0])
        else:
            q_show = q if state["absolute"] else relative_to(q, state["ref"])
        R = quat_to_mat(q_show)
        pos = slide.pos
        pts = CORNERS @ R.T * BOARD_SCALE + np.array([pos[0], pos[1], 0.0])
        board.set_verts([pts[idx] for idx, _ in FACES])
        board.set_facecolor([c for _, c in FACES])
        T = np.array(trail)
        trail_line.set_data_3d(T[:, 0], T[:, 1], np.zeros(len(T)))
        for k in range(3):
            xs, ys, zs = [], [], []
            for mp, Rm in marks[-300:]:
                d = Rm[:, k] * 0.07
                xs += [mp[0], mp[0] + d[0], np.nan]
                ys += [mp[1], mp[1] + d[1], np.nan]
                zs += [0.0, d[2], np.nan]
            hist_arrows[k].set_data_3d(np.array(xs, float), np.array(ys, float), np.array(zs, float))
            tip = R[:, k] * 0.14
            axes_lines[k].set_data_3d(np.array([pos[0], pos[0] + tip[0]]), np.array([pos[1], pos[1] + tip[1]]), np.array([0, tip[2]]))
        n0 = R @ np.array([0, SIZE[1] / 2 * BOARD_SCALE, 0])
        n1 = R @ np.array([0, SIZE[1] / 2 * BOARD_SCALE + 0.04, 0])
        nose.set_data_3d(np.array([pos[0] + n0[0], pos[0] + n1[0]]), np.array([pos[1] + n0[1], pos[1] + n1[1]]), np.array([n0[2], n1[2]]))
        # equal-scale metre axes that grow with the path
        lo, hi = np.minimum(T.min(0), 0.0), np.maximum(T.max(0), 0.0)
        half = max(0.5, float((hi - lo).max()) / 2 * 1.15 + 0.1)
        mid = (hi + lo) / 2
        ax.set_xlim(mid[0] - half, mid[0] + half)
        ax.set_ylim(mid[1] - half, mid[1] + half)
        ax.set_zlim(-half, half)
        ax.set_box_aspect((1, 1, 1))
        st = state["status"]
        if st["state"] == "connected":
            conn, color = "BOARD CONNECTED", "tab:green"
        elif st["state"] == "replay":
            conn, color = st["message"].upper(), "tab:green"
        else:
            conn, color = "BOARD NOT CONNECTED: " + st["message"], "tab:red"
        if q is None and st["state"] == "connected":
            conn += " (waiting for data; the board starts up for ~35 s)"
        mode = "settling: hold still" if not slide.ready else ("moving" if slide.moving else "still")
        title.set_text(conn + f"  |  {mode}" + chr(10) + ("absolute angles" if state["absolute"] else "start = flat, heading 0")
                       + ("  |  tracker: ZUPT Kalman filter" if slide.use_kf
                          else f"  |  tracker: simple ZUPT, drift correction {'on' if slide.correct else 'off'}"))
        title.set_color(color)
        roll, pitch, yaw = quat_to_euler(q_show)
        fb = "front" if slide.pos[0] >= 0 else "back"
        lr = "left" if slide.pos[1] >= 0 else "right"
        numbers.set_text(f"roll {roll:+7.1f}°   pitch {pitch:+7.1f}°   heading {yaw % 360:6.1f}°" + chr(10)
                         + f"{fb} {abs(slide.pos[0]) * 100:5.1f} cm   {lr} {abs(slide.pos[1]) * 100:5.1f} cm from the start")
        return []

    def on_key(e):
        if e.key == "z" and state["q"] is not None:
            state["ref"] = state["q"].copy()
            state["absolute"] = False
            slide.zero()
            trail[:] = [np.zeros(2)]; marks.clear()
        elif e.key == "p":
            slide.zero()
            trail[:] = [np.zeros(2)]; marks.clear()
        elif e.key == "k":
            slide.use_kf = not slide.use_kf
        elif e.key == "d":
            slide.correct = not slide.correct
        elif e.key == "a":
            state["absolute"] = not state["absolute"]
        elif e.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    if args.save:
        while replay_events:
            pump()
        draw()
        fig.savefig(args.save, dpi=110)
        print("saved", args.save)
        return
    from matplotlib import animation
    anim = animation.FuncAnimation(fig, draw, interval=30, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        if link:
            link.stop()
    del anim


if __name__ == "__main__":
    main()
