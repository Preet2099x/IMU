"""Live 3D path of the board, tracked on the board itself (src/espmain.cpp).

    python visualizer/track3d.py                        # finds the ESP32 on WiFi (or USB) by itself
    python visualizer/track3d.py --port COM8            # over USB (no raw samples)
    python visualizer/track3d.py --replay FILE.log      # look at a saved session again
    python visualizer/track3d.py --retrack FILE.log     # run a saved session through visualizer/tracker3d.py

The black line is where the board has been (metres: x front, y left, z up, from
where it first settled). The little red/green/blue arrows show which way its X, Y
and Z axes pointed along the way; the board is drawn at its current place.

How to use it: put the board down and keep it still for a second ("settling"),
then move it and stop. Each stop is where the board re-learns which way is down,
so short moves with a pause at the end track best.

Every live session is saved to visualizer/recordings/track_<time>.log, including
every raw sample over WiFi, so any run can be replayed and examined afterwards.

Keys: Z position back to zero (and clear the path), Y current heading becomes
      forward, C clear the path, Q quit.
Needs matplotlib, numpy and pyserial. Only one program can use the board's WiFi
connection at a time, so close the other viewers first.
"""
import argparse
import queue
import sys
import threading
import time
from pathlib import Path

import matplotlib
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))  # boardfind.py lives in the project root
import boardfind  # noqa: E402

DEFAULT_PORT = "auto"
MOVE_WARNING = True  # the stop-and-go tracker warns after 2 s of moving; RIDI (continuous movement) turns this off
TITLE_PREFIX = ""  # set by ridi_adapter/ridi_live.py so the window says which tracker it shows
STATES = {0: "settling: hold it still", 1: "still", 2: "moving",
          3: "LOST: moved over 3 s without a stop, position paused; hold it still"}


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler(q):
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw % 360


def parse_trk(line):
    f = line.strip().split(",")
    if len(f) != 14 or f[0] != "TRK":
        return None
    try:
        v = [float(x) for x in f[1:]]
    except ValueError:
        return None
    return {"ms": v[0], "q": np.array(v[1:5]), "p": np.array(v[5:8]), "v": np.array(v[8:11]),
            "state": int(v[11]), "moves": int(v[12])}


def parse_stat(line):
    f = line.strip().split(",")
    if len(f) != 14 or f[0] != "STAT":
        return None
    try:
        v = [float(x) for x in f[1:]]
    except ValueError:
        return None
    keys = ["accHz", "gyrHz", "accGlitches", "gyrOverruns", "rawDrops", "accStd", "gyroMean",
            "bgx", "bgy", "bgz", "gLocal", "lastMoveTime", "lastMoveVres"]
    return dict(zip(keys, v))


class Link(threading.Thread):
    """Keeps a connection to the board, asks for the tracking stream, saves every
    line it receives, and passes TRK/STAT lines on. Reconnects by itself."""

    def __init__(self, port, raw, log_path):
        super().__init__(daemon=True)
        self.port, self.raw, self.log_path = port, raw, log_path
        self.auto = port == "auto"  # look the board up every time we connect: its address changes
        self.events = queue.Queue()
        self.status = "connecting to " + port
        self.connected = False
        self.last_data = 0.0
        self.outbox = queue.Queue()
        self.halt = threading.Event()

    def send(self, data: bytes):
        self.outbox.put(data)

    def on_line(self, text, now):
        """One line from the board. The default shows the board's own tracking (TRK/STAT lines);
        ridi_adapter/ridi_live.py replaces this to track from the raw samples instead."""
        if text.startswith("TRK,"):
            ev = parse_trk(text)
            if ev:
                self.last_data = now
                self.events.put(("trk", ev))
        elif text.startswith("STAT,"):
            st = parse_stat(text)
            if st:
                self.events.put(("stat", st))
        elif text[0] not in "AG" or text[1:2] != ",":
            self.events.put(("text", text))

    def run(self):
        import serial
        log = open(self.log_path, "w", encoding="utf-8", buffering=1)
        while not self.halt.is_set():
            if self.auto:
                self.status = "looking for the board (USB, or WiFi on this network)..."
                found = boardfind.find_port(prefer_wifi=True)
                if not found:
                    self.connected = False
                    self.status = boardfind.not_found_message()
                    self.halt.wait(2.0)
                    continue
                self.port = found
            try:
                ser = boardfind.open_port(self.port, 115200, timeout=0.05)
            except (serial.SerialException, OSError, ValueError) as e:
                self.connected = False
                self.status = f"cannot reach {self.port} ({str(e)[:60]})"
                if self.auto:
                    self.port = "auto"
                self.halt.wait(2.0)
                continue
            self.connected, self.status = True, "connected to " + self.port
            buf, last_keep = b"", 0.0
            try:
                while not self.halt.is_set():
                    now = time.time()
                    if now - last_keep >= 1.0:
                        ser.write(b"r" if self.raw else b"t")
                        last_keep = now
                    while not self.outbox.empty():
                        ser.write(self.outbox.get_nowait())
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("utf-8", "replace").strip()
                        if not text:
                            continue
                        log.write(text + "\n")
                        self.on_line(text, now)
                    if len(buf) > 65536:
                        buf = b""
            except (serial.SerialException, OSError) as e:
                self.status = f"lost {self.port} ({str(e)[:60]})"
                if self.auto:
                    self.port = "auto"
            finally:
                self.connected = False
                try:
                    ser.write(b"h")
                    ser.close()
                except Exception:
                    pass
            self.halt.wait(1.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=DEFAULT_PORT, help="socket://host:port or a COM port; default 'auto' finds the board")
    ap.add_argument("--replay", metavar="LOG", help="show a saved session (the board's own tracking)")
    ap.add_argument("--retrack", metavar="LOG", help="run a saved session's raw samples through tracker3d.py")
    ap.add_argument("--save", metavar="PNG", help="with --replay/--retrack: save the final picture and exit")
    ap.add_argument("--no-raw", action="store_true", help="don't ask for raw samples (smaller logs)")
    args = ap.parse_args()
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    link = None
    pending = []  # events for replay modes
    if args.replay:
        for line in open(args.replay, encoding="utf-8", errors="replace"):
            ev = parse_trk(line)
            if ev:
                pending.append(("trk", ev))
        source = "replaying " + Path(args.replay).name
    elif args.retrack:
        sys.path.insert(0, str(HERE))
        import tracker3d as t3
        rows, _ = t3.replay(args.retrack)
        for k, (t, p, stt) in enumerate(rows):
            if k % 8 == 0:  # 400 Hz -> 50 Hz
                pending.append(("trk", {"ms": t / 1000, "q": np.array([1.0, 0, 0, 0]), "p": p, "v": np.zeros(3),
                                        "state": stt, "moves": 0}))
        source = "re-tracked " + Path(args.retrack).name
    else:
        log_path = HERE / "recordings" / time.strftime("track_%Y%m%d_%H%M%S.log")
        log_path.parent.mkdir(exist_ok=True)
        print("saving the session to", log_path)
        # raw samples (for replay) only exist over WiFi; asking for them elsewhere is harmless
        link = Link(args.port, not args.no_raw, log_path)
        link.start()
        source = None

    path = [np.zeros(3)]
    marks = []  # (position, orientation matrix) every 0.4 s
    cur = {"q": np.array([1.0, 0, 0, 0]), "p": np.zeros(3), "state": 0, "moves": 0, "n": 0, "ms": 0.0, "since": 0.0}
    stat = {}
    notes = []

    fig = plt.figure(figsize=(9, 8.5))
    fig.patch.set_facecolor("#c9c9c9")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)  front +")
    ax.set_ylabel("Y (m)  left +")
    ax.set_zlabel("Z (m)  up +")
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis._axinfo["grid"].update(linestyle=":", color="0.55", linewidth=0.8)
    ax.view_init(elev=24, azim=-35)
    (trail_line,) = ax.plot([], [], [], color="k", lw=1.6)
    hist_arrows = [ax.plot([], [], [], color=c, lw=1.0)[0] for c in ("#e03c3c", "#2eaa4f", "#3478e0")]
    board = Poly3DCollection([], edgecolor="k", linewidths=0.5)
    ax.add_collection3d(board)
    now_arrows = [ax.plot([], [], [], color=c, lw=3)[0] for c in ("#e03c3c", "#2eaa4f", "#3478e0")]
    title = ax.set_title("")
    fig.subplots_adjust(bottom=0.14)
    numbers = fig.text(0.5, 0.02, "", ha="center", fontsize=12, family="monospace")

    size = np.array([0.042, 0.064, 0.004])  # drawn at about the real board's size (m)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * size / 2
    faces = [([4, 5, 7, 6], "#0b5c3c"), ([0, 1, 3, 2], "#0b5c3c"), ([2, 3, 7, 6], "#0b5c3c"),
             ([0, 1, 5, 4], "#0b5c3c"), ([1, 3, 7, 5], "#14a06b"), ([0, 2, 6, 4], "#2b3a52")]

    def take(ev):
        kind, d = ev
        if kind == "trk":
            if d["state"] != cur["state"]:
                cur["since"] = d["ms"]
            cur.update(q=d["q"], p=d["p"], state=d["state"], moves=d["moves"], ms=d["ms"])
            cur["n"] += 1
            if np.linalg.norm(d["p"] - path[-1]) > 1e-4 or len(path) == 1:
                path.append(d["p"].copy())
            if cur["n"] % 20 == 0:
                marks.append((d["p"].copy(), quat_to_mat(d["q"])))
        elif kind == "stat":
            stat.update(d)
        elif kind == "text":
            notes.append(d)
            del notes[:-3]

    def pump():
        if link is not None:
            while not link.events.empty():
                take(link.events.get_nowait())
        else:
            for ev in pending[:25]:
                take(ev)
            del pending[:25]

    def draw(_=None):
        pump()
        P = np.array(path)
        R = quat_to_mat(cur["q"])
        pos = cur["p"]
        pts = corners @ R.T + pos
        board.set_verts([pts[idx] for idx, _ in faces])
        board.set_facecolor([c for _, c in faces])
        trail_line.set_data_3d(P[:, 0], P[:, 1], P[:, 2])
        # follow the recent path (last 600 points, about 12 s of moving) so one bad
        # excursion cannot shrink every later move to nothing
        recent = P[-600:]
        lo, hi = np.minimum(recent.min(0), pos - 0.1), np.maximum(recent.max(0), pos + 0.1)
        half = max(0.25, float((hi - lo).max()) / 2 * 1.15)
        mid = (hi + lo) / 2
        ax.set_xlim(mid[0] - half, mid[0] + half)
        ax.set_ylim(mid[1] - half, mid[1] + half)
        ax.set_zlim(mid[2] - half, mid[2] + half)
        ax.set_box_aspect((1, 1, 1))
        arrow = half * 0.08
        for k in range(3):
            xs, ys, zs = [], [], []
            for mp, Rm in marks[-400:]:
                d = Rm[:, k] * arrow
                xs += [mp[0], mp[0] + d[0], np.nan]
                ys += [mp[1], mp[1] + d[1], np.nan]
                zs += [mp[2], mp[2] + d[2], np.nan]
            hist_arrows[k].set_data_3d(np.array(xs, float), np.array(ys, float), np.array(zs, float))
            d = R[:, k] * arrow * 1.8
            now_arrows[k].set_data_3d(np.array([pos[0], pos[0] + d[0]]), np.array([pos[1], pos[1] + d[1]]),
                                      np.array([pos[2], pos[2] + d[2]]))
        if link is not None:
            fresh = time.time() - link.last_data < 1.5
            if link.connected and fresh:
                head, color = "BOARD CONNECTED", "tab:green"
            elif link.connected:
                head, color = "CONNECTED, NO DATA YET (board starting?)", "tab:orange"
            else:
                head, color = "BOARD NOT CONNECTED: " + link.status, "tab:red"
        else:
            head, color = source.upper() + ("" if pending else "  (end)"), "tab:green"
        line2 = f"{STATES.get(cur['state'], '?')}   |   {cur['moves']} moves"
        moving_for = (cur["ms"] - cur["since"]) / 1000
        if cur["state"] == 3:
            color = "tab:orange"
        elif cur["state"] == 2 and moving_for > 2 and MOVE_WARNING:
            line2 = f"moving {moving_for:.0f} s without a stop: position no longer reliable, hold it still"
            color = "tab:orange"
        if stat:
            line2 += f"   |   last move {stat['lastMoveTime']:.1f} s, leftover speed {stat['lastMoveVres']:.2f} m/s"
        title.set_text((TITLE_PREFIX + "\n" if TITLE_PREFIX else "") + head + "\n" + line2)
        title.set_color(color)
        roll, pitch, yaw = euler(cur["q"])
        extra = ""
        if stat:
            extra = (f"\nsamples {stat['accHz']:.0f}/{stat['gyrHz']:.0f} Hz   dropped {int(stat['rawDrops'])}"
                     f"   glitches {int(stat['accGlitches'])}/{int(stat['gyrOverruns'])}")
        numbers.set_text(f"x {pos[0]:+.3f}  y {pos[1]:+.3f}  z {pos[2]:+.3f} m      "
                         f"roll {roll:+6.1f}  pitch {pitch:+6.1f}  heading {yaw:5.1f}" + extra)
        return []

    def on_key(e):
        if e.key == "z":
            if link:
                link.send(b"z")
            path[:] = [np.zeros(3)]
            marks.clear()
        elif e.key == "y" and link:
            link.send(b"y")
            path[:] = [cur["p"].copy()]
            marks.clear()
        elif e.key == "c":
            path[:] = [cur["p"].copy()]
            marks.clear()
        elif e.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    if args.save:
        while pending:
            pump()
        draw()
        fig.savefig(args.save, dpi=110)
        print("saved", args.save)
        return
    from matplotlib import animation
    anim = animation.FuncAnimation(fig, draw, interval=40, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        if link:
            link.halt.set()
            time.sleep(0.2)
    del anim


if __name__ == "__main__":
    main()
