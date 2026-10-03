"""Live map of where the IMU board is and how it has moved.

All the measuring happens on the Teensy. This only draws what the firmware's
'@' lines say (see the "Dead reckoning" comment in src/main.cpp): the board's
live position and axes, the trail of the move under way, and the corrected
path of every finished move, in 3D and from above. Everything the board prints is also
echoed here and saved under logs/.

    python tools/move_plot.py                 # find the Teensy and plot live
    python tools/move_plot.py --mag           # don't auto-skip mag calibration
    python tools/move_plot.py --replay logs/moves_20261003_120000.txt

Keys in the plot window: f then a push forward sets which way is forward,
z zeroes the board's position, q quits.
"""

import argparse
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import serial
from serial.tools import list_ports

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483
LOG_DIR = Path(__file__).resolve().parent.parent / "logs"
FRAME_MS = 50

STATE_COLOR = {"READY": "tab:green", "MOVING": "tab:orange"}
AXIS_COLORS = ("tab:red", "tab:green", "tab:blue")  # board x, y, z


class Board:
    """What the firmware has said so far."""

    def __init__(self):
        self.state = "CONNECTING"
        self.message = "waiting for the board"
        self.version = 0  # bumps whenever the finished moves change
        self.live = None  # (position cm, (board x, y, z axes)) in the earth frame
        self.reset()

    def reset(self):
        self.moves = []  # (n, start xyz cm, end xyz cm)
        self.paths = {}  # n -> the path the move took, xyz cm
        self.trail = []  # live positions during the move under way
        self.headline = ""
        self.version += 1

    def feed(self, line):
        if line.startswith("#link "):
            self.state, self.message = "CONNECTING", line[len("#link "):]
            return
        if not line.startswith("@"):
            return
        kind, _, rest = line[1:].partition(" ")
        if kind == "LIVE":
            v = [float(x) for x in rest.split()]
            self.live = (v[0:3], (v[3:6], v[6:9], v[9:12]))
            if self.state == "MOVING":
                self.trail.append(v[0:3])
        elif kind == "STATE":
            state, _, self.message = rest.partition(" ")
            if state != "MOVING":
                self.trail = []
            self.state = state
        elif kind == "PATH":
            n, *v = rest.split()
            v = [float(x) for x in v]
            self.paths[int(n)] = [v[i:i + 3] for i in range(0, len(v) - 2, 3)]
        elif kind == "MOVE":
            numbers, _, text = rest.partition(" | ")
            v = numbers.split()
            self.moves.append((int(v[0]), [float(x) for x in v[1:4]], [float(x) for x in v[4:7]]))
            self.headline = text
            self.trail = []
            self.version += 1
        elif kind == "INFO":
            self.headline = rest
            self.version += 1
        elif kind == "RESET":
            self.reset()


def find_teensy():
    for p in list_ports.comports():
        if p.vid == TEENSY_VID and p.pid == TEENSY_PID:
            return p.device
    return None


class Link:
    """Serial connection that survives the board being unplugged and replugged."""

    def __init__(self, port, lines, log_path, skip_mag):
        self.port, self.lines, self.log_path, self.skip_mag = port, lines, log_path, skip_mag
        self.ser = None
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, data):
        ser = self.ser
        if ser:
            try:
                ser.write(data)
            except serial.SerialException:
                pass

    def _run(self):
        with open(self.log_path, "a", encoding="utf-8") as log:
            while True:
                port = self.port or find_teensy()
                if not port:
                    self.lines.put("#link Teensy not found - is it plugged in?")
                    time.sleep(1.0)
                    continue
                try:
                    with serial.Serial(port, 115200, timeout=0.05) as ser:
                        self.ser = ser
                        self.lines.put(f"#link connected on {port}, waiting for the board")
                        buf = b""
                        while True:
                            buf += ser.read(4096)
                            while b"\n" in buf:
                                raw, buf = buf.split(b"\n", 1)
                                line = raw.decode(errors="replace").rstrip()
                                log.write(line + "\n")
                                if not line.startswith("@LIVE"):  # 25 a second, too many to read
                                    print(line, flush=True)
                                if self.skip_mag and "SKIP mag calibration" in line:
                                    ser.write(b"s")
                                self.lines.put(line)
                            log.flush()
                except serial.SerialException as e:
                    self.lines.put(f"#link can't use {port} ({e}) - close any other serial monitor")
                finally:
                    self.ser = None
                time.sleep(1.0)


class View:
    """The figure. Finished moves are redrawn only when they change; the live
    dot, trail and board axes are cheap artists updated every frame."""

    def __init__(self, board):
        self.board = board
        self.fig = plt.figure(figsize=(13, 6.5))
        self.fig.canvas.manager.set_window_title("IMU moves")
        self.ax3d = self.fig.add_subplot(1, 2, 1, projection="3d")
        self.ax3d.view_init(elev=22, azim=-150)  # from behind and to the right of the start
        self.ax2d = self.fig.add_subplot(1, 2, 2)
        self.fig.subplots_adjust(left=0.03, right=0.97, top=0.88, bottom=0.12, wspace=0.15)
        self.title = self.fig.suptitle("", fontsize=15, fontweight="bold")
        self.info = self.fig.text(0.5, 0.025, "", ha="center", fontsize=12)
        self.drawn_version = None
        self.center, self.half = [0.0, 0.0, 0.0], 20.0

    def _fit(self, pts):
        lo = [min(p[k] for p in pts) for k in range(3)]
        hi = [max(p[k] for p in pts) for k in range(3)]
        self.center = [(a + b) / 2 for a, b in zip(lo, hi)]
        # Headroom, so a moving dot doesn't force a full redraw every frame.
        self.half = max(20.0, max(b - a for a, b in zip(lo, hi)) / 2 * 1.3 + 8.0)

    def _outside(self, p):
        return any(abs(p[k] - self.center[k]) > self.half * 0.95 for k in range(3))

    def redraw_moves(self):
        b, ax3d, ax2d = self.board, self.ax3d, self.ax2d
        elev, azim = ax3d.elev, ax3d.azim  # keep whatever view the user dragged to
        ax3d.cla()
        ax2d.cla()

        pts = [[0.0, 0.0, 0.0]] + [p for _, s, e in b.moves for p in (s, e)] + b.trail
        pts += [p for path in b.paths.values() for p in path]
        if b.live:
            pts.append(b.live[0])
        self._fit(pts)
        c, h = self.center, self.half

        for i, (n, s, e) in enumerate(b.moves):
            latest = i == len(b.moves) - 1
            color = "tab:purple" if latest else "tab:blue"
            alpha = 1.0 if latest else 0.5
            xs, ys, zs = zip(*b.paths.get(n, [s, e]))
            # The arrowhead points the way the move went overall.
            d = [q - p for p, q in zip(s, e)]
            length = max(sum(v * v for v in d) ** 0.5, 1e-6)
            head = [v / length * h * 0.1 for v in d]

            ax3d.plot(xs, ys, zs, "-", color=color, alpha=alpha, lw=2.5)
            ax3d.quiver(*[p - q for p, q in zip(e, head)], *head, color=color, alpha=alpha,
                        linewidth=2.5, arrow_length_ratio=1.0)
            ax3d.text(*e, f" {n}", color=color, fontsize=10)
            # From above: left is drawn to the left, forward is up the screen.
            ax2d.plot([-y for y in ys], xs, "-", color=color, alpha=alpha, lw=2.5)
            ax2d.annotate("", xy=(-e[1], e[0]), xytext=(-e[1] + head[1] * 0.1, e[0] - head[0] * 0.1),
                          arrowprops=dict(arrowstyle="-|>", lw=2.5, color=color, alpha=alpha,
                                          mutation_scale=18))
            ax2d.text(-e[1], e[0], f" {n}", color=color, fontsize=10, va="bottom")

        ax3d.scatter([0], [0], [0], color="black", s=30)
        ax3d.set_xlim(c[0] - h, c[0] + h)
        ax3d.set_ylim(c[1] - h, c[1] + h)
        ax3d.set_zlim(c[2] - h, c[2] + h)
        ax3d.set_box_aspect((1, 1, 1))
        ax3d.set_xlabel("X forward (cm)")
        ax3d.set_ylabel("Y left (cm)")
        ax3d.set_zlabel("Z up (cm)")
        ax3d.view_init(elev=elev, azim=azim)
        ax3d.set_title("3D (drag to rotate)")

        ax2d.plot([0], [0], "ko", ms=6)
        ax2d.set_xlim(-c[1] - h, -c[1] + h)
        ax2d.set_ylim(c[0] - h, c[0] + h)
        ax2d.set_aspect("equal")
        ax2d.grid(True, alpha=0.3)
        ax2d.set_xlabel("<- left        right ->   (cm)")
        ax2d.set_ylabel("forward (cm)")

        # Live artists, given their data every frame.
        self.trail3d, = ax3d.plot([], [], [], "-", color="tab:orange", lw=2)
        self.axes3d = [ax3d.plot([], [], [], "-", color=col, lw=3)[0] for col in AXIS_COLORS]
        self.dot3d, = ax3d.plot([], [], [], "o", color="tab:red", ms=10)
        self.trail2d, = ax2d.plot([], [], "-", color="tab:orange", lw=2)
        self.nose2d, = ax2d.plot([], [], "-", color=AXIS_COLORS[0], lw=3)
        self.dot2d, = ax2d.plot([], [], "o", color="tab:red", ms=11)
        self.drawn_version = b.version

    def update_live(self):
        b = self.board
        if b.trail:
            xs, ys, zs = zip(*b.trail)
            self.trail3d.set_data_3d(xs, ys, zs)
            self.trail2d.set_data([-y for y in ys], xs)
        else:
            self.trail3d.set_data_3d([], [], [])
            self.trail2d.set_data([], [])

        if b.live:
            p, axes = b.live
            size = self.half * 0.3
            for line, a in zip(self.axes3d, axes):
                line.set_data_3d([p[0], p[0] + size * a[0]], [p[1], p[1] + size * a[1]],
                                 [p[2], p[2] + size * a[2]])
            self.dot3d.set_data_3d([p[0]], [p[1]], [p[2]])
            nose = axes[0]
            self.nose2d.set_data([-p[1], -p[1] - size * nose[1]], [p[0], p[0] + size * nose[0]])
            self.dot2d.set_data([-p[1]], [p[0]])
            self.ax2d.set_title(f"From above (red line = board's x axis)   |   height {p[2]:+.1f} cm")

        self.title.set_text(f"{b.state}: {b.message}")
        self.title.set_color(STATE_COLOR.get(b.state, "dimgray"))
        self.info.set_text(b.headline or "No moves yet. Press F, then push the board forward once "
                           "to set forward. Z zeroes.")

    def frame(self):
        b = self.board
        if b.version != self.drawn_version or (b.live and self._outside(b.live[0])):
            self.redraw_moves()
        self.update_live()
        self.fig.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: find the Teensy)")
    ap.add_argument("--mag", action="store_true", help="let the board run its mag calibration")
    ap.add_argument("--replay", type=Path, help="draw a saved log instead of the live board")
    ap.add_argument("--save", type=Path, help="with --replay: save the picture here and exit")
    args = ap.parse_args()

    plt.rcParams["keymap.fullscreen"] = ["ctrl+f"]  # 'f' sets forward here
    board = Board()
    view = View(board)

    if args.replay:
        for line in args.replay.read_text(encoding="utf-8", errors="replace").splitlines():
            board.feed(line)
        board.state, board.message = "REPLAY", args.replay.name
        view.frame()
        if args.save:
            view.fig.savefig(args.save, dpi=110)
            return
        plt.show()
        return

    LOG_DIR.mkdir(exist_ok=True)
    log_path = LOG_DIR / f"moves_{datetime.now():%Y%m%d_%H%M%S}.txt"
    print(f"saving output to {log_path}")
    lines = queue.Queue()
    link = Link(args.port, lines, log_path, skip_mag=not args.mag)

    def on_key(event):
        if event.key in ("z", "f"):
            link.send(event.key.encode())

    def tick():
        while True:
            try:
                board.feed(lines.get_nowait())
            except queue.Empty:
                break
        view.frame()

    view.fig.canvas.mpl_connect("key_press_event", on_key)
    timer = view.fig.canvas.new_timer(interval=FRAME_MS)
    timer.add_callback(tick)
    timer.start()
    view.frame()
    plt.show()


if __name__ == "__main__":
    main()
