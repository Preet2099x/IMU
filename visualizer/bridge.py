"""Gets the board's fast stream to the browser: line parsing, an event hub, the
serial reader, the stored accelerometer calibration, and a simulator.
Stdlib only, plus pyserial for the real board."""
import json
import math
import queue
import threading
import time
from pathlib import Path

TEENSY_VID_PID = (0x16C0, 0x0483)
KEEPALIVE_S = 1.0  # the board drops back to its normal text output after 3 s without one
G = 9.80665


def parse_viz(line):
    """VIZ,ms,q0,q1,q2,q3,fx,fy,fz,gx,gy,gz -> event, or None.
    f is the accelerometer in g, sensor frame, as the chip reports it."""
    f = line.strip().split(",")
    if len(f) != 12 or f[0] != "VIZ":
        return None
    try:
        v = [float(x) for x in f[1:]]
    except ValueError:
        return None
    return {"type": "viz", "ms": v[0], "q": v[1:5], "f": v[5:8], "g": v[8:11]}


def parse_acal(line):
    """ACAL,ox,oy,oz,sx,sy,sz (the board confirming a calibration) -> dict, or None."""
    f = line.strip().split(",")
    if len(f) != 7 or f[0] != "ACAL":
        return None
    try:
        v = [float(x) for x in f[1:]]
    except ValueError:
        return None
    return {"offset": v[:3], "scale": v[3:]}


# --- accelerometer calibration, kept on this computer and sent to the board ---

IDENTITY_CAL = {"offset": [0.0, 0.0, 0.0], "scale": [1.0, 1.0, 1.0]}


def valid_cal(cal):
    try:
        o, s = cal["offset"], cal["scale"]
        return (
            len(o) == 3 and len(s) == 3
            and all(isinstance(x, (int, float)) and math.isfinite(x) for x in [*o, *s])
            and all(abs(x) < 0.5 for x in o)
            and all(0.5 < x < 2.0 for x in s)
        )
    except (KeyError, TypeError):
        return False


def cal_command(cal):
    """The line the firmware understands: c,ox,oy,oz,sx,sy,sz"""
    return ("c," + ",".join("%.6f" % x for x in [*cal["offset"], *cal["scale"]]) + "\n").encode()


class CalStore:
    """The calibration in use. Backed by a file, or by memory only if path is None."""

    def __init__(self, path=None):
        self.path = Path(path) if path else None
        self.cal = None
        if self.path and self.path.exists():
            try:
                loaded = json.loads(self.path.read_text(encoding="utf-8"))
                if valid_cal(loaded):
                    self.cal = {"offset": loaded["offset"], "scale": loaded["scale"]}
            except (OSError, ValueError):
                pass

    def get(self):
        return self.cal or IDENTITY_CAL

    def set(self, cal):
        if not valid_cal(cal):
            raise ValueError("calibration values out of range")
        self.cal = {"offset": [float(x) for x in cal["offset"]], "scale": [float(x) for x in cal["scale"]]}
        if self.path:
            self.path.write_text(json.dumps(self.cal, indent=2), encoding="utf-8")


class Hub:
    """Fans events out to every connected browser and remembers the last status."""

    def __init__(self):
        self._lock = threading.Lock()
        self._clients = set()
        self.status = {"type": "status", "state": "waiting", "port": None, "message": "Starting"}

    def subscribe(self):
        q = queue.Queue(maxsize=500)
        with self._lock:
            self._clients.add(q)
            q.put_nowait(self.status)
        return q

    def unsubscribe(self, q):
        with self._lock:
            self._clients.discard(q)

    def publish(self, event):
        with self._lock:
            if event["type"] == "status":
                self.status = event
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(event)
            except queue.Full:  # a stalled browser tab: drop its oldest event
                try:
                    q.get_nowait()
                    q.put_nowait(event)
                except (queue.Empty, queue.Full):
                    pass

    def set_status(self, state, port=None, message="", **extra):
        self.publish({"type": "status", "state": state, "port": port, "message": message, **extra})


def handle_line(hub, text):
    """Publishes one line from the board. Returns 'viz', 'text' (its normal
    Heading output) or None, so the caller knows what the board is doing."""
    text = text.strip()
    if not text:
        return None
    event = parse_viz(text)
    if event:
        hub.publish(event)
        return "viz"
    if text.startswith("Heading:"):
        return "text"
    acal = parse_acal(text)
    if acal:
        hub.publish({"type": "acal", **acal})
    elif not text.startswith("DIAG"):
        hub.publish({"type": "log", "text": text})
    return None


def find_teensy_port():
    from serial.tools import list_ports

    for p in list_ports.comports():
        if (p.vid, p.pid) == TEENSY_VID_PID:
            return p.device
    return None


class SerialLink(threading.Thread):
    """Reads the board, asks it for the fast stream once it is running, gives it
    the stored calibration, and reconnects if it disappears (for example while
    being re-flashed)."""

    def __init__(self, hub, cal_store, port=None, baud=115200, skip_cal=False, record=None):
        super().__init__(daemon=True)
        self.hub, self.cal_store, self.port, self.baud, self.skip_cal = hub, cal_store, port, baud, skip_cal
        self.record = record  # open file: every VIZ line is written to it
        self._out = queue.Queue()
        self._halt = threading.Event()
        self._session = 0

    def send(self, data: bytes):
        self._out.put(data)

    def send_cal(self):
        self.send(cal_command(self.cal_store.get()))

    def stop(self):
        self._halt.set()

    def run(self):
        import serial

        while not self._halt.is_set():
            port = self.port or find_teensy_port()
            if not port:
                self.hub.set_status("waiting", None, "No Teensy found (USB 16C0:0483). Plug it in.")
                self._halt.wait(1.0)
                continue
            try:
                ser = serial.Serial(port, self.baud, timeout=0.1)
            except (serial.SerialException, OSError) as e:
                self.hub.set_status("busy", port, f"Cannot open {port}. Close any other serial monitor. ({e})")
                self._halt.wait(2.0)
                continue

            self._session += 1
            self.hub.set_status("connected", port, f"Connected to {port}", session=self._session)
            try:
                self._pump(ser)
            except (serial.SerialException, OSError) as e:
                self.hub.set_status("waiting", port, f"Lost {port}: {e}")
            finally:
                try:
                    ser.write(b"h")  # hand the board back to its normal text output
                    ser.close()
                except Exception:
                    pass
            self._halt.wait(1.0)

    def _pump(self, ser):
        buf = b""
        running = False  # the board has finished its startup and prints data
        running_since = None
        warned = False
        last_keepalive = 0.0
        while not self._halt.is_set():
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace")
                    kind = handle_line(self.hub, text)
                    if kind == "viz" and self.record:
                        self.record.write(text.strip() + "\n")
                    if kind and not running:
                        running, running_since = True, time.time()
                        ser.write(cal_command(self.cal_store.get()))  # the board forgets it on every reset
                    if kind == "viz":
                        warned = True
                    if self.skip_cal and "SKIP mag calibration" in text:
                        ser.write(b"s")
            now = time.time()
            # Only once it is running: any key press during startup would skip calibration.
            if running and now - last_keepalive >= KEEPALIVE_S:
                ser.write(b"v")
                last_keepalive = now
            if running and not warned and now - running_since > 4.0:
                warned = True
                self.hub.publish({"type": "log", "text": "The board is not sending the fast stream. Re-flash the latest firmware (pio run -t upload)."})
            while not self._out.empty():
                ser.write(self._out.get_nowait())
            if len(buf) > 4096:
                buf = b""


# --- demo and replay sources -------------------------------------------------

def _quat(roll, pitch, yaw):
    cr, sr = math.cos(math.radians(roll) / 2), math.sin(math.radians(roll) / 2)
    cp, sp = math.cos(math.radians(pitch) / 2), math.sin(math.radians(pitch) / 2)
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def _rotate(q, v):
    """q * v * q^-1 for a unit quaternion [w, x, y, z]."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v);  v' = v + w*t + q_vec x t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return [vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)]


def _min_jerk(tau):
    """Position fraction and its second derivative for a smooth 0..1 move."""
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    return s, 60 * tau - 180 * tau**2 + 120 * tau**3


# (x front, y left, z up) in metres, and how long to rest and to move.
DEMO_PATH = [(0.5, 0.0, 0.0), (0.5, 0.5, 0.0), (0.0, 0.5, 0.35), (-0.4, 0.0, 0.0), (0.0, 0.0, 0.0)]
DEMO_REST, DEMO_MOVE = 2.0, 1.3
# A sensor with the kind of errors the real one has.
DEMO_SCALE, DEMO_OFFSET = (1.004, 0.995, 0.979), (0.006, -0.010, 0.014)


def demo_sample(t):
    """(acceleration in the earth frame in m/s^2, (roll, pitch) tilt) at time t of the demo route."""
    period = DEMO_REST + DEMO_MOVE
    leg, u = int(t // period) % len(DEMO_PATH), t % period
    prev = DEMO_PATH[leg - 1] if leg else DEMO_PATH[-1]
    dest = DEMO_PATH[leg]
    if u < DEMO_REST:
        acc = (0.0, 0.0, 0.0)
    else:
        _, s2 = _min_jerk((u - DEMO_REST) / DEMO_MOVE)
        acc = tuple((d - p) * s2 / DEMO_MOVE**2 for d, p in zip(dest, prev))
    # Leaning into the push, as a hand would.
    pitch, roll = max(-15, min(15, -acc[0] * 4)), max(-15, min(15, acc[1] * 4))
    return acc, (roll, pitch)


def demo_raw(acc, tilt, scale=DEMO_SCALE, offset=DEMO_OFFSET):
    """What the accelerometer reports (g, sensor frame) for that motion and tilt."""
    q = _quat(tilt[0], tilt[1], 0.0)
    conj = [q[0], -q[1], -q[2], -q[3]]
    f_true = _rotate(conj, [acc[0] / G, acc[1] / G, acc[2] / G + 1.0])
    return [f * s + o for f, s, o in zip(f_true, scale, offset)]


class Simulator(threading.Thread):
    """Emits the board's fast stream for a scripted route, with sensor noise and
    errors like the real accelerometer's, so the viewer can be tried without a board."""

    def __init__(self, hub, rate_hz=50.0):
        super().__init__(daemon=True)
        self.hub, self.period = hub, 1.0 / rate_hz
        self._halt = threading.Event()

    def stop(self):
        self._halt.set()

    def run(self):
        import random

        rng = random.Random(4)
        self.hub.set_status("demo", None, "Demo: scripted route, no board connected", session=1)
        for line in ("=== IMU startup ===", "gyro bias (dps): x=0.150 y=0.200 z=-0.026", "=== running ==="):
            handle_line(self.hub, line)
        t0 = time.time()
        prev_tilt = (0.0, 0.0)
        while not self._halt.is_set():
            t = time.time() - t0
            acc, tilt = demo_sample(t)
            q = _quat(tilt[0], tilt[1], 0.0)
            f = [v + rng.gauss(0, 0.002) for v in demo_raw(acc, tilt)]
            g = [(tilt[0] - prev_tilt[0]) / self.period + rng.gauss(0, 0.2), (tilt[1] - prev_tilt[1]) / self.period + rng.gauss(0, 0.2), rng.gauss(0, 0.2)]
            prev_tilt = tilt
            handle_line(self.hub, "VIZ,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f" % (t * 1000, *q, *f, *g))
            self._halt.wait(self.period)


class Replayer(threading.Thread):
    """Plays back a file recorded with --record, at the speed it was recorded, forever."""

    def __init__(self, hub, path):
        super().__init__(daemon=True)
        self.hub, self.path = hub, Path(path)
        self._halt = threading.Event()

    def stop(self):
        self._halt.set()

    def run(self):
        lines = [l for l in self.path.read_text(encoding="utf-8").splitlines() if parse_viz(l)]
        if len(lines) < 2:
            self.hub.set_status("waiting", None, f"Nothing to replay in {self.path}")
            return
        session = 0
        while not self._halt.is_set():
            session += 1
            self.hub.set_status("demo", None, f"Replaying {self.path.name}", session=session)
            start_ms = parse_viz(lines[0])["ms"]
            t0 = time.time()
            for line in lines:
                due = t0 + (parse_viz(line)["ms"] - start_ms) / 1000.0
                if self._halt.wait(max(0.0, due - time.time())):
                    return
                handle_line(self.hub, line)
            self._halt.wait(1.5)
