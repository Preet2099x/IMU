"""Pieces that get IMU data to the browser: line parsing, an event hub, the
serial reader and a synthetic-data simulator. Stdlib only, plus pyserial for
the real board."""
import math
import queue
import re
import threading
import time

TEENSY_VID_PID = (0x16C0, 0x0483)

_NUM = r"(-?\d+(?:\.\d+)?)"
ATT_RE = re.compile(
    rf"Heading:\s*{_NUM}\s+Pitch:\s*{_NUM}\s+Roll:\s*{_NUM}"
    rf"(?:\s+Gyro\(dps\)\s+X:\s*{_NUM}\s+Y:\s*{_NUM}\s+Z:\s*{_NUM})?"
)


def parse_attitude(line):
    """Turns one firmware output line into an attitude event, or None."""
    m = ATT_RE.search(line)
    if not m:
        return None
    v = [float(g) if g is not None else None for g in m.groups()]
    event = {"type": "att", "heading": v[0], "pitch": v[1], "roll": v[2]}
    if v[3] is not None:
        event["gyro"] = v[3:6]
    return event


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
    text = text.strip()
    if not text:
        return
    event = parse_attitude(text)
    if event:
        event["t"] = time.time() * 1000.0
        hub.publish(event)
    elif not text.startswith("DIAG"):
        hub.publish({"type": "log", "text": text})


def find_teensy_port():
    from serial.tools import list_ports

    for p in list_ports.comports():
        if (p.vid, p.pid) == TEENSY_VID_PID:
            return p.device
    return None


class SerialLink(threading.Thread):
    """Reads the board's serial output and reconnects if it disappears
    (for example while it is being re-flashed)."""

    def __init__(self, hub, port=None, baud=115200, skip_cal=False):
        super().__init__(daemon=True)
        self.hub, self.port, self.baud, self.skip_cal = hub, port, baud, skip_cal
        self._out = queue.Queue()
        self._halt = threading.Event()
        self._session = 0  # bumps on every fresh connection, so the viewer knows the board may have restarted

    def send(self, data: bytes):
        self._out.put(data)

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
                    ser.close()
                except Exception:
                    pass
            self._halt.wait(1.0)

    def _pump(self, ser):
        buf = b""
        while not self._halt.is_set():
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace")
                    handle_line(self.hub, text)
                    if self.skip_cal and "SKIP mag calibration" in text:
                        ser.write(b"s")
            while not self._out.empty():
                ser.write(self._out.get_nowait())
            if len(buf) > 4096:
                buf = b""


class Simulator(threading.Thread):
    """Emits lines in the firmware's format, so the whole pipeline can be tried
    without a board. Each 8 s phase moves one axis, then a combined motion."""

    def __init__(self, hub, rate_hz=10.0):
        super().__init__(daemon=True)
        self.hub, self.period = hub, 1.0 / rate_hz
        self._halt = threading.Event()

    def stop(self):
        self._halt.set()

    @staticmethod
    def pose(t):
        phase, u = int(t // 8) % 4, (t % 8) / 8.0
        s = math.sin(2 * math.pi * u)
        roll = pitch = yaw = 0.0
        if phase == 0:
            yaw = 360.0 * u
        elif phase == 1:
            pitch = 50.0 * s
        elif phase == 2:
            roll = 70.0 * s
        else:
            roll, pitch, yaw = 40.0 * s, 30.0 * math.sin(4 * math.pi * u), 90.0 * s
        return roll, pitch, yaw % 360.0

    def run(self):
        self.hub.set_status("demo", None, "Demo mode: synthetic motion, no board connected", session=1)
        for line in ("=== IMU startup ===", "gyro bias (dps): x=0.150 y=0.200 z=-0.026", "=== running ==="):
            handle_line(self.hub, line)
        t0 = time.time()
        prev = self.pose(0.0)
        while not self._halt.is_set():
            t = time.time() - t0
            cur = self.pose(t)
            gx, gy, gz = [((c - p + 540) % 360 - 180) / self.period for c, p in zip(cur, prev)]
            prev = cur
            roll, pitch, yaw = cur
            handle_line(
                self.hub,
                f"Heading:{yaw:6.1f}  Pitch:{pitch:6.1f}  Roll:{roll:6.1f}  "
                f"Gyro(dps) X:{gx:7.2f} Y:{gy:7.2f} Z:{gz:7.2f}",
            )
            self._halt.wait(self.period)
