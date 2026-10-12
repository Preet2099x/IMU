"""Live 3D view of the IMU board and its moves, in the browser.

All the measuring happens on the Teensy. This reads the board's serial output,
saves all of it under logs/, and serves a web page (tools/web/) that draws what
the firmware's '@' lines say (see the "Dead reckoning" comment in src/main.cpp).
The lines reach the page as Server-Sent Events; the page sends the board's
keys (f, z, r) back with a POST.

    python tools/motion_viewer.py                  # find the Teensy, open the browser
    python tools/motion_viewer.py --mag            # don't auto-skip mag calibration
    python tools/motion_viewer.py --replay logs/moves_20261005_130652.txt
    python tools/motion_viewer.py --replay FILE --speed 4
"""

import argparse
import queue
import threading
import time
import webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
from serial.tools import list_ports

TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483
TOOLS_DIR = Path(__file__).resolve().parent
WEB_DIR = TOOLS_DIR / "web"
LOG_DIR = TOOLS_DIR.parent / "logs"
LIVE_PERIOD_S = 0.04  # the firmware sends @LIVE 25 times a second
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
}


class Hub:
    """Fans the board's lines out to every open page, and keeps enough of them
    to bring a newly opened (or reloaded) page up to date."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = []
        self.link = "#link waiting for the board"
        self.state = None
        self.steps = None
        self.live = None
        self.history = []  # @PATH, @MOVE and @INFO since the last @RESET

    def publish(self, line):
        with self.lock:
            if line.startswith("#link "):
                self.link = line
            elif line.startswith("@LIVE"):
                self.live = line
            elif line.startswith("@STATE"):
                self.state = line
            elif line.startswith("@STEPS"):
                self.steps = line  # the step count outlives a position reset
            elif line.startswith("@RESET"):
                self.history = []
            elif line.startswith("@"):
                self.history.append(line)
            else:
                return  # readable lines stay in the console and the log
            for q in self.clients:
                q.put(line)

    def subscribe(self):
        q = queue.Queue()
        with self.lock:
            # "#hello" tells the page to start from scratch; "#caught-up" ends the
            # catch-up, so the page doesn't pop up old notices as new.
            catch_up = ["#hello", self.link, "@RESET", *self.history, self.state, self.steps, self.live,
                        "#caught-up"]
            for line in catch_up:
                if line:
                    q.put(line)
            self.clients.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            self.clients.remove(q)

    def has_clients(self):
        with self.lock:
            return bool(self.clients)


def find_teensy():
    for p in list_ports.comports():
        if p.vid == TEENSY_VID and p.pid == TEENSY_PID:
            return p.device
    return None


class Link:
    """Serial connection to the board that survives it being unplugged."""

    def __init__(self, port, hub, log_path, skip_mag):
        self.port, self.hub, self.log_path, self.skip_mag = port, hub, log_path, skip_mag
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
                    self.hub.publish("#link Teensy not found - is it plugged in?")
                    time.sleep(1.0)
                    continue
                try:
                    with serial.Serial(port, 115200, timeout=0.05) as ser:
                        self.ser = ser
                        self.hub.publish(f"#link connected on {port}")
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
                                self.hub.publish(line)
                            log.flush()
                except serial.SerialException as e:
                    self.hub.publish(f"#link can't use {port} ({e}) - close any other serial monitor")
                finally:
                    self.ser = None
                time.sleep(1.0)


def replay(path, hub, speed):
    """Plays a saved log back at its real pace, once a page is watching."""
    while not hub.has_clients():
        time.sleep(0.2)
    hub.publish(f"#link replay of {path.name}")
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        hub.publish(line)
        if line.startswith("@LIVE"):
            time.sleep(LIVE_PERIOD_S / speed)
    hub.publish(f"#link replay of {path.name} finished")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # keep the console for the board's own output

    def do_GET(self):
        if self.path == "/events":
            self._events()
            return
        name = "index.html" if self.path in ("/", "/index.html") else self.path.split("?")[0].lstrip("/")
        file = (WEB_DIR / name).resolve()
        if WEB_DIR not in file.parents or not file.is_file():
            self.send_error(404)
            return
        body = file.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", CONTENT_TYPES.get(file.suffix, "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        hub = self.server.hub
        q = hub.subscribe()
        try:
            while True:
                try:
                    line = q.get(timeout=15.0)
                    self.wfile.write(f"data: {line}\n\n".encode())
                except queue.Empty:
                    self.wfile.write(b": still here\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        finally:
            hub.unsubscribe(q)

    def do_POST(self):
        if self.path != "/cmd":
            self.send_error(404)
            return
        key = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode().strip()
        link = self.server.link
        if key in ("f", "z", "r") and link:
            link.send(key.encode())
        self.send_response(204)
        self.end_headers()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: find the Teensy)")
    ap.add_argument("--mag", action="store_true", help="let the board run its mag calibration")
    ap.add_argument("--replay", type=Path, help="play a saved log instead of the live board")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed (default 1 = real time)")
    ap.add_argument("--http-port", type=int, default=8765, help="port for the web page (default 8765)")
    ap.add_argument("--no-browser", action="store_true", help="don't open the browser")
    args = ap.parse_args()

    hub = Hub()
    link = None
    if args.replay:
        threading.Thread(target=replay, args=(args.replay, hub, args.speed), daemon=True).start()
    else:
        LOG_DIR.mkdir(exist_ok=True)
        log_path = LOG_DIR / f"moves_{datetime.now():%Y%m%d_%H%M%S}.txt"
        print(f"saving output to {log_path}")
        link = Link(args.port, hub, log_path, skip_mag=not args.mag)

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    server.daemon_threads = True
    server.hub, server.link = hub, link
    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"viewer at {url}  (Ctrl+C to stop)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()