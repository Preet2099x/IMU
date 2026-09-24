"""Live numbers from the board in a web page: accelerometer, gyro, magnetometer
(x, y, z) plus pitch, roll and heading.

    python dashboard/server.py --port socket://192.168.137.200:8888     (WiFi)
    python dashboard/server.py --port COM8                              (USB)

Needs pyserial. Only one program can talk to the board's WiFi port at a time,
so close the 3D visualizer first."""
import argparse
import json
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

HERE = Path(__file__).parent
KEYS = ["ms", "heading", "pitch", "roll", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz"]
state = {"connected": False, "message": "Connecting...", "data": None, "updated": 0.0}


def parse_log(line):
    """LOG,ms,heading,pitch,roll,ax,ay,az,gx,gy,gz,mx,my,mz -> dict, or None."""
    f = line.strip().split(",")
    if len(f) != len(KEYS) + 1 or f[0] != "LOG":
        return None
    try:
        return dict(zip(KEYS, (float(x) for x in f[1:])))
    except ValueError:
        return None


def reader(port, baud, skip_cal):
    while True:
        try:
            ser = serial.serial_for_url(port, baud, timeout=0.1)
        except (serial.SerialException, OSError) as e:
            state.update(connected=False, message=f"Cannot open {port}: {e}")
            time.sleep(2)
            continue
        state.update(connected=True, message=f"Connected to {port}")
        buf, running, last_keepalive = b"", False, 0.0
        try:
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace")
                    if skip_cal and "SKIP mag calibration" in text:
                        ser.write(b"s")
                    if "=== running ===" in text or text.startswith(("Heading:", "LOG,")):
                        running = True
                    d = parse_log(text)
                    if d:
                        state.update(data=d, updated=time.time())
                if len(buf) > 4096:
                    buf = b""
                now = time.time()
                if running and now - last_keepalive >= 1.0:  # 'l' keeps the board's log stream on
                    ser.write(b"l")
                    last_keepalive = now
        except (serial.SerialException, OSError) as e:
            state.update(connected=False, message=f"Lost {port}: {e}")
        finally:
            try:
                ser.write(b"h")
                ser.close()
            except Exception:
                pass
        time.sleep(1)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/data":
            body = json.dumps({**state, "age": time.time() - state["updated"] if state["updated"] else None}).encode()
            ctype = "application/json"
        else:
            body, ctype = (HERE / "index.html").read_bytes(), "text/html; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser(description="Live IMU numbers in the browser")
    ap.add_argument("--port", required=True, help="COM8, or socket://192.168.137.200:8888 for WiFi")
    ap.add_argument("--http-port", type=int, default=8767)
    ap.add_argument("--skip-cal", action="store_true", help="skip the board's 30 s magnetometer calibration")
    ap.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 to open the page from a phone on the same WiFi")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()
    threading.Thread(target=reader, args=(args.port, 115200, args.skip_cal), daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.http_port), Handler)
    url = f"http://localhost:{args.http_port}/"
    print("Dashboard at", url)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
