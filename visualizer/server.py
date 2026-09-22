#!/usr/bin/env python3
"""Local server for the 3D IMU visualizer.

Serves ./web and streams the board's motion data to the browser over
Server-Sent Events. Only listens on localhost.

    python server.py            # find the Teensy automatically
    python server.py --port COM7
    python server.py --demo     # scripted route, no board needed
    python server.py --record session.csv        # also save what the board sends
    python server.py --replay session.csv        # play a saved session back
"""
import argparse
import json
import queue
import sys
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from bridge import CalStore, Hub, Replayer, SerialLink, Simulator, valid_cal

HERE = Path(__file__).resolve().parent
WEB_DIR = HERE / "web"
LOCAL_HOSTS = {"localhost", "127.0.0.1"}


class Handler(SimpleHTTPRequestHandler):
    hub = None
    link = None  # SerialLink, or None when there is no board
    cal_store = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB_DIR), **kwargs)

    def log_message(self, fmt, *args):
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    # Rejecting foreign Host headers stops DNS-rebinding tricks from other pages.
    def _host_ok(self):
        return (self.headers.get("Host") or "").rsplit(":", 1)[0] in LOCAL_HOSTS

    def _json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not self._host_ok():
            return self.send_error(403)
        path = self.path.split("?")[0]
        if path == "/events":
            return self._stream_events()
        if path == "/api/cal":
            return self._json(self.cal_store.get())
        super().do_GET()

    def do_POST(self):
        # The custom header can't be sent cross-origin without a CORS preflight,
        # which this server never grants, so other websites can't change the calibration.
        if not self._host_ok() or self.headers.get("X-IMU-Viewer") != "1":
            return self.send_error(403)
        if self.path != "/api/cal":
            return self.send_error(404)
        length = int(self.headers.get("Content-Length") or 0)
        try:
            cal = json.loads(self.rfile.read(length)) if 0 < length <= 512 else None
        except ValueError:
            cal = None
        if not isinstance(cal, dict) or not valid_cal(cal):
            return self.send_error(400, "Calibration values out of range")
        self.cal_store.set(cal)
        if self.link is not None:
            self.link.send_cal()
        self._json(self.cal_store.get())

    def _stream_events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        q = self.hub.subscribe()
        try:
            while True:
                try:
                    payload = f"data: {json.dumps(q.get(timeout=10))}\n\n"
                except queue.Empty:
                    payload = ": ping\n\n"
                self.wfile.write(payload.encode())
                self.wfile.flush()
        except OSError:  # the browser tab went away
            pass
        finally:
            self.hub.unsubscribe(q)


def main():
    ap = argparse.ArgumentParser(description="3D IMU visualizer server")
    ap.add_argument("--port", help="serial port such as COM7 (default: auto-detect the Teensy)")
    ap.add_argument("--http-port", type=int, default=8766)
    ap.add_argument("--demo", action="store_true", help="scripted route, no board needed")
    ap.add_argument("--replay", metavar="FILE", help="play back a session saved with --record")
    ap.add_argument("--record", metavar="FILE", help="save the board's fast stream to a file")
    ap.add_argument("--cal-file", default=str(HERE / "accel_cal.json"), help="where the accelerometer calibration is kept")
    ap.add_argument("--skip-cal", action="store_true",
                    help="skip the board's magnetometer calibration prompt automatically")
    ap.add_argument("--no-browser", action="store_true", help="don't open a browser tab")
    args = ap.parse_args()

    hub = Hub()
    recorder = None
    if args.demo or args.replay:
        cal_store = CalStore(None)  # nothing here should end up on a real board
        source, link = (Simulator(hub) if args.demo else Replayer(hub, args.replay)), None
    else:
        try:
            import serial  # noqa: F401
        except ImportError:
            sys.exit("pyserial is not installed: run 'pip install pyserial', or use --demo")
        cal_store = CalStore(args.cal_file)
        if args.record:
            recorder = open(args.record, "w", encoding="utf-8", buffering=1)
        source = link = SerialLink(hub, cal_store, args.port, skip_cal=args.skip_cal, record=recorder)
    Handler.hub, Handler.link, Handler.cal_store = hub, link, cal_store

    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    except OSError as e:
        sys.exit(f"Cannot listen on port {args.http_port}: {e}. Is the visualizer already running?")
    source.start()

    url = f"http://localhost:{args.http_port}/"
    print(f"3D IMU visualizer at {url}  (Ctrl+C to stop)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()
        source.join(timeout=1.5)  # lets the serial thread tell the board to go back to its text output
        server.server_close()
        if recorder:
            recorder.close()


if __name__ == "__main__":
    main()
