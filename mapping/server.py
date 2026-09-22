#!/usr/bin/env python3
"""Local server for the 3D IMU viewer.

Serves ./web and streams the board's attitude to the browser over
Server-Sent Events. Only listens on localhost.

    python server.py            # find the Teensy automatically
    python server.py --port COM7
    python server.py --demo     # synthetic motion, no board needed
"""
import argparse
import json
import queue
import sys
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from bridge import Hub, SerialLink, Simulator

WEB_DIR = Path(__file__).resolve().parent / "web"
LOCAL_HOSTS = {"localhost", "127.0.0.1"}


class Handler(SimpleHTTPRequestHandler):
    hub = None
    link = None  # SerialLink, or None in demo mode

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

    def do_GET(self):
        if not self._host_ok():
            return self.send_error(403)
        if self.path.split("?")[0] == "/events":
            return self._stream_events()
        super().do_GET()

    def do_POST(self):
        # The custom header can't be sent cross-origin without a CORS preflight,
        # which this server never grants, so other websites can't write to the board.
        if not self._host_ok() or self.headers.get("X-IMU-Viewer") != "1":
            return self.send_error(403)
        if self.path != "/send":
            return self.send_error(404)
        length = int(self.headers.get("Content-Length") or 0)
        data = self.rfile.read(length) if 0 < length <= 32 else b""
        if not data or not data.isascii() or not data.decode().isprintable():
            return self.send_error(400)
        if self.link is None:
            return self.send_error(409, "Demo mode has no board to send to")
        self.link.send(data)
        self.send_response(204)
        self.end_headers()

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
    ap = argparse.ArgumentParser(description="3D IMU viewer server")
    ap.add_argument("--port", help="serial port such as COM7 (default: auto-detect the Teensy)")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--demo", action="store_true", help="synthetic motion, no board needed")
    ap.add_argument("--skip-cal", action="store_true",
                    help="skip the board's magnetometer calibration prompt automatically")
    ap.add_argument("--no-browser", action="store_true", help="don't open a browser tab")
    args = ap.parse_args()

    hub = Hub()
    if args.demo:
        source, link = Simulator(hub), None
    else:
        try:
            import serial  # noqa: F401
        except ImportError:
            sys.exit("pyserial is not installed: run 'pip install pyserial', or use --demo")
        source = link = SerialLink(hub, args.port, skip_cal=args.skip_cal)
    Handler.hub, Handler.link = hub, link

    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    except OSError as e:
        sys.exit(f"Cannot listen on port {args.http_port}: {e}. Is the viewer already running?")
    source.start()

    url = f"http://localhost:{args.http_port}/"
    print(f"3D IMU viewer at {url}  (Ctrl+C to stop)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()
        server.server_close()


if __name__ == "__main__":
    main()
