"""Finds the IMU board, wherever it is: on USB (Teensy or ESP32), or the ESP32 on
whatever WiFi network this computer is on. Every viewer and logger uses this, so
none of them need the board's address typed in.

    python boardfind.py           # prints what it found

The ESP32 is looked for, in order, at:
  1. the address it had last time (remembered in .board_address, git-ignored),
  2. imu.local (the board announces itself by that name),
  3. every address on this computer's own networks (a quick scan of port 8888).
A candidate only counts if it really talks like the board, so another device that
happens to have that port open is ignored.

Note: the ESP32 serves one program at a time, and connecting to it (which the search
does) makes it drop whoever was connected before. The tools only search when they
have no connection, so this only matters if you start a second one on purpose.
"""
import ipaddress
import re
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

WIFI_PORT = 8888
HOSTNAME = "imu.local"
TEENSY_VID_PID = (0x16C0, 0x0483)
ESP32_VID_PID = (0x10C4, 0xEA60)  # the CP2102 USB-serial chip on the ESP32 board
CACHE = Path(__file__).with_name(".board_address")
SCAN_EVERY_S = 8.0  # a full scan at most this often when a tool keeps retrying

_last_scan = 0.0


def usb_port(kinds=("teensy", "esp32")):
    """First plugged-in board of these kinds, as a COM port name, or None."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    ports = list_ports.comports()
    for kind in kinds:
        wanted = TEENSY_VID_PID if kind == "teensy" else ESP32_VID_PID
        for p in ports:
            if (p.vid, p.pid) == wanted:
                return p.device
    return None


def _talks_like_the_board(host, port=WIFI_PORT, timeout=1.2):
    """Connects and waits for a line only this firmware prints."""
    try:
        s = socket.create_connection((host, port), timeout=0.6)
    except OSError:
        return False
    try:
        s.settimeout(0.3)
        end = time.time() + timeout
        buf = b""
        while time.time() < end and len(buf) < 4096:
            try:
                chunk = s.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                return False
            if not chunk:
                return False
            buf += chunk
            if b"Heading:" in buf or b"TRK," in buf or b"STAT," in buf or b"VIZ," in buf or b"LOG," in buf:
                return True
        return False
    finally:
        try:
            s.close()
        except OSError:
            pass


def _resolve(name, timeout=1.5):
    """name -> IPv4 address, or None. Resolving can hang, so it runs on the side."""
    out = []

    def work():
        try:
            out.append(socket.gethostbyname(name))
        except OSError:
            pass

    t = threading.Thread(target=work, daemon=True)
    t.start()
    t.join(timeout)
    return out[0] if out else None


def local_networks():
    """The /24 networks this computer is on (private addresses only)."""
    addrs = set()
    try:
        addrs.update(socket.gethostbyname_ex(socket.gethostname())[2])
    except OSError:
        pass
    try:  # the address used to reach the outside; no packet is actually sent
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        addrs.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    nets = []
    for a in addrs:
        try:
            ip = ipaddress.ip_address(a)
        except ValueError:
            continue
        if ip.is_private and not ip.is_loopback and not ip.is_link_local:
            net = ipaddress.ip_network(a + "/24", strict=False)
            if net not in nets:
                nets.append(net)
    return nets


def scan(nets=None, port=WIFI_PORT, workers=96):
    """Addresses on the local networks that accept a connection on the board's port."""
    nets = local_networks() if nets is None else nets
    hosts = [str(h) for net in nets for h in net.hosts()]
    found, lock, idx = [], threading.Lock(), [0]

    def work():
        while True:
            with lock:
                if idx[0] >= len(hosts):
                    return
                h = hosts[idx[0]]
                idx[0] += 1
            try:
                socket.create_connection((h, port), timeout=0.35).close()
            except OSError:
                continue
            with lock:
                found.append(h)

    threads = [threading.Thread(target=work, daemon=True) for _ in range(min(workers, max(1, len(hosts))))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return found


def _remember(host):
    try:
        CACHE.write_text(host)
    except OSError:
        pass


def _cached():
    try:
        return CACHE.read_text().strip() or None
    except OSError:
        return None


def find_wifi(full_scan=True):
    """The ESP32's address on the network, or None."""
    global _last_scan
    for lookup in (_cached, lambda: _resolve(HOSTNAME)):  # the cheap one first
        host = lookup()
        if host and _talks_like_the_board(host):
            _remember(host)
            return host
    if full_scan and time.time() - _last_scan >= SCAN_EVERY_S:
        _last_scan = time.time()
        for host in scan():
            if _talks_like_the_board(host):
                _remember(host)
                return host
    return None


def find_port(prefer_wifi=False):
    """A port string a viewer can open: 'COM7' or 'socket://192.168.x.y:8888', or None.
    Order: USB first, then WiFi; the tracking viewers pass prefer_wifi=True because
    the raw samples for replay only come over WiFi."""
    if prefer_wifi:
        host = find_wifi()
        if host:
            return "socket://%s:%d" % (host, WIFI_PORT)
    usb = usb_port()
    if usb:
        return usb
    if not prefer_wifi:
        host = find_wifi()
        if host:
            return "socket://%s:%d" % (host, WIFI_PORT)
    return None


def open_port(port, baud=115200, timeout=0.1):
    """Opens a COM port or a socket://host:port address. DTR and RTS are kept low: on the
    ESP32 board the USB chip wires them to reset, so pyserial's normal open (which raises
    them) restarts the board and it then stays silent for several seconds."""
    import serial

    ser = serial.serial_for_url(port, baud, timeout=timeout, do_not_open=True)
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def computer_wifi_name():
    """The WiFi network this computer is on right now (Windows), or None."""
    if not sys.platform.startswith("win"):
        return None
    try:
        out = subprocess.run(["netsh", "wlan", "show", "interfaces"], capture_output=True, text=True,
                             timeout=4).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"^\s+SSID\s+:\s*(.+?)\s*$", out, re.M)
    return m.group(1) if m else None


def board_wifi_name():
    """The network the ESP32 firmware was built to join (src/wifi_config.h), or None."""
    try:
        text = (Path(__file__).parent / "src" / "wifi_config.h").read_text(encoding="utf-8")
    except OSError:
        return None
    m = re.search(r'^#define\s+WIFI_SSID\s+"([^"]*)"', text, re.M)
    return m.group(1) if m else None


def not_found_message():
    """Why no board was found, and what to do, naming the two networks when they differ."""
    mine, theirs = computer_wifi_name(), board_wifi_name()
    msg = "No board found on USB or on this computer's WiFi."
    if mine and theirs and mine != theirs:
        msg += (f" This computer is on \"{mine}\" but the ESP32 is set up for \"{theirs}\", "
                f"so a board powered without USB cannot be reached: join this computer to \"{theirs}\" "
                f"(its 2.4 GHz band), or plug the ESP32 into USB.")
    elif theirs:
        msg += (f" The ESP32 joins \"{theirs}\" by itself (2.4 GHz only) and this looks for it there; "
                f"check it is powered and that network is on.")
    else:
        msg += " Plug a board into USB, or check the ESP32 is on the same 2.4 GHz WiFi."
    return msg


NOT_FOUND = "No board found. Plug a board into USB, or check the ESP32 is on the same 2.4 GHz WiFi as this computer."  # short fallback


if __name__ == "__main__":
    print("networks on this computer:", [str(n) for n in local_networks()] or "none")
    print("USB board:", usb_port() or "none")
    t0 = time.time()
    host = find_wifi()
    print("ESP32 on WiFi:", f"{host}:{WIFI_PORT}" if host else "not found", f"({time.time() - t0:.1f} s)")
    print("would use:", find_port() or not_found_message())
