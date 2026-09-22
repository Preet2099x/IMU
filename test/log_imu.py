"""Logs the IMU unattended: 13 back-to-back runs of one hour, one CSV per run.

Columns: pc_time, board_ms, heading, pitch, roll, accel x/y/z (g), gyro x/y/z
(deg/s), mag x/y/z (counts, calibrated if the board ran its mag calibration).
20 samples per second. Each file is flushed as it goes, so a crash or Ctrl+C keeps
everything recorded so far. When a run finishes its JSON copy and a summary are
written, and the next run starts straight away with no gap (the board is never
restarted between runs, so the mag calibration carries over). If the USB cable
drops, it reconnects by itself and carries on.

    python test/log_imu.py                       # 13 runs of 60 min, auto-detects the Teensy
    python test/log_imu.py --runs 2 --minutes 1 --prefix test/trial  # a quick trial
    python test/log_imu.py --port COM7

Files: test/imu_1hr_3.csv, imu_1hr_4.csv, ... imu_1hr_15.csv (numbering starts at --start).
An existing file is never overwritten: that number is skipped.

Close any serial monitor or the visualizer first: only one program can hold the port.
Needs pyserial (pip install pyserial) and the firmware in this repo (it adds the
"l" log mode).
"""
import argparse
import csv
import json
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

TEENSY_VID_PID = (0x16C0, 0x0483)
COLUMNS = ["pc_time", "board_ms", "heading", "pitch", "roll",
           "acc_x_g", "acc_y_g", "acc_z_g", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
           "mag_x", "mag_y", "mag_z"]


def find_port():
    for p in list_ports.comports():
        if (p.vid, p.pid) == TEENSY_VID_PID:
            return p.device
    return None


def parse_log(line):
    """LOG,ms,heading,pitch,roll,ax,ay,az,gx,gy,gz,mx,my,mz -> list of 13 numbers, or None."""
    parts = line.strip().split(",")
    if len(parts) != 14 or parts[0] != "LOG":
        return None
    try:
        return [float(v) for v in parts[1:]]
    except ValueError:
        return None


def open_port(port_arg):
    """Waits until the board can be opened; returns the port."""
    warned = False
    while True:
        port = port_arg or find_port()
        if port:
            try:
                return serial.Serial(port, 115200, timeout=0.2)
            except serial.SerialException as e:
                if not warned:
                    print(f"Could not open {port}: {e}\nClose any serial monitor or the visualizer. Retrying...")
                    warned = True
        elif not warned:
            print("No Teensy found (USB 16C0:0483). Plug it in. Waiting...")
            warned = True
        time.sleep(2)


def summarize(rows, out):
    out.with_suffix(".json").write_text(json.dumps([dict(zip(COLUMNS, r)) for r in rows]))
    dur = (rows[-1][1] - rows[0][1]) / 1000
    print(f"\n{out.name}: {len(rows)} samples over {dur / 60:.1f} min ({len(rows) / max(dur, 1e-9):.1f} per second).")
    for i, name in enumerate(COLUMNS[2:], start=2):
        col = [r[i] for r in rows]
        print(f"  {name:<11} min {min(col):9.2f}  max {max(col):9.2f}  mean {statistics.fmean(col):9.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--runs", type=int, default=13)
    ap.add_argument("--minutes", type=float, default=60.0, help="length of each run")
    ap.add_argument("--start", type=int, default=3, help="number of the first file")
    ap.add_argument("--prefix", default="imu_1hr_", help="file name start (in test/ unless it has a folder)")
    args = ap.parse_args()

    prefix = Path(args.prefix)
    if prefix.parent == Path("."):
        prefix = Path(__file__).with_name(prefix.name)
    run_len = args.minutes * 60

    # Numbers that already have a file are skipped, so nothing recorded is overwritten.
    names, n = [], args.start
    while len(names) < args.runs:
        f = prefix.with_name(f"{prefix.name}{n}.csv")
        if not f.exists():
            names.append(f)
        n += 1

    def path_for(run):
        return names[run - 1]

    print(f"{args.runs} runs of {args.minutes:g} min. Files: {path_for(1).name} ... {path_for(args.runs).name}")
    print("Ctrl+C stops early and keeps everything recorded so far.")
    ser = open_port(args.port)
    running = False        # only talk to the board once it is past its start-up prompts
    last_keepalive = last_data = last_report = 0.0
    run = 0                # 0 until the first data arrives
    run_start = 0.0
    rows, fh, writer, out = [], None, None, None
    buf = b""

    def begin_run(now):
        nonlocal run, run_start, rows, fh, writer, out
        run += 1
        run_start = now
        rows = []
        out = path_for(run)
        fh = out.open("w", newline="")
        writer = csv.writer(fh)
        writer.writerow(COLUMNS)
        print(f"--- run {run} of {args.runs} started, writing {out.name}")

    def end_run():
        nonlocal fh
        if fh:
            fh.close()
            fh = None
            if rows:
                summarize(rows, out)

    try:
        while True:
            now = time.time()
            try:
                if running and now - last_keepalive >= 1.0:
                    ser.write(b"l")  # "keep sending the log" (the board stops after 3 s of silence)
                    last_keepalive = now
                buf += ser.read(4096)
            except (serial.SerialException, OSError):
                print("Connection lost (unplugged or the board reset). Reconnecting...")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = open_port(args.port)
                running, buf, last_data = False, b"", time.time()
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                vals = parse_log(line)
                if vals:
                    last_data = now
                    if run == 0:
                        begin_run(now)
                    elif now - run_start >= run_len:
                        end_run()
                        if run >= args.runs:
                            raise StopIteration
                        begin_run(now)
                    row = [datetime.now().isoformat(timespec="milliseconds")] + vals
                    rows.append(row)
                    writer.writerow(row)
                else:
                    if line.startswith("Heading:") or line == "=== running ===":
                        running = True  # normal running output: safe to ask for the log now
                    print("board:", line)
            if fh and len(rows) % 100 == 0:
                fh.flush()
            if run and now - last_report >= 60 and rows:
                last_report = now
                r = rows[-1]
                left = run_len - (now - run_start)
                print(f"run {run}/{args.runs} | {len(rows):>6} samples | heading {r[2]:6.1f} pitch {r[3]:6.1f} roll {r[4]:6.1f} | {left / 60:5.1f} min left in this run")
            if run and now - last_data > 5 and running:
                print("WARNING: no data for 5 s, still waiting.")
                last_data = now
    except StopIteration:
        pass
    except KeyboardInterrupt:
        print("Stopped early.")
        end_run()
    finally:
        try:
            ser.write(b"h")
            ser.close()
        except Exception:
            pass
    if not run:
        sys.exit("Nothing was logged. Is the board running the firmware from this repo?")
    print(f"\nFinished: {run} run(s) saved next to this script.")


if __name__ == "__main__":
    main()
