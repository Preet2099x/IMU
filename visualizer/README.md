# 3D IMU visualizer

The board moving through a 3D room. It turns as you turn it, and slides front,
back, left, right, up and down as you move it.

## Run

Close any serial monitor first (only one program can hold the port), then:

```bash
python visualizer/server.py
```

or double-click `visualizer/run_visualizer.bat`. It opens http://localhost:8766,
finds the Teensy by its USB ID and reconnects if you re-flash or unplug it.
Needs `pyserial` (`pip install pyserial`).

| Option | Meaning |
|---|---|
| `--demo` | A scripted route with no board |
| `--port COM7` | Use a specific port instead of auto-detecting |
| `--record FILE` | Also save what the board sends, to look at or replay later |
| `--replay FILE` | Play a saved session back |
| `--skip-cal` | Skip the board's startup magnetometer calibration |

The board must be running the firmware in this repo (`pio run -t upload`): it
adds a 50 Hz data stream that the viewer asks for. Plain serial monitors still
see the normal text output, and the board goes back to it by itself when the
viewer closes.

## First time: calibrate the accelerometer

Do this once. Press **Calibrate accelerometer**, then turn the board onto each of
its six sides in turn and hold it still for a couple of seconds. Each side ticks
off by itself. It takes about a minute.

Without it, the accelerometer's small errors (this board reads gravity about 1.5%
low) look like a steady push, which is what made the position creep. The result
is saved in `visualizer/accel_cal.json`, and the viewer sends it to the board every
time it connects, so the board's own tilt estimate benefits too.

## Using it

The board turns on screen as you turn it (roll, pitch, heading), and **rises and
falls in the room as you lift and lower it**. Only height is tracked. Sideways
position can't be followed from an accelerometer alone, so the board stays in the
middle.

1. Put the board down and keep it **still for a second** ("Hold still to start").
2. Lift it: the panel shows "Going up" and the height changes. Lower it: "Going down".
3. Stop and hold still: the height stays where it is.

Height is approximate (about 60-100% of the true distance in simulation, always the
right direction) and small errors add up over many moves. Press **Z** to bring it
back to the middle.

| Key | |
|---|---|
| `Z` | Zero height: back to the middle of the room |
| `H` | Zero heading: front becomes the way the nose points now |
| `T` / `C` | Trail on/off, clear trail |
| `1` `2` `3` `4` | Iso, top, side, rear view (drag to orbit, scroll to zoom) |

## How it works

Gravity gives a reliable "up", so the vertical acceleration is the board's
acceleration with gravity taken off. Adding it up once gives speed and twice gives
height, which drifts, so:

- **While still** the speed is set to zero and the sensor's resting offset is
  learned. A hand's tremor averages out, so a held board counts as still too.
- **During a move** the acceleration is added up from the moment it began (the
  first fraction of a second is caught up once the move is recognised).
- **When a move ends** the speed left over is the error that built up, so half of
  it times the duration is taken back off the height.

Left, right, front and back are deliberately not tracked; they drift too fast
(a 0.15 m/s² error is 7 m in ten seconds). A barometer would be the usual way to
make height solid.

## Tests

```bash
node visualizer/web/test_lift.mjs          # height tracking on simulated data
node visualizer/web/test_tracker.mjs       # the older full 3D tracker (no longer used by the page)
node visualizer/web/test_calibration.mjs   # accelerometer calibration maths
python visualizer/test_bridge.py           # stream parsing, calibration storage, demo route
```

## 3D tracking on the ESP32 (track3d.py)

The ESP32 firmware (`src/espmain.cpp`, tracking in `src/tracker3d.h`) tracks the
board's position itself, at 400 Hz. `track3d.py` draws the path it reports:

```bash
python visualizer/track3d.py                    # finds the board itself (WiFi first, then USB)
python visualizer/track3d.py --port COM8        # over USB
python visualizer/track3d.py --replay FILE.log  # a saved session again
```

Keep the board still for a second at the start, then move it and stop. The
accelerometer is only used for "which way is down" while the board is still, so
short moves with a pause at the end track best; long continuous waving still
drifts (see `analysis/report.html` for why). Every session is saved to
`recordings/track_<time>.log` with every raw sample, and
`python visualizer/tracker3d.py FILE.log` runs it through a Python copy of the
tracker so settings can be tried on real recordings.

Keys: `Z` position to zero, `Y` current heading becomes forward, `C` clear path.

### track2.py: smooth, a few seconds late

```bash
python visualizer/track2.py            # shows the board where it was 4.5 s ago
python visualizer/track2.py --delay 3
```

`track3d.py` shows the board's position in real time, so at every stop it jumps
(on real recordings by 1-2 cm, sometimes about a metre) when the board takes back the
drift of the move. `track2.py` holds the picture back a few seconds, waits for the
stop, and bends the whole move smoothly instead: on the three latest recordings the
biggest jump at a stop fell from 67-119 cm to 1.5-2.5 cm, and it ends where the
board ends. It draws no path line (`T` switches a thin one on). A move that is still
going after the delay is shown uncorrected.

## Finding the board

None of the viewers or loggers need an address typed in. `boardfind.py` (project
root) looks for the board on USB (Teensy or ESP32) and for the ESP32 on whatever
WiFi network this computer is on: at the address it had last time, then `imu.local`,
then a quick scan of the local network. `python boardfind.py` shows what it found.

The ESP32 only joins **2.4 GHz** networks. Its network name and password are in
`src/wifi_config.h` (git-ignored; copy `src/wifi_config.example.h`). After a change
there, flash with `pio run -e esp32 -t upload --upload-port COM8` over USB (or
`pio run -e esp32_ota -t upload` over WiFi once it is on the network).

