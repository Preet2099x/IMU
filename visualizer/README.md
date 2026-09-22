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
