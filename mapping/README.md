# 3D IMU mapping

A live 3D view of the board's orientation. It starts flat and turns, tilts and
rolls as you move the real board.

## Run

Close any serial monitor first (only one program can hold the port), then:

```bash
python mapping/server.py
```

or double-click `mapping/run_viewer.bat`. It finds the Teensy by its USB ID,
opens the viewer in your browser at http://localhost:8765 and reconnects if you
re-flash or unplug the board. Needs `pyserial` (`pip install pyserial`).

| Option | Meaning |
|---|---|
| `--port COM7` | Use a specific port instead of auto-detecting |
| `--demo` | Synthetic motion, no board needed |
| `--skip-cal` | Skip the board's 30 s magnetometer calibration prompt automatically |
| `--http-port N` | Serve on another port (default 8765) |

If the board is waiting for a serial connection at power-up, connecting the
viewer starts its calibration: keep it still first, then rotate it if asked. The
last few lines the board prints show in the bottom-left corner.

## What you see

- **Board** with its own axes: X red (the orange nose), Y green, Z blue. These are
  the chip's axes, so the picture matches the numbers on serial.
- **Gimbal rings**: yaw (blue), pitch (green) and roll (red), nested in that
  order, so you can see which angle each movement changes.
- **Floor**: compass ring in degrees, a shadow of the board, and a needle showing
  where the nose points. The board's heading is the angle counter-clockwise from the start.
- **Nose trail** (orange) and **gyro arrow** (magenta, points along the rotation axis).

## Controls

| Key | |
|---|---|
| `Z` | Re-zero: the current pose becomes flat, heading 0 |
| `G` / `T` / `C` | Gimbal rings on/off, trail on/off, clear trail |
| `1` `2` `3` `4` | Iso, top, side, rear view (drag to orbit, scroll to zoom) |

**Start = flat** (default) treats the first reading as the board lying flat and
facing heading 0, even if it started upside down or turned. Angles are measured
from there. **Absolute** shows the angles exactly as the board reports them.

## Notes

- Positive pitch tips the nose *down* (right-hand rule about Y). Raise the front
  edge and pitch goes negative.
- Heading is only steady against north if the magnetometer is calibrated;
  otherwise it is relative and drifts slowly.
- It shows orientation only. Sliding the board across the desk won't move it:
  position from an IMU alone drifts by metres within seconds, so it isn't drawn.

## Tests

```bash
node mapping/web/test_orientation.mjs   # quaternion / reference maths
python mapping/test_bridge.py           # serial line parsing and event routing
```
