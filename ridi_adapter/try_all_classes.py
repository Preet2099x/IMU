"""Runs all four of RIDI's carrying styles (handheld, leg, bag, body) over one recording and prints
how each one's horizontal speed compares with reality while the board is still and while it moves.

    python ridi_adapter/try_all_classes.py visualizer/recordings/track_20260924_162050.log
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import ridi_model as rm  # noqa: E402
from try_ridi import predict_world_velocity  # noqa: E402
from ridi_data import load_track_log  # noqa: E402

d = load_track_log(sys.argv[1])
print(f"{Path(sys.argv[1]).name}: {d['t'][-1]:.0f} s")
first = True
for cls in (0, 1, 2, 3):
    ends, xz, vw = predict_world_velocity(d, cls)
    moving = d["state"][ends - 1] == 2
    sp = np.linalg.norm(vw[:, :2], axis=1)
    ref = np.linalg.norm(d["v_ref"][ends - 1][:, :2], axis=1)
    m = moving & (ref > 0.05)
    corr = np.corrcoef(sp[m], ref[m])[0, 1] if m.sum() > 10 else float("nan")
    print(f"  {rm.CLASS_NAMES[cls]:9s} still: median {np.median(sp[~moving]):.2f} m/s | moving: median {np.median(sp[moving]):.2f}, "
          f"90th pct {np.percentile(sp[moving], 90):.2f} | correlation with real speed {corr:+.2f}", flush=True)
