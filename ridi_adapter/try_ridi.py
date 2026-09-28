"""First look: what does RIDI's trained "handheld" speed model say about our board's real recordings?

    python ridi_adapter/try_ridi.py visualizer/recordings/track_20260924_162050.log

Runs the model every 0.15 s on one-second windows, and compares its speed with the speed our own
tracker measured over the same moves (which is reliable on short, calm moves). Writes a picture.
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import ridi_features as rf  # noqa: E402
import ridi_model as rm  # noqa: E402
from ridi_data import load_track_log  # noqa: E402


def predict_world_velocity(d, cls=0):
    """RIDI's handheld model on a 200 Hz stream -> (times, world velocity) every 0.15 s."""
    gyro_g = rf.align_with_gravity(d["gyro"], d["grav"])
    lin_g = rf.align_with_gravity(d["lin"], d["grav"])
    ends = np.arange(rf.WINDOW, len(d["t"]), rf.STEP)
    X = rf.features_at(gyro_g, lin_g, ends)
    reg = [rm.load_regressor(cls, ch) for ch in (0, 1)]
    xz = np.stack([r.predict(X) for r in reg], axis=1)
    v_world = rf.local_speed_to_world(xz, d["grav"][ends - 1], d["q"][ends - 1])
    return ends, xz, v_world


def main(path, out_png):
    d = load_track_log(path)
    print(f"{Path(path).name}: {d['t'][-1]:.0f} s of tracked data at 200 Hz ({len(d['t'])} frames)")
    ends, xz, vw = predict_world_velocity(d)
    t = d["t"][ends - 1]
    moving = d["state"][ends - 1] == 2
    v_ref = d["v_ref"][ends - 1]
    sp_ridi = np.linalg.norm(vw[:, :2], axis=1)
    sp_ref = np.linalg.norm(v_ref[:, :2], axis=1)
    print(f"windows: {len(ends)}  (moving according to our tracker: {moving.sum()})")
    for name, m in (("still", ~moving), ("moving", moving)):
        if m.sum():
            print(f"  our board {name:6s}: RIDI horizontal speed  median {np.median(sp_ridi[m]):.2f}  "
                  f"90th pct {np.percentile(sp_ridi[m], 90):.2f} m/s   |  ours median {np.median(sp_ref[m]):.2f}")
    m = moving & (sp_ref > 0.05)
    if m.sum() > 10:
        c = np.corrcoef(sp_ridi[m], sp_ref[m])[0, 1]
        print(f"  correlation of RIDI's speed with ours while moving: {c:+.2f} over {m.sum()} windows")
    fig, ax = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    ax[0].plot(t, sp_ref, label="our tracker", color="k")
    ax[0].plot(t, sp_ridi, label="RIDI handheld model", color="tab:red")
    ax[0].set_ylabel("horizontal speed (m/s)")
    ax[0].legend()
    ax[1].plot(t, xz[:, 0], label="sideways x")
    ax[1].plot(t, xz[:, 1], label="forward/back z (forward is negative)")
    ax[1].set_ylabel("RIDI output (m/s)")
    ax[1].legend()
    ax[2].plot(d["t"], np.linalg.norm(d["gyro"], axis=1) * 57.3, lw=0.6, color="tab:green")
    ax[2].set_ylabel("|gyro| deg/s")
    ax[2].set_xlabel("seconds")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    print("saved", out_png)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "ridi_first_look.png")
