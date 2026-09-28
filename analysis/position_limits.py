"""Measures, from the real recordings in visualizer/recordings, why position cannot be
recovered from this IMU alone, and draws the figures for the report.

    python analysis/position_limits.py

Writes analysis/figures/*.png and analysis/results.json. Everything here comes from
the board's own logs (saved with visualizer/server.py --record or orient3d.py); there
is no simulated data.
"""
import glob
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "visualizer"))
import orient3d as o  # noqa: E402
from bridge import CalStore, parse_viz  # noqa: E402

G = 9.80665
FIG = HERE / "figures"
FIG.mkdir(exist_ok=True)
cal = CalStore(ROOT / "visualizer" / "accel_cal.json").get()
OFF, SCALE = np.array(cal["offset"]), np.array(cal["scale"])


def load(path):
    rows = []
    for line in open(path):
        ev = parse_viz(line)
        if ev:
            rows.append(ev)
    ms = np.array([e["ms"] for e in rows]) / 1000.0
    q = np.array([o.qnorm(np.array(e["q"], float)) for e in rows])
    f = (np.array([e["f"] for e in rows]) - OFF) * SCALE
    g = np.array([e["g"] for e in rows])
    return ms, q, f, g


def split_at_restarts(name, data):
    """The board's clock restarts from zero when it reboots; treat each run separately."""
    ms = data[0]
    cuts = [0, *(np.where(np.diff(ms) < 0)[0] + 1), len(ms)]
    out = {}
    for k in range(len(cuts) - 1):
        sl = slice(cuts[k], cuts[k + 1])
        out[name if len(cuts) == 2 else f"{name}#{k + 1}"] = tuple(x[sl] for x in data)
    return out


LOGS = {}
for p_ in sorted(glob.glob(str(ROOT / "visualizer" / "recordings" / "*.viz"))):
    LOGS.update(split_at_restarts(Path(p_).name, load(p_)))
LOGS = {k: v for k, v in LOGS.items() if len(v[0]) > 500}


def earth_accel(q, f):
    """Acceleration in earth axes with gravity taken off, from the firmware orientation."""
    R = np.array([o.quat_to_mat(x) for x in q])
    return np.einsum("nij,nj->ni", R, f * G) - np.array([0, 0, G]), R


# ---------------------------------------------------------------- still windows
WIN_S = 10.0
still = []  # (log, i0, i1)
for name, (ms, q, f, g) in LOGS.items():
    fm = np.linalg.norm(f, axis=1)
    gn = np.linalg.norm(g, axis=1)
    i = 0
    while i < len(ms):
        j = np.searchsorted(ms, ms[i] + WIN_S)
        if j >= len(ms):
            break
        if gn[i:j].max() < 3.0 and fm[i:j].std() < 0.01:
            still.append((name, i, j))
            i = j
        else:
            i += 25
res = {"windows": len(still)}

noise, eps, gbias, drift_raw, drift_bias, tilt_deg = [], [], [], [], [], []
curves = []
for name, i0, i1 in still:
    ms, q, f, g = LOGS[name]
    a, R = earth_accel(q[i0:i1], f[i0:i1])
    t = ms[i0:i1] - ms[i0]
    fb = f[i0:i1] * G
    noise.append(np.sqrt(((fb - fb.mean(0)) ** 2).sum(1).mean()))
    hz = a.mean(0)[:2]  # mean horizontal error: what would show up as sideways acceleration
    eps.append(np.linalg.norm(hz))
    tilt_deg.append(np.degrees(np.arcsin(min(1, np.linalg.norm(hz) / G))))
    gbias.append(np.linalg.norm(g[i0:i1].mean(0)))
    dt = np.gradient(t)
    v = np.cumsum(a[:, :2] * dt[:, None], axis=0)
    p = np.cumsum(v * dt[:, None], axis=0)
    drift_raw.append(np.linalg.norm(p[-1]))
    curves.append((t, np.linalg.norm(p, axis=1)))
    # if the resting error is measured over the first 2 s and removed
    k = np.searchsorted(t, 2.0)
    a2 = a[:, :2] - a[:k, :2].mean(0)
    v2 = np.cumsum(a2[k:] * dt[k:, None], axis=0)
    p2 = np.cumsum(v2 * dt[k:, None], axis=0)
    drift_bias.append(np.linalg.norm(p2[-1]))

pct = lambda x, p: float(np.percentile(x, p))  # noqa: E731
res["accel_noise_rms_mps2"] = pct(noise, 50)
res["resting_horizontal_accel_error_mps2"] = {"median": pct(eps, 50), "p90": pct(eps, 90), "max": float(max(eps))}
res["resting_tilt_equivalent_deg"] = {"median": pct(tilt_deg, 50), "p90": pct(tilt_deg, 90)}
res["gyro_residual_dps"] = {"median": pct(gbias, 50), "p90": pct(gbias, 90)}
res["still_10s_drift_m"] = {"no_correction_median": pct(drift_raw, 50), "no_correction_p90": pct(drift_raw, 90),
                            "no_correction_max": float(max(drift_raw)),
                            "bias_removed_median": pct(drift_bias, 50), "bias_removed_p90": pct(drift_bias, 90)}

# ---------------------------------------------------------------- analytic growth
t = np.logspace(-1, 1.7, 200)
eps_meas = res["resting_horizontal_accel_error_mps2"]["median"]
gb = np.radians(max(res["gyro_residual_dps"]["median"], 1e-3))
sig = res["accel_noise_rms_mps2"] / np.sqrt(3)  # per axis
dt_s = 0.02
fig, ax = plt.subplots(figsize=(8.5, 5.2))
curves_a = {
    f"measured resting accel error ({eps_meas:.3f} m/s²)": 0.5 * eps_meas * t ** 2,
    "tilt error 0.5°": 0.5 * G * np.radians(0.5) * t ** 2,
    "tilt error 1°": 0.5 * G * np.radians(1.0) * t ** 2,
    "tilt error 3° (during a move)": 0.5 * G * np.radians(3.0) * t ** 2,
    f"gyro bias {np.degrees(gb):.2f}°/s left uncorrected (t³)": G * gb * t ** 3 / 6,
    "accel noise only (random walk)": sig * np.sqrt(dt_s) * t ** 1.5 / np.sqrt(3),
}
for label, y in curves_a.items():
    ax.loglog(t, y, label=label, lw=1.8)
for yv, lab in ((0.01, "1 cm"), (0.1, "10 cm"), (1, "1 m")):
    ax.axhline(yv, color="0.6", ls=":", lw=1)
    ax.text(t[-1], yv, lab, ha="right", va="bottom", color="0.4", fontsize=8)
ax.set_xlabel("time since the last known-good position (s)")
ax.set_ylabel("position error (m)")
ax.set_title("Position error from integrating an accelerometer twice")
ax.legend(fontsize=8, loc="upper left")
ax.grid(alpha=0.3, which="both")
fig.tight_layout()
fig.savefig(FIG / "growth.png", dpi=130)
plt.close(fig)
res["growth_at_seconds"] = {
    s_: {k: float(np.interp(s_, t, y)) for k, y in curves_a.items()} for s_ in (1, 2, 5, 10, 30)
}

# ---------------------------------------------------------------- measured drift of a still board
fig, ax = plt.subplots(figsize=(8.5, 4.6))
for tt, pp in curves[:60]:
    ax.plot(tt, pp, color="tab:red", alpha=0.25, lw=1)
ts = np.linspace(0, WIN_S, 100)
ax.plot(ts, 0.5 * eps_meas * ts ** 2, color="k", lw=2, label="½ · median error · t²")
ax.set_xlabel("seconds (board resting on a table the whole time)")
ax.set_ylabel("apparent sideways position (m)")
ax.set_title(f"A board that never moved: double integration over {len(curves)} real 10 s rests")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(FIG / "still_drift.png", dpi=130)
plt.close(fig)

# ---------------------------------------------------------------- a slide there and back, raw
name = "left_and_back.viz"
if name in LOGS:
    ms, q, f, g = LOGS[name]
    end = np.searchsorted(ms - ms[0], 25.0)
    tr = o.Slide()
    tr.correct = False
    ref = q[0]
    ys, xs, vs, mv, tt = [], [], [], [], []
    for i in range(end):
        tr.update(ms[i] * 1000, q[i], f[i], g[i], np.radians(o.quat_to_euler(ref)[2]))
        xs.append(tr.pos[0]); ys.append(tr.pos[1]); vs.append(tr.vel[1]); mv.append(tr.moving); tt.append(ms[i] - ms[0])
    tt = np.array(tt)
    fig, axs = plt.subplots(3, 1, figsize=(8.5, 7), sharex=True)
    axs[0].plot(tt, np.linalg.norm(g[:end], axis=1), color="tab:red", lw=0.8)
    axs[0].set_ylabel("rotation rate (°/s)")
    axs[0].set_title("Slide left, back to the start, hop left, hop back to the start (real recording)")
    axs[1].plot(tt, vs, color="tab:blue")
    axs[1].set_ylabel("speed, left/right (m/s)")
    axs[2].plot(tt, ys, color="tab:green", label="tracked left/right position")
    axs[2].plot(tt, xs, color="tab:orange", label="tracked front/back position")
    axs[2].axhline(0, color="k", lw=1)
    axs[2].axhline(0, color="k", lw=0)
    axs[2].set_ylabel("position (m)")
    axs[2].set_xlabel("seconds")
    axs[2].legend(fontsize=8)
    for a_ in axs:
        a_.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG / "slide_and_back.png", dpi=130)
    plt.close(fig)
    res["slide_and_back"] = {"seconds": 25.0, "true_final_position_m": 0.0,
                             "tracked_final_m": [float(xs[-1]), float(ys[-1])],
                             "tracked_max_excursion_m": float(np.abs(np.array([xs, ys])).max())}

# ---------------------------------------------------------------- trackers on every real log
table = []
for name, (ms, q, f, g) in LOGS.items():
    tr = o.Tracks()
    ref = q[0]
    ps, pk = [], []
    for i in range(len(ms)):
        tr.update(ms[i] * 1000, q[i], f[i], g[i], np.radians(o.quat_to_euler(ref)[2]))
        ps.append(tr.simple.pos.copy()); pk.append(tr.kf.out.copy())
    ps, pk = np.array(ps), np.array(pk)
    table.append({"log": name, "seconds": float(ms[-1] - ms[0]),
                  "simple_max_m": float(np.linalg.norm(ps, axis=1).max()), "simple_final_m": float(np.linalg.norm(ps[-1])),
                  "kalman_max_m": float(np.linalg.norm(pk, axis=1).max()), "kalman_final_m": float(np.linalg.norm(pk[-1]))})
res["trackers_on_real_logs"] = table

# ---------------------------------------------------------------- how wrong is gravity removal while moving
mag_rest, mag_move = [], []
for name, (ms, q, f, g) in LOGS.items():
    fm = np.linalg.norm(f, axis=1)
    gn = np.linalg.norm(g, axis=1)
    rest = gn < 2.0
    move = gn > 20.0
    mag_rest.append(np.abs(fm[rest] - 1.0))
    mag_move.append(np.abs(fm[move] - 1.0))
mr, mm = np.concatenate(mag_rest), np.concatenate(mag_move)
res["specific_force_deviation_from_1g"] = {"at_rest_median_g": float(np.median(mr)), "at_rest_p95_g": float(np.percentile(mr, 95)),
                                            "rotating_over_20dps_median_g": float(np.median(mm)), "rotating_over_20dps_p95_g": float(np.percentile(mm, 95))}
fig, ax = plt.subplots(figsize=(8.5, 4.4))
bins = np.linspace(0, 0.6, 61)
ax.hist(mr, bins=bins, alpha=0.7, density=True, label="resting", color="tab:blue")
ax.hist(mm, bins=bins, alpha=0.6, density=True, label="being moved (rotating > 20°/s)", color="tab:red")
ax.set_xlabel("| |accelerometer| − 1 g |  (g)")
ax.set_ylabel("density")
ax.set_title("What the accelerometer reads: gravity alone at rest, gravity plus motion otherwise")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(FIG / "specific_force.png", dpi=130)
plt.close(fig)

json.dump(res, open(HERE / "results.json", "w"), indent=2)
print(json.dumps(res, indent=2))
