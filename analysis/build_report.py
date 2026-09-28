"""Builds analysis/report.html (self-contained) from analysis/results.json and figures/.

    python analysis/position_limits.py     # measure, draw figures
    python analysis/build_report.py        # write the report
"""
import base64
import json
from pathlib import Path

HERE = Path(__file__).parent
R = json.load(open(HERE / "results.json"))


def img(name):
    return "data:image/png;base64," + base64.b64encode((HERE / "figures" / name).read_bytes()).decode()


def cm(x):
    return f"{x * 100:.0f} cm" if x < 1 else f"{x:.1f} m"


g = R["growth_at_seconds"]
keys = list(g["1"].keys())
k_rest, k05, k1, k3, k_gyro, k_noise = keys
rows = ""
for label, k in (("Measured resting accelerometer error", k_rest), ("Tilt error 0.5°", k05), ("Tilt error 1°", k1),
                 ("Tilt error 3° (a hand move)", k3), ("Gyro bias, tilt never corrected (t³)", k_gyro),
                 ("Accelerometer noise alone", k_noise)):
    rows += "<tr><th scope='row'>" + label + "</th>" + "".join(f"<td>{cm(g[s][k])}</td>" for s in ("1", "2", "5", "10", "30")) + "</tr>"

trk = ""
for t in R["trackers_on_real_logs"]:
    trk += (f"<tr><th scope='row'>{t['log']}</th><td>{t['seconds']:.0f} s</td>"
            f"<td>{t['simple_max_m']:.1f} m</td><td>{t['simple_final_m']:.1f} m</td>"
            f"<td>{t['kalman_max_m']:.1f} m</td><td>{t['kalman_final_m']:.1f} m</td></tr>")

sb = R["slide_and_back"]
close = (sb["tracked_final_m"][0] ** 2 + sb["tracked_final_m"][1] ** 2) ** 0.5
still = R["still_10s_drift_m"]
eps = R["resting_horizontal_accel_error_mps2"]
tilt = R["resting_tilt_equivalent_deg"]
gy = R["gyro_residual_dps"]
sf = R["specific_force_deviation_from_1g"]

html = open(HERE / "report_template.html", encoding="utf-8").read()
for key, val in {
    "GROWTH_ROWS": rows, "TRACKER_ROWS": trk,
    "IMG_GROWTH": img("growth.png"), "IMG_STILL": img("still_drift.png"),
    "IMG_SLIDE": img("slide_and_back.png"), "IMG_FORCE": img("specific_force.png"),
    "NOISE": f"{R['accel_noise_rms_mps2']:.3f}", "NWIN": str(R["windows"]),
    "EPS_MED": f"{eps['median']:.4f}", "EPS_P90": f"{eps['p90']:.4f}",
    "TILT_MED": f"{tilt['median']:.3f}", "TILT_P90": f"{tilt['p90']:.3f}",
    "GYRO_MED": f"{gy['median']:.3f}", "GYRO_P90": f"{gy['p90']:.3f}",
    "STILL_MED": f"{still['no_correction_median'] * 100:.0f}", "STILL_P90": f"{still['no_correction_p90'] * 100:.0f}",
    "STILL_MAX": f"{still['no_correction_max']:.1f}",
    "CLOSE": f"{close:.2f}", "FINAL_X": f"{sb['tracked_final_m'][0]:+.2f}", "FINAL_Y": f"{sb['tracked_final_m'][1]:+.2f}",
    "SF_REST": f"{sf['at_rest_median_g']:.3f}", "SF_MOVE": f"{sf['rotating_over_20dps_median_g']:.3f}",
    "SF_MOVE95": f"{sf['rotating_over_20dps_p95_g']:.2f}",
}.items():
    html = html.replace("@@" + key + "@@", val)
(HERE / "report.html").write_text(html, encoding="utf-8")
print("wrote", HERE / "report.html", len(html) // 1024, "KB")
