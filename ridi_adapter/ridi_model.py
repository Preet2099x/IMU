"""Reads RIDI's trained speed models (ridi_imu_models/svr_cascade0308) without OpenCV's
machine-learning module, which the OpenCV in this project does not have.

The models are OpenCV SVM files (YAML text, 200-350 MB each). This reads the numbers
straight from the text, caches them as compact .npz files next to the models (first
use only), and predicts with the same formula OpenCV uses for an epsilon-SVR with an
RBF kernel:

    speed = sum_i alpha_i * exp(-gamma * |x - sv_i|^2)  -  rho

RIDI regresses two numbers from one second of IMU data: the sideways speed (x) and the
forward/back speed (z) in a gravity-aligned frame, separately for four ways of carrying
a phone while walking: 0 handheld, 1 leg, 2 bag, 3 body.
"""
import re
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = ROOT / "ridi_imu-master" / "ridi_imu_models" / "svr_cascade0308"
CACHE_DIR = MODEL_DIR.with_name("svr_cascade0308_npz")  # git-ignored
CLASS_NAMES = {0: "handheld", 1: "leg", 2: "bag", 3: "body"}
CHANNELS = ("x", "z")  # regressor_<class>_0 is sideways speed, _1 is forward speed


def _numbers(text):
    """All numbers in a block of OpenCV YAML flow-sequence text."""
    text = text.replace("- [", " ").replace("[", " ").replace("]", " ").replace(",", " ")
    return np.fromstring(text, dtype=np.float64, sep=" ")


def convert_regressor(yaml_path, out_path):
    """YAML -> npz with the support vectors, their weights, rho and gamma."""
    text = Path(yaml_path).read_text()
    gamma = float(re.search(r"gamma:\s*([-+0-9.eE]+)", text).group(1))
    var_count = int(re.search(r"var_count:\s*(\d+)", text).group(1))
    sv_total = int(re.search(r"sv_total:\s*(\d+)", text).group(1))
    i0 = text.index("support_vectors:") + len("support_vectors:")
    i1 = text.index("decision_functions:")
    sv = _numbers(text[i0:i1])
    if sv.size != sv_total * var_count:
        raise ValueError(f"{yaml_path}: expected {sv_total} x {var_count} support-vector values, found {sv.size}")
    sv = sv.reshape(sv_total, var_count).astype(np.float32)
    tail = text[i1:]
    rho = float(re.search(r"rho:\s*([-+0-9.eE]+)", tail).group(1))
    a0 = tail.index("alpha:") + len("alpha:")
    alpha = _numbers(tail[a0:])
    if alpha.size != sv_total:
        raise ValueError(f"{yaml_path}: expected {sv_total} weights, found {alpha.size}")
    np.savez(out_path, sv=sv, alpha=alpha.astype(np.float32), rho=rho, gamma=gamma)


class SVR:
    def __init__(self, npz_path):
        d = np.load(npz_path)
        self.sv = d["sv"]
        self.alpha = d["alpha"].astype(np.float64)
        self.rho = float(d["rho"])
        self.gamma = float(d["gamma"])
        self.sv_sq = (self.sv.astype(np.float64) ** 2).sum(1)

    def predict(self, X):
        """X: (n, 1200) feature rows -> (n,) speeds."""
        X = np.atleast_2d(np.asarray(X, np.float64))
        d2 = (X ** 2).sum(1)[:, None] + self.sv_sq[None, :] - 2.0 * X @ self.sv.T.astype(np.float64)
        return np.exp(-self.gamma * np.maximum(d2, 0.0)) @ self.alpha - self.rho


def load_regressor(cls=0, channel=0, verbose=True):
    """The regressor for a carrying style and channel, converting it the first time."""
    CACHE_DIR.mkdir(exist_ok=True)
    npz = CACHE_DIR / f"regressor_{cls}_{channel}.npz"
    if not npz.exists():
        src = MODEL_DIR / f"regressor_{cls}_{channel}.yaml"
        if verbose:
            print(f"converting {src.name} ({src.stat().st_size // 2**20} MB) once ...", flush=True)
        t = time.time()
        convert_regressor(src, npz)
        if verbose:
            print(f"  done in {time.time() - t:.0f} s", flush=True)
    return SVR(npz)


if __name__ == "__main__":
    for c in (0,):
        for ch in (0, 1):
            m = load_regressor(c, ch)
            print(f"regressor {c}_{ch} ({CLASS_NAMES[c]}, {CHANNELS[ch]}): {m.sv.shape[0]} support vectors, "
                  f"gamma {m.gamma:.6f}, rho {m.rho:+.4f}")
            print("   prediction on an all-zero window:", m.predict(np.zeros(m.sv.shape[1]))[0])
