"""Turns our board's recordings into what RIDI's speed model expects, and back.

RIDI's model looks at one second of the IMU (200 frames at 200 Hz):
  * the gyro and the linear acceleration (acceleration with gravity taken off), both
    turned into a "stabilized" frame in which gravity points along +y, so only the
    board's heading matters, not its tilt;
  * lightly smoothed (a Gaussian, sigma 2 frames) inside the window;
  * laid out frame after frame: [gx gy gz ax ay az] x 200 = 1200 numbers.
It answers with the board's speed in that frame: x sideways, z forward/back (forward is -z).

Our board's axes line up with a phone lying flat, screen up: x to the right, y toward
the nose (the phone's top), z up. So a board lying flat and pointing "forward" is the
same picture RIDI trained on, and no axis swapping is needed.

Following the RIDI code (python/training_data.py, python/geometry.py).
"""
import numpy as np

WINDOW = 200          # frames per feature window (1 s at 200 Hz)
RATE_HZ = 200.0
SIGMA = 2.0           # frames
STEP = 30             # frames between predictions in RIDI's localization (0.15 s)
LOCAL_UP = np.array([0.0, 1.0, 0.0])  # where gravity is turned to in the stabilized frame


def quat_rotate(q, v):
    """Rotate vectors v (n,3) by quaternions q (n,4) [w,x,y,z]: q v q*."""
    u = q[:, 1:]
    t = 2.0 * np.cross(u, v)
    return v + q[:, :1] * t + np.cross(u, t)


def quat_from_two_vectors(v1, v2):
    """(n,4) quaternions turning direction v1 (n,3) into direction v2 (3,). As in RIDI's geometry.py."""
    v1n = v1 / np.linalg.norm(v1, axis=1, keepdims=True)
    v2n = v2 / np.linalg.norm(v2)
    w = np.cross(v1n, v2n)
    q = np.concatenate([(1.0 + v1n @ v2n)[:, None], w], axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):  # exactly opposite vectors: caller replaces those rows
        return q / np.linalg.norm(q, axis=1, keepdims=True)


def align_with_gravity(data, gravity, local_up=LOCAL_UP, eps=1e-3):
    """RIDI's align_3dvector_with_gravity: remove tilt by turning gravity onto local_up."""
    gn = gravity / np.linalg.norm(gravity, axis=1, keepdims=True)
    q = quat_from_two_vectors(gravity, local_up)
    out = quat_rotate(q, data)
    gd = gn @ local_up
    out[gd > 1.0 - eps] = data[gd > 1.0 - eps]                      # already aligned
    flip = gd < -1.0 + eps                                          # upside down: flip y and z
    out[flip] = data[flip] * np.array([1.0, -1.0, -1.0])
    return out


def gaussian_smooth(x, sigma=SIGMA, truncate=4.0):
    """Gaussian smoothing along axis 0 with mirrored borders (scipy's gaussian_filter1d behaviour)."""
    r = int(truncate * sigma + 0.5)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    p = np.pad(x, ((r, r), (0, 0)), mode="symmetric")
    return np.stack([np.convolve(p[:, c], k, mode="valid") for c in range(x.shape[1])], axis=1)


def features_at(gyro_g, lin_g, ends):
    """Feature rows for windows ending just before each index in `ends`. gyro_g/lin_g are the
    gravity-aligned streams (n,3). One row is 200 frames x [gyro xyz, linacc xyz], smoothed per window."""
    both = np.concatenate([gyro_g, lin_g], axis=1)
    rows = np.empty((len(ends), WINDOW * 6))
    for k, e in enumerate(ends):
        rows[k] = gaussian_smooth(both[e - WINDOW:e]).reshape(-1)
    return rows


def local_speed_to_world(local_xz, gravity, orientation):
    """A predicted (x, z) speed in the stabilized frame -> velocity in the room frame:
    v_world = R_device * Rg^T * [x, 0, z], where Rg turns the device's gravity onto +y."""
    n = len(local_xz)
    v = np.zeros((n, 3))
    v[:, 0] = local_xz[:, 0]
    v[:, 2] = local_xz[:, 1]
    q_g = quat_from_two_vectors(gravity, LOCAL_UP)
    q_g_inv = q_g * np.array([1, -1, -1, -1])
    dev = quat_rotate(q_g_inv, v)                       # back to the device frame
    return quat_rotate(orientation, dev)                # device -> room
