# RIDI on our board (ridi_adapter)

Uses the trained models in `ridi_imu-master/ridi_imu_models/svr_cascade0308` with our ESP32 recordings.
The models are RIDI's speed regressors: from one second of gravity-aligned gyro and linear
acceleration (200 Hz) they predict the board's sideways and forward speed.

| File | What it does |
|---|---|
| `ridi_model.py` | Reads the OpenCV SVM YAML directly (this project's OpenCV has no `ml` module), converts each model once to `.npz` (git-ignored), predicts with the SVR formula `sum(alpha*exp(-gamma*|x-sv|^2)) - rho`. |
| `ridi_features.py` | RIDI's feature pipeline: gravity alignment, Gaussian smoothing, 200-frame windows, and turning a predicted local speed into a room-frame velocity. |
| `ridi_data.py` | Loads a `visualizer/track3d.py` recording (raw 400 Hz A/G lines) as 200 Hz phone-style streams. |
| `try_ridi.py`, `try_all_classes.py` | Run the models on a recording and compare with our own tracker. |
| `test_ridi.py` | Checks the axis conventions (a board lying flat pointing forward is forward = -z, as RIDI defines it). |

## Result (2026-09-28, real recording track_20260924_162050, 160 s)

RIDI's pretrained models do not carry over to hand movements of the board. They were trained on phones
carried by people walking, so any activity is read as "walking forward at walking speed":

| Carrying style | Board still (median speed) | Board moving (median speed) | Correlation with real speed |
|---|---|---|---|
| handheld | 0.16 m/s | 1.04 m/s | -0.15 |
| leg | 0.05 | 0.47 | -0.03 |
| bag | 0.04 | 1.15 | +0.42 |
| body | 0.06 | 0.52 | -0.03 |

Real speed on the same moves is 0.02-0.3 m/s. A 20 cm slide lasting one second would come out as over a
metre forward. RIDI's other step (correcting a constant accelerometer bias) has nothing to fix here: on calm
moves a constant bias explains only 5-6% of the leftover speed at each stop.

## What would make RIDI work here

The model has to be trained on this board's own movements, and that needs true positions (ground truth) while
moving, for example a webcam tracking a marker taped to the board. Nothing else in the project provides that.
