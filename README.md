# 🛣️  Lane Detection & PID Steering

Real‑time **lane detection** using OpenCV + **PID control** for smooth steering‑angle estimation.  
Tested on dash‑cam sequences and live USB cameras.

<p align="center">
  <img src="docs/media/demo_pipeline.gif" alt="pipeline demo" width="650"/>
</p>

---

## 📂 Repository Layout

```text
.
├── main.py                # entry‑point: iterates images / webcam, displays angle
├── OpencvLaneDetect.py    # end‑to‑end lane‑detect + steering‑angle class
├── PIDSteering.py         # generic PID implementation with EMA + rate limiter
├── get_calibration.py     # chessboard camera‑calibration script → calib_params.json
├── calibration.py         # loads matrix & wraps homography for BEV (⚠️ add your own)
├── frame/                 # sample JPG frames for quick test
└── docs/media/            # put screenshots / GIFs here – auto‑referenced in README
```

> **Tip** Add your vehicle‑specific ROS node or CAN writer under a new `vehicle/` folder – `main.py` already prints the filtered angle every frame.

---

## ⚙️  Installation

```bash
python -m venv venv && source venv/bin/activate
pip install opencv-python numpy
# optional (plots / yaml)
pip install matplotlib pyyaml
```

- Python 3.8 – 3.11 verified (Windows / macOS / Linux).  
- For an extra speed‑up, replace `opencv-python` with GPU builds (`opencv-python-headless` + `opencv-contrib-python`).

---

## 🔧 Camera Calibration (once per lens)

1. Print a **10 × 7 chessboard** (inside corners).  
2. Capture at least **5 clear images** from different angles and save them under `./frame/` (or any folder).
3. Run:

```bash
python get_calibration.py --glob "frame/*.jpg" \
                          --pattern 10x7 --square 25 \
                          --out calib_params.json
```

This will write **`calib_params.json`** and a visualization folder `calib_visual/`.  
Copy or symlink the JSON next to `calibration.py` so the detector can load it.

---

## 🚀 Quick Start

### A. Batch frames (offline)

```bash
python main.py               # expects JPGs under ./frame/
```
Press **space** to pause/resume, **q** to quit.

---

## 🏗️  Pipeline Overview

1. **ROI Mask** – trapezoid mask keeps road pixels only.  
2. **Perspective Warp (IPM)** – transforms ROI to Bird‑Eye‑View using calibration matrix.  
3. **Edge Extraction** – HSV+HLS threshold → glare suppression → morphology → Canny.  
4. **HoughLinesP** – extract short segments.  
5. **Merge & Average** – slope filtering merges segments into ≤2 lines.  
6. **x‑offset** – deviation of lane center from image center.  
7. **PID Controller** – converts offset to steering angle with integral anti‑windup, EMA smoothing, and rate limiting.  
8. **Display** – overlay lane, heading line, and textual direction (LEFT / STRAIGHT / RIGHT).

---

## 🛠  PID Tuning

| Symbol | Description | Default in `PIDSteering.py` |
|--------|-------------|-----------------------------|
| `Kp`   | Proportional gain – reacts to current error | `0.55` |
| `Ki`   | Integral gain – removes steady bias         | `0.0005` |
| `Kd`   | Derivative gain – anticipates trend         | `1.1` |
| `ema_alpha` | EMA smoothness (0 = slow, 1 = none)    | `0.10` |
| `rate_limit` | Max Δθ per frame [deg]                | `3` |

1. Start with **`Kp`** only until oscillation appears.  
2. Add **`Kd`** to damp overshoot.  
3. Increase **`Ki`** gently to eliminate constant bias.  
4. Adjust **`rate_limit`** to match physical servo speed if driving a robot car.

PID update runs every frame and returns a **clamped [45°, 135°]** steering command by default.

---

## 🖥️  Debug Views

Set `SHOW_IMAGE = True` in `OpencvLaneDetect.py` to open the following windows:

| Window            | Content                                      |
|-------------------|----------------------------------------------|
| **original**      | Raw BGR frame                                |
| **ROI overlay**   | Semi‑transparent ROI mask                    |
| **white edge**    | Canny edges after threshold / morphology     |
| **line segments** | Hough segments                               |
| **lane lines**    | Averaged left / right lane                   |
| **heading**       | Final steering overlay & numerical angle     |

**Maintainer**: **doyeon**  <dypark@cau.ac.kr>
