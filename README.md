# Track B — Multi-Sensor Extrinsics Calibration (AprilTags)

Estimate the rigid transform **T_cam2_cam1** between two cameras mounted on a rigid rig from AprilTag corner detections.

This repo includes:

- **Phase 1 – Simulator**: Generates synthetic detections with known intrinsics, tag size, and tag world layout.
- **Phase 2 – Solver**: Recovers **T_cam2_cam1** from detections using per-frame PnP and robust aggregation.

All outputs are written to `/out`.

---

## Quickstart (one command)

```bash
# macOS/Linux or Git Bash on Windows
bash run.sh
```

This will:
1) Create/activate a virtualenv  
2) Install dependencies  
3) Generate synthetic data (`/out/cams.json`, `/out/tags.json`, `/out/detections.csv`, `/out/sim_gt.json`)  
4) Estimate extrinsics and write `/out/results.json`

Optional verification:

```bash
bash run_tests.sh
```

Prints rotation (deg) and translation (m) error between the estimated transform and simulator ground truth.

---

## Requirements

- Python 3.10+ (tested on 3.12)
- OS: Windows 10/11, macOS, or Linux
- Packages (installed by `run.sh`): `numpy`, `pandas`, `opencv-python`, `scipy`

---

## Repository layout

```
.
├─ bin/
│  ├─ simulate.py      # Phase 1: generate cams/tags/detections
│  └─ calibrate.py     # Phase 2: estimate T_cam2_cam1 → results.json
├─ src/
│  ├─ __init__.py
│  ├─ geom.py          # SE(3) compose/invert, Euler helpers
│  ├─ io_utils.py      # read/write CSV/JSON (robust to whitespace)
│  ├─ sim.py           # trajectory, projection, render detections
│  └─ solver.py        # PnP, track building, robust aggregation, RMSE
├─ out/                # generated data + results (created by scripts)
├─ run.sh              # venv + simulate + calibrate
├─ run_tests.sh        # compares results.json vs sim_gt.json
├─ requirements.txt
└─ README.md
```

---

## How to run (alternative ways)

### A) Direct commands

```bash
# (activate your venv if not using run.sh)
python bin/simulate.py --out out
python bin/calibrate.py --cams out/cams.json --detections out/detections.csv --tags out/tags.json --out out
```

### B) Windows CMD / PowerShell (no bash)

```bat
.\.venv\Scripts\activate.bat
pip install -r requirements.txt

python bin\simulate.py --out out
python bin\calibrate.py --cams out\cams.json --detections out\detections.csv --tags out	ags.json --out out
```

---

## Outputs

- `out/cams.json`
  ```json
  {
    "cam1": {"fx": 920.0, "fy": 918.0, "cx": 640.0, "cy": 360.0},
    "cam2": {"fx": 915.0, "fy": 917.0, "cx": 640.0, "cy": 360.0},
    "tag_size_m": 0.12
  }
  ```
- `out/tags.json` — world-frame tag corners (meters), CCW order per tag id  
- `out/detections.csv` — AprilTag corner detections per frame:
  ```
  ts_ns,cam_id,tag_id,u0,v0,u1,v1,u2,v2,u3,v3
  ```
- `out/sim_gt.json` — simulator ground truth:
  ```json
  { "T_cam2_cam1_matrix": [[...],[...],[...],[0,0,0,1]] }
  ```
- `out/results.json` — solver estimate:
  ```json
  {
    "T_cam2_cam1_matrix": [[r11,r12,r13,tx],[r21,r22,r23,ty],[r31,r32,r33,tz],[0,0,0,1]],
    "rpy_deg": [roll, pitch, yaw],
    "xyz_m": [tx, ty, tz],
    "reproj_rmse_px": <float>,
    "pairs_used": <int>
  }
  ```

---

## Method (solver)

1. **Per-timestamp PnP**  
   For each camera and timestamp, pick any visible tag and run `cv2.solvePnP` (tries RANSAC → ITERATIVE → IPPE_SQUARE) to get **T_cam_world** from the 4 tag corners and known intrinsics.

2. **Pairing**  
   Prefer exact timestamp overlap. If none (CSV quirks), pair frames by sorted order (synthetic data stays time-aligned).

3. **Compose extrinsics**  
   For each pair, compute **T_cam2_cam1 = T_cam2_world · inv(T_cam1_world)**, then robust-aggregate across all pairs.

4. **Robust aggregation**  
   - Translations: component-wise median  
   - Rotations: quaternion median (normalized)  
   Final transform is returned (matching the simulator’s convention).

5. **Verification**  
   - **SE(3) error** vs ground truth (angle + translation) in `run_tests.sh`  
   - **Reprojection RMSE** over the exact points used by PnP

---

## Verification

**Q1. How do you know your code is accurate?**  
- Simulate ground-truth **T_cam2_cam1** and detect AprilTag corners with pixel noise.  
- The solver’s estimate is compared against the ground truth using SE(3) residuals: rotation angle and translation norm.

**Q2. What have you done to verify correctness?**  
- `run_tests.sh` prints angle (deg) and translation (m) error between `out/results.json` and `out/sim_gt.json`.  
- Reprojection RMSE in `results.json` reports pixel residuals for the exact corners used by PnP.  
- With default noise (`--noise_px 0.5`), SE(3) errors are low; with smaller noise they fall near zero.

**Q3. How can someone else check your verification?**  
- Re-run `bash run.sh` and then `bash run_tests.sh`.  
- Inspect `out/results.json` vs `out/sim_gt.json`.  
- Optional: change noise level:
  ```bash
  python bin/simulate.py --out out --noise_px 0.2
  python bin/calibrate.py --cams out/cams.json --detections out/detections.csv --tags out/tags.json --out out
  bash run_tests.sh
  ```

**Current run (example):**
```
Angle error (deg): 0.000
Trans error (m):  0.0036
```

---

