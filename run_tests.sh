#!/usr/bin/env bash
set -e
# shellcheck disable=SC1091
source .venv/bin/activate 2>/dev/null || source .venv/Scripts/activate

python - << 'PY'
import json, numpy as np
gt = json.load(open("out/sim_gt.json"))
est = json.load(open("out/results.json"))
Tgt = np.array(gt["T_cam2_cam1_matrix"])
Test = np.array(est["T_cam2_cam1_matrix"])
def se3_err(A,B):
    D = np.linalg.inv(A)@B
    R = D[:3,:3]
    ang = np.arccos(max(-1,min(1,(np.trace(R)-1)/2)))
    t = np.linalg.norm(D[:3,3])
    return ang, t
ang, t = se3_err(Tgt, Test)
print(f"Angle error (deg): {np.degrees(ang):.3f}")
print(f"Trans error (m):  {t:.4f}")
PY
