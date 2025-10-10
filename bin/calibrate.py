#!/usr/bin/env python3
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


import argparse, os, json
import numpy as np
from src.io_utils import read_cams, read_tags, read_detections_csv, save_json
from src.solver import estimate_extrinsics_cam2_cam1, compute_reproj_rmse
from src.geom import rpy_from_R

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", required=True)
    ap.add_argument("--detections", required=True)
    ap.add_argument("--tags", required=True)
    ap.add_argument("--out", default="out")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cams = read_cams(args.cams)
    tags = read_tags(args.tags)
    rows = read_detections_csv(args.detections)

    T_c2_c1, pairs_used, per_pose = estimate_extrinsics_cam2_cam1(
        cams, tags, rows
    )

    rmse = compute_reproj_rmse(cams, tags, per_pose)

    R = T_c2_c1[:3,:3]; t = T_c2_c1[:3,3]
    out = {
        "T_cam2_cam1_matrix": T_c2_c1.tolist(),
        "rpy_deg": list(np.degrees(rpy_from_R(R))),
        "xyz_m": list(t.tolist()),
        "reproj_rmse_px": float(rmse),
        "pairs_used": int(pairs_used),
    }
    save_json(os.path.join(args.out, "results.json"), out)
    print("✅ Wrote", os.path.join(args.out, "results.json"))

if __name__ == "__main__":
    main()
