#!/usr/bin/env python3
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import argparse, json, os
import numpy as np
from src.sim import sample_trajectory, render_detections
from src.io_utils import write_cams, write_detections_csv, write_tags, save_json

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="out")
    ap.add_argument("--n_steps", type=int, default=150)
    ap.add_argument("--noise_px", type=float, default=0.5)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    # Cameras (intrinsics) (matches spec)
    cams = {
        "cam1": {"fx": 920.0, "fy": 918.0, "cx": 640.0, "cy": 360.0},
        "cam2": {"fx": 915.0, "fy": 917.0, "cx": 640.0, "cy": 360.0},
        "tag_size_m": 0.12,
    }

    # Ground truth fixed extrinsics cam2 wrt cam1
    T_cam2_cam1 = np.eye(4)
    T_cam2_cam1[:3,:3] = np.array([[0.9999, 0.0100, 0.0070],
                                   [-0.0100, 0.9999, -0.0020],
                                   [-0.0070, 0.0021, 0.9999]])
    T_cam2_cam1[:3,3] = np.array([0.20, 0.01, 0.02])

    # Tag layout (grid on z=0)
    tags_world = {}
    grid = [(x,y) for x in range(-2,3) for y in range(-1,2)]
    for i,(gx,gy) in enumerate(grid):
        size = cams["tag_size_m"]
        cx, cy, cz = gx*0.6, gy*0.6, 2.0
        s = size/2
        corners = [
            [cx - s, cy - s, cz],
            [cx + s, cy - s, cz],
            [cx + s, cy + s, cz],
            [cx - s, cy + s, cz],
        ]
        tags_world[i] = {"corners": corners}

    # Rig poses (cam1 is reference on the rig)
    ts, T_world_cam1_seq = sample_trajectory(n=args.n_steps)

    # project detections for both cameras
    det_rows = []
    for t_ns, T_w_c1 in zip(ts, T_world_cam1_seq):
        det_rows += render_detections(
            t_ns, cams, tags_world, T_w_c1, T_cam2_cam1, noise_px=args.noise_px
        )

    # Write files
    write_cams(os.path.join(args.out, "cams.json"), cams)
    write_tags(os.path.join(args.out, "tags.json"), tags_world)
    write_detections_csv(os.path.join(args.out, "detections.csv"), det_rows)

    save_json(os.path.join(args.out, "sim_gt.json"), {
        "T_cam2_cam1_matrix": T_cam2_cam1.tolist()
    })

    print(f"✅ Synthetic data written to {args.out}")

if __name__ == "__main__":
    main()
