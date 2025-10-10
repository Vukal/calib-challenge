import json, csv
import numpy as np
from pathlib import Path

def write_cams(path, cams_dict):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f: json.dump(cams_dict, f, indent=2)

def write_tags(path, tags_world):
    with open(path,"w") as f: json.dump({"tags": tags_world}, f, indent=2)

def write_detections_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ts_ns","cam_id","tag_id","u0","v0","u1","v1","u2","v2","u3","v3"])
        w.writerows(rows)

def save_json(path, obj):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path,"w") as f: json.dump(obj,f,indent=2)

def read_cams(path):
    j = json.load(open(path))
    cams = {}
    for k in list(j.keys()):
        if k == "tag_size_m": continue
        d = j[k]
        K = np.array([[d["fx"],0,d["cx"]],
                      [0,d["fy"],d["cy"]],
                      [0,0,1]], dtype=float)
        cams[k] = {"K": K, "dist": np.zeros(5)}
    cams["tag_size_m"] = j["tag_size_m"]
    return cams

def read_tags(path):
    j = json.load(open(path))
    tags = {int(tid): np.array(v["corners"], dtype=float) for tid,v in j["tags"].items()}
    return tags

def read_detections_csv(path):
    import pandas as pd
    import numpy as np

    # Read & normalize headers
    df = pd.read_csv(path)
    df.columns = [str(c).strip() for c in df.columns]

    rows = []
    for _, r in df.iterrows():
        cam_id = str(r["cam_id"]).strip().lower()  # "cam1" / "cam2"
        if cam_id not in ("cam1", "cam2"):
            # tolerate variants like "CAM1", " cam1 "
            if cam_id.replace(" ", "") in ("cam1", "cam2"):
                cam_id = cam_id.replace(" ", "")
            else:
                # skip unknown cam_ids
                continue

        rows.append({
            "ts_ns": int(r["ts_ns"]),
            "cam_id": cam_id,
            "tag_id": int(r["tag_id"]),
            "corners": np.array([
                [float(r["u0"]), float(r["v0"])],
                [float(r["u1"]), float(r["v1"])],
                [float(r["u2"]), float(r["v2"])],
                [float(r["u3"]), float(r["v3"])],
            ], dtype=float),
        })
    return rows


