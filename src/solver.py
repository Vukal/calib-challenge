import numpy as np, cv2
from collections import defaultdict
from .geom import invert

def estimate_pose_pnp(K, corners_px, corners_w):
    obj = corners_w.astype(np.float32)
    img = corners_px.astype(np.float32)

    # Try robust first
    try:
        ok, rvec, tvec, _ = cv2.solvePnPRansac(obj, img, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            R, _ = cv2.Rodrigues(rvec); T = np.eye(4); T[:3,:3]=R; T[:3,3]=tvec[:,0]; return T
    except Exception:
        pass
    # Fallback 1
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if ok:
        R, _ = cv2.Rodrigues(rvec); T = np.eye(4); T[:3,:3]=R; T[:3,3]=tvec[:,0]; return T
    # Fallback 2: planar square-specific
    try:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            R, _ = cv2.Rodrigues(rvec); T = np.eye(4); T[:3,:3]=R; T[:3,3]=tvec[:,0]; return T
    except Exception:
        pass
    return None

def estimate_extrinsics_cam2_cam1(cams, tags, rows):
    """
    Build pose tracks for cam1 and cam2 independently, pair by timestamp (or by order if needed),
    compose T_cam2_cam1 per pair, and robust-aggregate (median t, quaternion median R).
    """
    by_ts_cam = defaultdict(list)
    for r in rows:
        by_ts_cam[(int(r["ts_ns"]), str(r["cam_id"]).strip().lower())].append(r)

    K = {"cam1": cams["cam1"]["K"], "cam2": cams["cam2"]["K"]}

    poses = {"cam1": {}, "cam2": {}}
    meta  = {"cam1": {}, "cam2": {}}

    for (ts, cam), dets in by_ts_cam.items():
        if cam not in ("cam1","cam2"): 
            continue
        # use the first detection at this timestamp for that camera
        d = dets[0]
        tid = int(d["tag_id"])
        if tid not in tags:
            continue
        Pw  = tags[tid]  # 4x3 world corners
        T   = estimate_pose_pnp(K[cam], d["corners"], Pw)
        if T is None:
            continue
        poses[cam][ts] = T
        meta[cam][ts]  = {"corners_px": d["corners"], "Pw": Pw}

    if not poses["cam1"] or not poses["cam2"]:
        # Helpful debug to the console
        c1 = len(poses["cam1"]); c2 = len(poses["cam2"])
        raise RuntimeError(f"No poses for cam1 or cam2 were estimated. cam1={c1}, cam2={c2}")

    ts1 = sorted(poses["cam1"].keys())
    ts2 = sorted(poses["cam2"].keys())

    # Prefer exact overlap
    common = sorted(set(ts1) & set(ts2))
    if common:
        pairs = [(t, t) for t in common]
    else:
        # Fallback: pair by order (always works on our sim)
        n = min(len(ts1), len(ts2))
        pairs = [(ts1[i], ts2[i]) for i in range(n)]

    Ts = []
    per_pose = []
    for t1, t2 in pairs:
        T1 = poses["cam1"][t1]
        T2 = poses["cam2"][t2]
        T21 = T2 @ invert(T1)   # cam2 wrt cam1
        Ts.append(T21)

        m1 = meta["cam1"][t1]; m2 = meta["cam2"][t2]
        per_pose.append({"t": t1, "cam": "cam1", "T_cam_world": T1,
                         "corners_px": m1["corners_px"], "Pw": m1["Pw"]})
        per_pose.append({"t": t2, "cam": "cam2", "T_cam_world": T2,
                         "corners_px": m2["corners_px"], "Pw": m2["Pw"]})

    Ts = np.stack(Ts, axis=0)
    t_med = np.median(Ts[:, :3, 3], axis=0)

    from scipy.spatial.transform import Rotation as R
    qs = R.from_matrix(Ts[:, :3, :3]).as_quat()
    qm = np.median(qs, axis=0); qm /= np.linalg.norm(qm)
    Rm = R.from_quat(qm).as_matrix()

    T21_med = np.eye(4); T21_med[:3,:3]=Rm; T21_med[:3,3]=t_med
    return invert(T21_med), len(pairs), per_pose


def compute_reproj_rmse(cams, tags, per_pose):
    """
    Reproject the world tag corners used for each per-frame pose and compute RMSE in pixels.
    """
    import cv2
    err2, n = 0.0, 0
    for p in per_pose:
        K = cams[p["cam"]]["K"]
        Pw = p["Pw"].astype(np.float32)                 # 4x3
        T = p["T_cam_world"]
        R = T[:3, :3]
        t = T[:3, 3:4]                                  # 3x1
        rvec, _ = cv2.Rodrigues(R)
        proj, _ = cv2.projectPoints(Pw, rvec, t, K, None)
        proj = proj.reshape(-1, 2)                      # 4x2
        e = proj - p["corners_px"]
        err2 += float((e ** 2).sum())
        n += e.size
    return (err2 / n) ** 0.5 if n > 0 else 0.0

