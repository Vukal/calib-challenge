import numpy as np
from .geom import compose

def sample_trajectory(n=150):
    ts = np.arange(n, dtype=np.int64) * 100
    T_seq = []
    for i in range(n):
        x = 1.5*np.sin(i/30)
        y = 0.2*np.sin(i/10)
        z = 1.5 + 0.2*np.cos(i/40)
        yaw = 0.3*np.sin(i/35)
        pitch = 0.05*np.sin(i/20)
        roll = 0.03*np.cos(i/25)
        Rz = rot_z(yaw); Ry = rot_y(pitch); Rx = rot_x(roll)
        R = Rz@Ry@Rx
        T = np.eye(4); T[:3,:3]=R; T[:3,3]=np.array([x,y,z])
        T_seq.append(T)
    return ts, T_seq

def rot_x(a): c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def rot_y(a): c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rot_z(a): c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def project_points(K, Xc):
    """
    Xc: Nx3 points in the camera frame.
    Return Nx2 pixel coordinates using pinhole projection.
    """
    # Normalize by depth (Z)
    x = Xc[:, :3] / Xc[:, 2:3]     # -> [X/Z, Y/Z, 1]
    uvw = (K @ x.T).T              # Nx3
    return uvw[:, :2]              # drop w since it's 1 after normalization


def render_detections(t_ns, cams, tags_world, T_w_c1, T_c2_c1, noise_px=0.5):
    rows = []
    # camera 1
    rows += _render_for_cam(t_ns, "cam1", cams["cam1"], K_from(cams["cam1"]),
                            tags_world, T_w_c1, noise_px)
    # camera 2 (compose with fixed extrinsics)
    T_w_c2 = compose(T_w_c1, T_c2_c1)
    rows += _render_for_cam(t_ns, "cam2", cams["cam2"], K_from(cams["cam2"]),
                            tags_world, T_w_c2, noise_px)
    return rows

def K_from(c):
    return np.array([[c["fx"],0,c["cx"]],[0,c["fy"],c["cy"]],[0,0,1]], float)

def _render_for_cam(ts_ns, cam_name, cam_dict, K, tags_world, T_w_c, noise_px):
    rows=[]
    Rcw = T_w_c[:3,:3].T
    tcw = -Rcw @ T_w_c[:3,3]
    for tid,tag in tags_world.items():
        Pw = np.array(tag["corners"])  # 4x3
        Pc = (Rcw @ Pw.T + tcw[:,None]).T
        if (Pc[:,2] > 0.2).all():  # visible if in front
            uv = project_points(K, Pc)
            uv += np.random.normal(0, noise_px, uv.shape)
            rows.append([ts_ns, cam_name, tid,
                         uv[0,0],uv[0,1], uv[1,0],uv[1,1],
                         uv[2,0],uv[2,1], uv[3,0],uv[3,1]])
    return rows
