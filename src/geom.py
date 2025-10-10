import numpy as np
from scipy.spatial.transform import Rotation as R

def rodrigues_to_R(rvec):
    theta = np.linalg.norm(rvec)
    if theta < 1e-12: return np.eye(3)
    k = rvec/theta
    K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K@K)

def compose(A,B):
    C = np.eye(4)
    C[:3,:3] = A[:3,:3] @ B[:3,:3]
    C[:3,3]  = A[:3,:3] @ B[:3,3] + A[:3,3]
    return C

def invert(T):
    Ri = T[:3,:3].T
    ti = -Ri @ T[:3,3]
    X = np.eye(4); X[:3,:3]=Ri; X[:3,3]=ti
    return X

def rpy_from_R(Rm):
    return R.from_matrix(Rm).as_euler('xyz', degrees=False)
