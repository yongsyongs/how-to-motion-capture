import numpy as np
def trace(A):
    return np.sum(np.diag(A))

def skew_symmetry_matrix(v): # skew-symmetric matrix
    assert v.shape == (3, 1) or v.shape == (3,)
    if v.shape == (3, 1):
        v = v.reshape(-1)
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])

def exp_map(w):
    assert w.shape == (3, 1)
    theta = np.linalg.norm(w)
    I = np.eye(3)
    if theta == 0:
        return I
    sk_w = skew_symmetry_matrix(w)
    exp = np.cos(theta) * I + (np.sin(theta) / theta) * sk_w + ((1 - np.cos(theta)) / theta ** 2) * w @ w.T
    assert exp.shape == (3, 3)
    return exp

def log_map(R):
    assert R.shape == (3, 3)
    theta = np.arccos((trace(R) - 1) / 2)
    if np.isclose(theta, 0):
        return np.zeros((3, 1), dtype=np.float32)
    w = (theta / (2 * np.sin(theta))) * np.array([
        [R[2, 1] - R[1, 2]],
        [R[0, 2] - R[2, 0]],
        [R[1, 0] - R[0, 1]]
    ], dtype=np.float32)
    return w