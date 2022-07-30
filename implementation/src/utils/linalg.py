import numpy as np


def trace(A):
    """
    행렬 A의 trace값을 계산한다.
    * trace: 행렬의 대각 성분들의 합
    """
    return np.sum(np.diag(A))


def skew_symmetry_matrix(v):
    """
    3차원 벡터로부터 skew symmetry matrix를 계산한다.
    """
    assert v.shape == (3, 1) or v.shape == (3,)
    if v.shape == (3, 1):
        v = v.reshape(-1)
    return np.array(
        [
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0],
        ]
    )


def exp_map(w):
    """
    3차원 벡터를 exponential mapping해 행렬로 반환한다.
    log_map의 역함수.
    """

    assert w.shape == (3, 1)
    theta = np.linalg.norm(w)
    I = np.eye(3)
    if theta == 0:
        return I
    w = w / theta
    sk_w = skew_symmetry_matrix(w)
    exp = (
        np.cos(theta) * I
        + (np.sin(theta) / theta) * sk_w
        + ((1 - np.cos(theta)) / theta**2) * w @ w.T
    )
    assert exp.shape == (3, 3)
    return exp


def log_map(R):
    """
    3 by 3 행렬을 log mapping해 벡터로 반환한다.
    exp_map의 역함수.
    """
    assert R.shape == (3, 3)
    theta = np.arccos((trace(R) - 1) / 2)
    if np.isclose(theta, 0):
        return np.zeros((3, 1), dtype=np.float32)
    w = (theta / (2 * np.sin(theta))) * np.array(
        [[R[2, 1] - R[1, 2]], [R[0, 2] - R[2, 0]], [R[1, 0] - R[0, 1]]],
        dtype=np.float32,
    )
    return w
