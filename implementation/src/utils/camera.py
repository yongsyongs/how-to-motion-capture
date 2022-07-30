import numpy as np
from tqdm import tqdm
from scipy.optimize import least_squares


class NotPerpendicularError(ValueError):
    pass


def estimate_homography(m: np.ndarray, M: np.ndarray, *args, **kwargs) -> np.ndarray:
    """영상에서 관측된 점과 실제 점 좌표로부터 카메라의 호모그라피 행렬을 계산한다.
    점들은 co-planar해야 한다.

    Parameters
    ----------
    m : np.ndarray
        영상에서 촬영된 점. (2, N)
    M : np.ndarray
        실제 점 좌표. (3, N)

    Returns
    -------
    np.ndarray
        계산된 호모그라피 행렬
    """

    assert len(m.shape) == len(M.shape) == 2
    assert m.shape[1] == M.shape[1]
    assert m.shape[0] == 2 and M.shape[0] == 3
    N = m.shape[1]

    M = np.concatenate([M[:-1], np.ones((1, N))])

    L = np.zeros((2 * N, 9))
    for i in range(N):
        L[2 * i, :3] = M[:, i]
        L[2 * i, 6:] = -m[0, i] * M[:, i]
        L[2 * i + 1, 3:6] = M[:, i]
        L[2 * i + 1, 6:] = -m[1, i] * M[:, i]

    w, v = np.linalg.eig(L.T @ L)
    i = np.argmin(w)
    x0 = v[:, i]

    def fun(x):
        _H = x.reshape(3, 3)
        _m = _H @ M
        _m = _m[:-1, :] / _m[-1:, :]
        _r = np.linalg.norm(m - _m, axis=0)

        return _r

    res = least_squares(fun, x0, method="lm", *args, **kwargs)
    H = res.x.reshape(3, 3)
    return H


def calculate_relative_extr(K, H):
    RT = np.linalg.inv(K) @ H
    r1 = RT[:, 0]
    r2 = RT[:, 1]
    t = RT[:, 2]

    s1 = np.linalg.norm(r1)
    s2 = np.linalg.norm(r2)
    r1 /= s1
    r2 /= s2
    if np.abs(np.dot(r1, r2)) > 2e-1:
        raise NotPerpendicularError("두 벡터가 수직하지 않는다. " + str(np.dot(r1, r2)))

    r3 = np.cross(r1, r2)
    t /= (s1 + s2) / 2
    R = np.stack([r1, r2, r3]).T
    T = t[..., np.newaxis]

    return R, T


def linear_triangulation(points_2d, P_mats):
    # points_2d: (n_cameras, n_points, 2)
    # P_mats: (n_cameras, 3, 4)
    n_cameras = P_mats.shape[0]
    n_points = points_2d.shape[1]

    A = []
    for i in range(n_cameras):
        # x: (n_points, 2)   | x[:, ?]: (n_points,)
        # P: (3, 4)          | P[?, :]: (4,)
        x = points_2d[i]
        P = P_mats[i]
        A.append(x[:, 0, np.newaxis] * P[np.newaxis, 2, :] - P[np.newaxis, 0, :])
        A.append(x[:, 1, np.newaxis] * P[np.newaxis, 2, :] - P[np.newaxis, 1, :])

    # Calculate best point
    A = np.array(A)  # (n_cameras, n_points, 4)
    A = A.transpose(1, 0, 2)  # (n_points, n_cameras, 4)
    U, D, Vt = np.linalg.svd(A)  # Singular Value Decomposition
    X = Vt[:, -1, 0:3] / Vt[:, -1, 3, np.newaxis]  # normalize

    return X  # (n_points, 3)


def project(points_3d, P_mats):
    # points_3d: (n_points, 3)
    # P_mats: (n_cameras, 3, 4)
    n_points = points_3d.shape[0]
    points_3d = np.concatenate(
        [points_3d, np.ones((n_points, 1))], axis=1
    ).T  # (4, n_points)
    points_2d = np.einsum(
        "cij, jk -> cik", P_mats, points_3d
    )  # (n_cameras, 3, n_points)
    points_2d = points_2d[:, :-1] / points_2d[:, -1:]  # (n_cameras, 2, n_points)
    points_2d = points_2d.transpose(0, 2, 1)  # (n_cameras, n_points, 2)

    return points_2d  # (n_cameras, n_points, 2)


def reproject(points_2d, P_mats):
    # points_2d: (n_cameras, n_points, 2)
    # P_mats: (n_cameras, 3, 4)
    points_3d = linear_triangulation(points_2d, P_mats)  # (n_points, 3)
    points_2d = project(points_3d, P_mats)  # (n_cameras, n_points, 2)
    return points_2d, points_3d


def calculate_reprojection_error(
    points_2d, P_mats, to_scalar=np.mean, with_points=False
):
    # points_2d: (n_cameras, n_points, 2)
    # P_mats: (n_cameras, 3, 4)
    reprojected_points_2d, reconstructed_points_3d = reproject(
        points_2d, P_mats
    )  # (n_cameras, n_points, 2), (n_points, 3)
    error = np.linalg.norm(points_2d - reprojected_points_2d, axis=-1)
    if to_scalar is not None:
        error = to_scalar(error)

    ret = error
    if with_points:
        ret = (error, reprojected_points_2d, reconstructed_points_3d)

    return ret
