import os
import argparse
import numpy as np
from scipy.optimize import least_squares
from tqdm import tqdm


class NotPerpendicularError(ValueError):
    pass


def get_reprojection_error(m, M, K, R, T):
    X = np.einsum("nij, njk -> nik", R, M) + T
    X /= X[:, -1:]
    X = np.einsum("ij, njk -> nik", K, X)
    e = X[:, :-1] - m
    mse = np.mean(e ** 2)
    rmse = np.sqrt(mse)
    return rmse


def find_H_from_M(m, M, *args, **kwargs):
    assert len(m.shape) == len(M.shape) == 2
    assert m.shape[1] == M.shape[1]
    assert m.shape[0] == 2 and M.shape[0] == 3
    # m: (2, N), M: (3, N)
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


def find_K_from_H(Hs):
    N_image = Hs.shape[0]

    h = lambda k, i, j: Hs[k, j - 1, i - 1]
    v = lambda k, i, j: np.array(
        [
            [
                h(k, i, 1) * h(k, j, 1),
                h(k, i, 1) * h(k, j, 2) + h(k, i, 2) * h(k, j, 1),
                h(k, i, 2) * h(k, j, 2),
                h(k, i, 3) * h(k, j, 1) + h(k, i, 1) * h(k, j, 3),
                h(k, i, 3) * h(k, j, 2) + h(k, i, 2) * h(k, j, 3),
                h(k, i, 3) * h(k, j, 3),
            ]
        ]
    ).T

    V = np.concatenate(
        [
            np.concatenate([v(i, 1, 2).T, (v(i, 1, 1) - v(i, 2, 2)).T], axis=0)
            for i in range(Hs.shape[0])
        ]
    )

    if N_image == 2:
        V = np.concatenate([V, np.array([[0, 1, 0, 0, 0, 0]])], axis=0)

    w, v = np.linalg.eig(V.T @ V)
    b = v[:, np.argmin(w)]
    b11, b12, b22, b13, b23, b33 = b
    v0 = (b12 * b13 - b11 * b23) / (b11 * b22 - b12 ** 2)
    lambda_ = b33 - (b13 ** 2 + v0 * (b12 * b13 - b11 * b23)) / b11
    alpha = np.sqrt(lambda_ / b11)
    beta = np.sqrt(lambda_ * b11 / (b11 * b22 - b12 ** 2))
    # gamma = -b12 * alpha ** 2 * beta / lambda_
    # u0 = gamma * v0 / beta - b13 * alpha ** 2 / lambda_
    gamma = 0
    u0 = gamma * v0 / beta - b13 * alpha ** 2 / lambda_
    K = np.array([[alpha, gamma, u0], [0, beta, v0], [0, 0, 1]])

    return K


def find_RT_from_KH(K, H):
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


def find_K_from_M(m, M):
    n_iter = 100
    n_samples = m.shape[0] // 10

    homography_matrices = np.stack(
        [find_H_from_M(m[i], M) for i in tqdm(range(m.shape[0]), leave=False)]
    )

    def fun(K, R, T):
        X = np.einsum("nij, jk -> nik", R, M) + T
        X /= X[:, -1:]
        X = np.einsum("ij, njk -> nik", K, X)
        e = X[:, :-1] - m
        e = np.linalg.norm(e, axis=1) ** 2
        mse = e.mean(axis=1)
        rmse = np.sqrt(mse)
        return rmse

    max_inlier_count = 0
    best_K = None
    for i in tqdm(range(n_iter), leave=False):
        args = np.random.choice(np.arange(homography_matrices.shape[0]), n_samples)
        sample_homography_matrices = homography_matrices[args]
        try:
            K = find_K_from_H(sample_homography_matrices)
            RTs = np.stack(
                [
                    np.concatenate(find_RT_from_KH(K, H), axis=1)
                    for H in homography_matrices
                ]
            )
            rmse = fun(K, RTs[..., :3], RTs[..., 3:])
            inlier_count = (rmse < 10).sum()

            if inlier_count > max_inlier_count:
                max_inlier_count = inlier_count
                best_K = K
        except np.linalg.LinAlgError as e:
            continue
        except NotPerpendicularError as e:
            continue
    assert best_K is not None
    return best_K


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("marker_dir", type=str)
    parser.add_argument("output_dir", type=str)
    args = parser.parse_args()

    names = os.listdir(args.marker_dir)
    os.makedirs(args.output_dir, exist_ok=True)

    camera_matrices = []

    for name in tqdm(names):
        data = np.load(os.path.join(args.marker_dir, name), allow_pickle=True).tolist()
        frame_numbers = data["frame_numbers"]
        m = data["markers"].transpose(0, 2, 1)

        # 피사체에 알맞게 수정하세요.
        M = np.array(
            [
                [0, 0, 1],
                [1, 0, 1],
                [2, 0, 1],
                [0, 1, 1],
                [1, 1, 1],
                [2, 1, 1],
                [0, 2, 1],
                [1, 2, 1],
                [2, 2, 1],
                [4, 2.5, 1],
            ]
        ).T

        K = find_K_from_M(m, M)
        np.save(os.path.join(args.output_dir, name), K)
