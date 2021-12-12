import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

def reprojection_error(K, R, T):
    X = np.einsum('nij, njk -> nik', R, M) + T
    X /= X[:, -1:]
    X = np.einsum('ij, njk -> nik', K, X)
    e = X[:, :-1] - m
    mse = np.mean(e ** 2)
    rmse = np.sqrt(mse)
    return rmse

def estimate_homography(m, M, *args, **kwargs):
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

    w, v = np.linalg.eig(L.T@L)
    i = np.argmin(w)
    x0 = v[:, i]
    def fun(x):
        _H = x.reshape(3, 3)
        _m = _H @ M
        _m = _m[:-1, :] / _m[-1:, :]
        _r = np.linalg.norm(m - _m, axis=0)

        return _r

    res = least_squares(fun, x0, method='lm', *args, **kwargs)
    return res.x.reshape(3, 3), res.fun


def extract_RT(K, H):
    RT = np.linalg.inv(K) @ H
    r1 = RT[:, 0]
    r2 = RT[:, 1]
    t = RT[:, 2]

    s1 = np.linalg.norm(r1)
    s2 = np.linalg.norm(r2)
    r1 /= s1
    r2 /= s2
    # assert np.dot(r1, r2) < 1e-6, np.dot(r1, r2)
    r3 = np.cross(r1, r2)
    t /= (s1 + s2) / 2
    R = np.stack([r1, r2, r3]).T
    T = t[..., np.newaxis]

    return R, T

def get_initial_K(Hs):
    N_image = Hs.shape[0]
    
    h = lambda k, i, j: Hs[k, j - 1, i - 1]
    v = lambda k, i, j: np.array([[
        h(k,i,1)*h(k,j,1),
        h(k,i,1)*h(k,j,2) + h(k,i,2)*h(k,j,1),
        h(k,i,2)*h(k,j,2),
        h(k,i,3)*h(k,j,1) + h(k,i,1)*h(k,j,3),
        h(k,i,3)*h(k,j,2) + h(k,i,2)*h(k,j,3),
        h(k,i,3)*h(k,j,3)
    ]]).T

    V = np.concatenate([
        np.concatenate([v(i, 1, 2).T, (v(i, 1, 1) - v(i, 2, 2)).T], axis=0)
        for i in range(Hs.shape[0])
    ])

    if N_image == 2: V = np.concatenate([V, np.array([[0,1,0,0,0,0]])],axis=0)
    
    w, v = np.linalg.eig(V.T@V)
    b = v[:, np.argmin(w)]
    b11, b12, b22, b13, b23, b33 = b
    v0 = (b12*b13-b11*b23)/(b11*b22-b12**2)
    lambda_ = b33-(b13**2+v0*(b12*b13-b11*b23))/b11
    alpha = np.sqrt(lambda_/b11)
    beta = np.sqrt(lambda_*b11/(b11*b22-b12**2))
    gamma = -b12*alpha**2*beta/lambda_
    u0 = gamma*v0/beta-b13*alpha**2/lambda_
    K = np.array([
        [alpha, gamma, u0],
        [0, beta, v0],
        [0, 0, 1]
    ])

    return K

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

def optimize_K(m, M, Hs, K_init, *args, **kwargs):
    N_image = Hs.shape[0]
    N_point = m.shape[2]
    assert m.shape[0] == M.shape[0] == N_image
    assert m.shape[1] == 2 and M.shape[1] == 3
    assert m.shape[2] == M.shape[2] == N_point

    x0 = K_init.ravel()[[0, 2, 4, 5]]
    extrs = [extract_RT(K_init, Hs[i]) for i in range(N_image)]
    ws = np.concatenate([log_map(x[0]) for x in extrs])[..., 0]
    Ts = np.concatenate([x[1] for x in extrs])[..., 0]
    x0 = np.concatenate([x0, ws, Ts])

    def fun(x):
        K = np.array([
            [x[0], 0, x[1]],
            [0, x[2], x[3]],
            [0, 0, 1]
        ])
        ws = x[4:4 + 3 * N_image].reshape(N_image, 3, 1)
        Ts = x[-3 * N_image:].reshape(N_image, 3, 1)
        Rs = np.stack([exp_map(w) for w in ws])

        X = np.einsum('nij, njk -> nik', Rs, M) + Ts
        X = X / X[:, -1:]
        X = np.einsum('ij, njk -> nik', K, X)
        return (m - X[:, :-1]).ravel()

    res = least_squares(fun, x0, method='lm', *args, **kwargs)
    x = res.x

    K = np.array([
        [x[0], 0, x[1]],
        [0, x[2], x[3]],
        [0, 0, 1]
    ])
    ws = x[4:4 + 3 * N_image].reshape(N_image, 3, 1)
    Ts = x[-3 * N_image:].reshape(N_image, 3, 1)
    Rs = np.stack([exp_map(w) for w in ws])

    return K, Rs, Ts

def intrinsic_calibration(m, M):
    # (N_images, Dim, N_points)
    Hs = np.stack([estimate_homography(m[i], M[i])[0] for i in range(m.shape[0])])
    K_init = get_initial_K(Hs)
    K, Rs, Ts = optimize_K(m, M, Hs, K_init)
    return K, Rs, Ts