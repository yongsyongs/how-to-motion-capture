import random
import numpy as np
from scipy.optimize import least_squares
from tqdm import tqdm

def get_reprojection_error(m, M, K, R, T):
    X = np.einsum('nij, njk -> nik', R, M) + T
    X /= X[:, -1:]
    X = np.einsum('ij, njk -> nik', K, X)
    e = X[:, :-1] - m
    mse = np.mean(e ** 2)
    rmse = np.sqrt(mse)
    return rmse

def find_homography(m, M, *args, **kwargs):
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
    return res.x.reshape(3, 3)

def find_intrinsic_parameters(Hs):
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

def extract_extrinsic_parameters(K, H):
    RT = np.linalg.inv(K) @ H
    r1 = RT[:, 0]
    r2 = RT[:, 1]
    t = RT[:, 2]

    s1 = np.linalg.norm(r1)
    s2 = np.linalg.norm(r2)
    r1 /= s1
    r2 /= s2
    assert np.dot(r1, r2) < 1e-6, np.dot(r1, r2)
    r3 = np.cross(r1, r2)
    t /= (s1 + s2) / 2
    R = np.stack([r1, r2, r3]).T
    T = t[..., np.newaxis]

    return R, T


def optimize_intrinsic_parameters(m, M):
    n_iter = 100
    n_samples = 10

    Hs = np.stack([find_homography(m[i], M) for i in tqdm(range(m.shape[0]))])
    def fun(K, R, T):
        X = np.einsum('nij, jk -> nik', R, M) + T
        X /= X[:, -1:]
        X = np.einsum('ij, njk -> nik', K, X)
        e = X[:, :-1] - m
        e = np.linalg.norm(e, axis=1) ** 2
        mse = e.mean(axis=1)
        rmse = np.sqrt(mse)
        return rmse

    def get_inlier_count(rmse):
        return (rmse < 3).sum()


    max_inlier_count = 0
    best_K = None
    for i in tqdm(range(n_iter)):
        args = np.random.choice(np.arange(Hs.shape[0]), n_samples)
        sample_Hs = Hs[args]
        K = find_intrinsic_parameters(sample_Hs)
        RTs = np.stack([np.concatenate(extract_extrinsic_parameters(K, H), axis=1) for H in sample_Hs])
        rmse = fun(K, RTs[..., :3], RTs[..., 3:])
        inlier_count = get_inlier_count(rmse)
        if inlier_count > max_inlier_count:
            max_inlier_count = inlier_count
            best_K = K

    return best_K
            
if __name__ == '__main__':
    m = np.ones((128, 2, 10))
    M = np.ones((3, 10))
    optimize_intrinsic_parameters(m, M)