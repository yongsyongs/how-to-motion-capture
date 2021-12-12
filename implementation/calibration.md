### 구현 방법

> OpenCV에서는 calibrateCamera라는 매우 쉽고 편한 함수를 제공하므로, 관련 [튜토리얼](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d)을 참고하기 바랍니다. OpenCV는 매우 훌륭한 툴이기에, 괜스레 이론을 이해하겠다고 소중한 인생을 낭비하는 행위를 하지 마십시오. 후술되는 코드를 읽을 바에 OpenCV Docs를 읽는 게 이득일 것입니다. 물론 독자께서 관련 연구실 대학원생이라면 얘기가 달라지겠지만요...

Intrinsic Calibration에 활용되는 알고리즘들은 대부분 [2]에서 제시된 방식에 기반으로, co-planar points들의 real world points와 image points가 주어졌을 때 적용할 수 있습니다. **일반적으로 사용되는 체스보드를 통한 캘리브레이션이 이에 해당합니다.** 지금부터 설명할 코드 역시 [2]에서 제시된 알고리즘을 구현한 예시입니다. 

예시 이미지는 EasyMocap의 [레포지토리](https://github.com/zju3dv/EasyMocap/blob/master/apps/calibration/Readme.md)에 업로드된 예시 이미지를 활용하였습니다. 아래와 같이 다운로드합니다.

```python
import os
import numpy as np
from scipy.optimize import least_squares

def donwload_image():
    url1 = "https://raw.githubusercontent.com/zju3dv/EasyMocap/master/apps/calibration/assets/intri_sample.png"
    url2 = "https://raw.githubusercontent.com/zju3dv/EasyMocap/master/apps/calibration/assets/extri_sample.png"
    os.system("curl " + url1 + " > image1.png")
    os.system("curl " + url2 + " > image2.png")

donwload_image()
```

체스보드로부터 이미지상의 픽셀 좌표를 찾고, 이에 대응하는 가상의 실제 좌표를 만들겠습니다.

```python
objp = np.zeros((6 * 9, 3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) + 1 # zero divsion
img1 = cv2.imread('image1.png')
img2 = cv2.imread('image2.png')
grays = [cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY), cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)]
ret1, corners1 = cv2.findChessboardCorners(grays[0], (9, 6), None)
ret2, corners2 = cv2.findChessboardCorners(grays[1], (9, 6), None)
assert ret1 and ret2
m = np.stack([corners1[:, 0, :], corners2[:, 0, :]]) # image points
M = np.stack([objp] * 2) # real world points

m = m.transpose(0, 2, 1); M = M.transpose(0, 2, 1)
```

Intrinsic Calibration을 위해서는 먼저 real world plane과 image plane 사이의 Homography Matrix를 구해야 합니다. scipy의 `least_squares`를 통해 이를 구하는 함수를 작성합니다.

```python
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

H, fun = estimate_homography(m[0], M[0], verbose=2)
```

위 함수가 적절한 호모그라피 변환을 생성하는지는 아래처럼 그림으로 비교하여 볼 수 있습니다.

```python
_m = H @ np.concatenate([M[0, :-1], np.ones((1, M.shape[2]))])
_m = _m[:2, :] / _m[2:, :]
plt.scatter(m[0, 0], m[0, 1])
plt.scatter(_m[0, :], _m[1, :])
```

`estimate_homography` 함수를 이용하여 내부 파라미터를 구하는 행렬을 아래와 같이 작성합니다.

```python
def get_K(b):
    assert b.shape == (6,), b.shape
    b11, b12, b22, b13, b23, b33 = b
    v0 = (b12*b13-b11*b23)/(b11*b22-b12**2)
    lambda_ = b33-(b13**2+v0*(b12*b13-b11*b23))/b11
    alpha = np.sqrt(lambda_/b11)
    beta = np.sqrt(lambda_*b11/(b11*b22-b12**2))
    gamma = -b12*alpha**2*beta/lambda_
    u0 = gamma*v0/beta-b13*alpha**2/lambda_
    return np.array([
        [alpha, gamma, u0],
        [0, beta, v0],
        [0, 0, 1]
    ])

def estimate_intr_params(m, M):
    N_image = m.shape[0]

    Hs = np.stack([estimate_homography(m[i], M[i])[0] for i in range(N_image)])
    
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
        for i in range(N_image)
    ])

    if N_image == 2: V = np.concatenate([V, np.array([[0,1,0,0,0,0]])], axis=0)
    
    w, v = np.linalg.eig(V.T@V)
    K = get_K(v[:, np.argmin(w)])

    return K

K = estimate_intr_params(m, M)
```

제시된 K값은 이후 최적화에 사용할 수 있습니다. homography를 이용할 때와 같이 `least_squares`를 사용합니다.

최적화 과정을 시작하기 전에 다음과 같이 코드를 변경합니다.

```python
def extract_RT(K, H):
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
        for i in range(m.shape[0])
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
```

그리고 몇 가지 유용한 함수들을 선언합니다.

````python
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
````

rotation matrix의 실질적 자유도는 3이기에 크기가 9인 변수를 활용하는 것보다 3인 변수를 활용하는 게 더 효율적입니다. 따라서 행렬의 exponential/log map을 활용합니다. 이를 Rodrigues 표현법이라 합니다. 이러한 접근법은 [1], [3]을 참조하였습니다.

지금까지 작성한 함수들을 바탕으로 최적화 코드를 작성합니다.

```python
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
```

이 함수로부터 도출된 결과를 OpenCV와 비교하면 다음과 같습니다. 

```python
# cv2 calibration
flags = cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6 | cv2.CALIB_FIX_S1_S2_S3_S4
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    M.transpose(0, 2, 1), m.transpose(0, 2, 1), 
    grays[0].shape[::-1], None, None, None, flags=flags,
)
Rs = np.stack([cv2.Rodrigues(rvec)[0] for rvec in rvecs])
Ts = np.stack(tvecs)

print('OpenCV')
print('\tfocal length:', (mtx[[0, 1], [0, 1]]))
print('\tprincipal points:', (mtx[[0, 1], [2, 2]]))
print('\treprojection error', reprojection_error(mtx, Rs, Ts))

Hs = np.stack([estimate_homography(m[i], M[i])[0] for i in range(m.shape[0])])
K_init = get_initial_K(Hs)
extrs = [extract_RT(K_init, Hs[i]) for i in range(Hs.shape[0])]
Rs = np.stack([x[0] for x in extrs])
Ts = np.stack([x[1] for x in extrs])
print('initial parameters')
print('\tfocal length:', (K_init[[0, 1], [0, 1]]))
print('\tprincipal points:', (K_init[[0, 1], [2, 2]]))
print('\treprojection: error', reprojection_error(K_init, Rs, Ts))
K, Rs, Ts = optimize_K(m, M, Hs, K_init)
print('optimized parameters')
print('\tfocal length:', (K[[0, 1], [0, 1]]))
print('\tprincipal points:', (K[[0, 1], [2, 2]]))
print('\treprojection: error', reprojection_error(K, Rs, Ts))
```

```python
OpenCV
    focal length: [1178.44594422 1147.19909444]
    principal points: [368.23160359 531.07523092]
    reprojection error: 2.1471591276286115
initial parameters
    focal length: [1172.05393581 1139.24265246]
    principal points: [451.57665774 505.26857517]
    reprojection: error 0.7009996289405573
optimized parameters
    focal length: [1166.52501258 1134.91050169]
    principal points: [451.77568397 502.7725782 ]
    reprojection: error 0.7007539550377244
```

현재는 이미지가 두 장이지만, 더 많은 이미지라던가 왜곡이 반영되는 상황 등에서는 OpenCV의 오차가 더 작을 수 있습니다. 특히, OpenCV의 함수는 **상당히 범용적으로 잘** 설계되었기에 입력 파라미터 및 flag에 따라서 상이한 결과를 냅니다.

다시 말씀드리지만, 관련 전공이 아니라면 그냥 OpenCV 쓰시는 게 정신 건강에 이롭습니다.
