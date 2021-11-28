## Calibration

직역하자면 **조정** 정도의 뜻을 가지는 캘리브레이션은, "기하학적으로 실제 촬영 환경을 모델링한 뒤, 최적화 방법론을 통해 미지수 값을 찾아내는 과정"입니다. 따라서 의외로 모션캡쳐 작업에는 고도의 수학적 이론이 활용되며, 이들을 이해할수록 모션캡쳐 작업이 쉬워집니다.

Camera Geometry는 카메라로 real world를 촬영하는 환경을 기하학적을 모델링하는 학문입니다. 이를 이해하는 데에는 최소한 선형대수학의 지식이 필요하며, 특히 선형대수학의 기하학적 접근을 잘 이해해야 합니다.

Optimization은 주어진 조건과 목적함수가 있을 때, 목적함수를 최소화(혹은 최대화)하는 미지수를 찾는 일련의 과정과 이를 다룬 학문입니다. 최적화는 매우 깊고 넓은 분야이므로 요구되는 수학적 지식이 매우 깊습니다. 다만 meta적으로, 혹은 추상적으로 이를 이해하려면 미적분학 정도의 지식이 요구됩니다.

이후 내용을 읽기 전에 아래 내용을 서치하여 이해해두시길 추천합니다.

- (Camera Geometry) Pinhole 카메라 모델
- (Camera Geometry) 좌표계 변환, 회전 변환

- (Camera Geometry) 동차 좌표계, Epipolar Geometry
- (Optimization) 최소제곱 최적화

OpenCV라는 훌륭한 라이브러리가 있기 때문에 최적화에 대한 이해는 부족해도 모션캡쳐를 구현하는 데에 큰 문제가 없습니다. 다만, 카메라 좌표계와 동차 좌표계에 대한 이해는 __필수__적이니 반드시 이해하시기 바랍니다.



### Intrinsic Calibration

Intrinsic Calibration은 내부 파라미터를 구합니다. 카메라의 내부 파라미터는 렌즈에 의해 맺히는 상과 실제 이미지 사이의 관계를 나타냅니다.

핀홀 카메라 모델에서 Intrinsic Calibration은 다음과 같은 내부 파라미터 행렬 $K$를 구합니다. 
$$
K=\begin{bmatrix}f_x & 0 & c_x\\0 & f_y & c_y\\0 & 0 & 1\end{bmatrix}
$$
(왜곡 계수에 대해서는 다루지 않습니다.)

카메라가 원점이며, 렌즈가 향하는 방향과 카메라 오른쪽 방향이 각각 $z,x$축인 오른손 (직교) 좌표계를 **카메라 좌표계**라고 합니다. 이 카메라 좌표계 상의 어떤 점 $X=(x,y,z)$는 **동차 좌표계** 변환으로 인해 $P=({x\over z},{y\over z},{z\over z})=(a,b,1)$가 됩니다. 이는 카메라로부터 거리가 1만큼인 거리에 형성된 **상**입니다. 

$K$는 이 상에 간단한 일차 변환을 적용하여 **이미지 픽셀 좌표상의 점**으로 변환합니다.
$$
KP=\begin{bmatrix}f_x & 0 & c_x\\0 & f_y & c_y\\0 & 0 & 1\end{bmatrix}\begin{bmatrix}a\\b\\1\end{bmatrix}=\begin{bmatrix}f_xa+c_x & f_yb+c_y & 1\end{bmatrix}
$$
이미 동차 좌표계로 변환된 시점에서 $z$축 데이터는 큰 의미가 없습니다. 따라서 $K$에 의한 변환은 카메라로부터 거리 1만큼 떨어진 곳에 형성된 상 $(a,b)$을 이미지 픽셀 좌표 $(u,v)=(f_xa+c_x, f_yb+c_y)$로 바꿔주는 변환이라고 할 수 있습니다. 이 $(u,v)$는 실제 이미지상의 픽셀입니다.

$f$를 초점 길이(focal length)라고 하는데, 일반적으로 이해되는 범용적인 의미의 초점길이가 아니고, 카메라 렌즈의 중심으로부터 수광센서까지의 거리를 말합니다. 이는 카메라 좌표계상에서 이미지 좌표계로의 **scale factor** 정도로 이해하면 편합니다. 카메라 좌표계의 공간 단위를 이미지 픽셀 공간 단위로 변환시켜주는 것입니다. 일반적으로 이미지 해상도와 비슷한 영역대의 단위를 가집니다. (이미지가 $1000\times1000$ 해상도라면 $f$는 $600$ 또는 $1500$과 같은 값을 가질 수 있습니다.)

$c$는 주점(principal point) 좌표인데, 실제 수광 센서의 중앙과 렌즈의 중앙 사이의 차이를 모델링함과 동시의 이미지에서 원점이 중앙이 아닌 좌상단에 위치하도록 합니다. $c$를 적당히 이미지 중앙 좌표라고 가정하는 게 처음 공부하는 입장에서 타당해 보일 수 있습니다. 그러나 카메라 캘리브레이션은 매우 작은 숫자에도 민감한 분야이며, 공장에서 제어하지 못하는 수준의 위치 오차는 캘리브레이션에 큰 영향을 미칩니다. 

특히, $c$에서 발생하는 오차로 인해 캘리브레이션 과정에서 발생한 $f$의 오차는 최소 몇 배 이상이라고 알려져 있습니다 [1]. 따라서 $c$ 역시 캘리브레이션 대상에 포함됩니다.

$K$에 속하는 모든 미지수는 "렌즈"와 "수광 센서"에 종속적이며, 카메라를 어떤 장소에 어떻게 위치시켜도 불변하는 요소들입니다.

### Extrinsic Calibration

외부 파라미터는 카메라와 카메라 외부 세상 사이의 관계를 나타냅니다. 구체적으로, 실제 세계의 3차원 직교 좌표계와 카메라 좌표계(이 역시 3차원 직교 좌표계) 사이의 관계를 나타냅니다. 

서로 다른 직교 좌표계가 있을 경우 둘 사이의 관계를 나타내는 것은 단 두 가지입니다. 기저의 방향 차이와 원점의 위치 차이입니다. 이는 간단하게 회전 행렬 $R$과 위치 벡터 $T$로 표현할 수 있습니다.

실제 좌표계 상에 위치한 점 $M=(X,Y,Z)$가 있을 때, 다음과 같은 변환을 통해 카메라 상의 좌표계로 표현할 수 있습니다.
$$
M^C=RX+T
$$
주의할 것은, 좌표계가 바뀌기 때문에 이에 맞춰 **표현이 변하는 것이지 점은 그대로라는 것입니다.**



이렇게 카메라 좌표계로 점을 표현하면 상을 구하고 바로 이미지로 투영할 수 있습니다. 픽셀상의 좌표를 $m$이라고 하면
$$
\lambda m=KM^C=K(RX+T)
$$
좀 더 구체적으로 살펴보겠습니다. $m$과 $M$의 끝에 1씩을 붙여 $m=(u,v,1), M=(X,Y,Z,1)$로 표현하겠습니다. 그러면 다음과 같이 표현할 수 있습니다.
$$
\lambda m=K[R|T]X
$$
$[R|T]$는 3 by 3 행렬과 3 by 1 열벡터를 가로로 이어 붙인(concatenate) 것입니다. 행렬의 shape을 계산하다보면 $\lambda m$의 크기의 3 by 1임을 알 수 있습니다. 즉 $\lambda m=(\lambda u, \lambda v, \lambda)$의 좌표를 가지는 동차 좌표계 상의 점입니다. 따라서 $\lambda$로 나누어주면 이미지상의 픽셀 좌표 $m$이 도출됩니다.



skew 관련 계수가 없을 경우 $K$의 자유도는 4이며, $R$는 회전 행렬이므로 자유도가 3입니다. 따라서 캘리브레이션은 전체 자유도가 10인 변수들을 구하는 최적화 과정입니다.



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



## References

[1] Hartley, Richard I., and Robert Kaucic. "Sensitivity of calibration to principal point position." *European Conference on Computer Vision*. Springer, Berlin, Heidelberg, 2002.

[2] Zhang, Zhengyou. "A flexible new technique for camera calibration." *IEEE Transactions on pattern analysis and machine intelligence* 22.11 (2000): 1330-1334.

[3] Mitchelson, Joel, and Adrian Hilton. "Wand-based multiple camera studio calibration." *Center Vision, Speech and Signal Process* (2003).

