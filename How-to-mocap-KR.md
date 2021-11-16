# 모션캡쳐 하는 방법

작성자(나용수, yongsyongs)는 모종의 이유로 모션캡쳐에 흥미를 가졌습니다. 조사를 해보니 구체적으로 모션 캡쳐를 어떻게 실현할 수 있는지에 대하여 잘 설명된 자료가 많지 않았습니다. 똑똑하지 못한 작성자는 어렵게 조사와 삽질을 반복하며 모션캡쳐 방법론을 어느 정도 이해하였습니다.

본 레포지토리는 이곳에 흘러들어온 당신이 삽질을 조금이나마 덜 하기 바라는 마음에서 모션캡쳐 일련의 과정을 설명합니다.

작성자는 이 자료가 교과서가 아니라 표지판의 역할을 하기 바라기에, 이론적으로 깊게 설명하지 않으려 합니다. 궁금하신 분들은 제시된 키워드를 서치하시어 공부하시길 추천드립니다.



> - 초안은 한국어로 간단하게 작성되었으며, 실행 코드 등은 시간이 지나 추가될 수 있습니다. 혹여 이해가 안되는 부분이 있다면 제 메일로 연락하여 주시기 바랍니다. 보충 설명이 담긴 답장과 함께 본문을 업데이트하도록 하겠습니다.
> - 영어 버전은 준비 중에 있지 않으며, 이에 대해 도움을 주실 분께서는 메일로 연락해주시기 바랍니다.



E-mail: ysn3830@gmail.com



## 재귀 반사 패시브 마커 기반 모션캡쳐

모션캡쳐는 어떤 센서로부터 사람의 동작을 측정하는 것을 말합니다. 일반적으로 센서는 광학 센서 = __카메라__입니다. 

>  IMU 센서에 기반하여 동작을 측정하는 작업이 일반적으로 개발하기 쉽고, 세팅 비용이 쌉니다. 실제로 아두이노만 다룰 줄 알면 되고, 센서값도 상당히 쌉니다. 그러나 측정값의 변화량에 기반한 metric이 산출되므로 오차가 누적되는 현상(drift error)이 발생하여 그리 선호되지는 않습니다. 간단하게 real-time 수준에서 정확도가 중요하지 않은 작업의 경우 IMU 기반 모션 캡쳐를 추천할 만합니다. 

본 문서에서는 광학 기반의 모션 캡쳐를 다룹니다. 정확하게는 __적외선 재귀 반사 Passive Marker__에 기반한 모션캡쳐에 대해서 다룹니다.

이 방식은 아래와 같은 Hardware들이 필요합니다.

- 적외선 카메라(및 카메라로 촬영하기 위한 기타 장비 전반 - 삼각대 등)
- 적외선 광원(LED 등)
- 적외선 재귀 반사 마커



재귀 반사란 빛이 반사될 때 투사된 방향으로 반사됨을 뜻합니다. 

![](https://www.3m.co.kr/wps/wcm/connect/ebf8774b-b704-4556-8a80-df1e2cf1e99d/1194129_410x205_HowRetroWorksRetro.jpg?MOD=AJPERES&CACHEID=ROOTWORKSPACE-ebf8774b-b704-4556-8a80-df1e2cf1e99d-mdE4h.E)

이 재귀 반사 성질을 가진 마커를 활용하면 화면상에서 마커의 위치를 쉽게 탐지할 수 있습니다. 구체적인 방법은 아래와 같습니다.

1. 적외선 카메라의 광각 중심 방향으로 적외선이 조사되도록 합니다
2. 적외선 재귀 반사 마커가 촬영되도록 마커를 위치합니다.
3. 재귀 반사 성질에 의해 광원에서 마커에 도달한 빛은 다시 광원으로 향합니다.
4. (상대적으로) 이 많은 양의 반사된 빛들은 렌즈에 도달하여 카메라가 촬영하게 됩니다.
5. 촬영 결과 적외선 영상에서는 마커 부분이 매우 빛나게 됩니다.



빛은 일반적으로 정반사 혹은 난반사되기에, 광원에서 출발한 빛들 중 적은 양많이 렌즈에 도달합니다. 그러나 재귀 반사가 발생하면 빛이 광원의 위치로 다시 반사되며, 광원과 카메라의 위치가 (거의) 같을 때 렌즈에 도달하는 빛이 매우 많아집니다. 따라서 마커의 위치를 영상에서 상당히 쉽게 찾을 수 있습니다.



마커의 위치를 영상에서 잘 찾아내었다면 이후 캘리브레이션(Calibration) 작업을 진행해야 합니다.



## Calibration

직역하자면 __조정__ 정도의 뜻을 가지는 캘리브레이션은, "기하학적으로 실제 촬영 환경을 모델링한 뒤, 최적화 방법론을 통해 미지수 값을 찾아내는 과정"입니다. 따라서 의외로 모션캡쳐 작업에는 고도의 수학적 이론이 활용되며, 이들을 이해할수록 모션캡쳐 작업이 쉬워집니다.

Camera Geometry는 카메라로 real world를 촬영하는 환경을 기하학적을 모델링하는 학문입니다. 이를 이해하는 데에는 최소한 선형대수학의 지식이 필요하며, 특히 선형대수학의 기하학적 접근을 잘 이해해야 합니다.

Optimization은 주어진 조건과 목적함수가 있을 때, 목적함수를 최소화(혹은 최대화)하는 미지수를 찾는 일련의 과정과 이를 다룬 학문입니다. 최적화는 매우 깊고 넓은 분야이므로 요구되는 수학적 지식이 매우 깊습니다. 다만 meta적으로, 혹은 추상적으로 이를 이해하려면 미적분학 정도의 지식이 요구됩니다.

이후 내용을 읽기 전에 아래 내용을 서치하여 이해해두시길 추천합니다.

- (Camera Geometry) Pinhole 카메라 모델
- (Camera Geometry) 좌표계 변환, 회전 변환

- (Camera Geometry) 동차 좌표계, Epipolar Geometry
- (Optimization) 최소제곱 최적화

OpenCV라는 훌륭한 라이브러리가 있기 때문에 최적화에 대한 이해는 부족해도 모션캡쳐를 구현하는 데에 큰 문제가 없습니다. 다만, 카메라 좌표계와 동차 좌표계에 대한 이해는 __필수__적이니 반드시 이해하시기 바랍니다.



### Intrinsic Calibration

핀홀 카메라 모델에서 Intrinsic Calibration은 미지수 행렬 $K$를 구하는 과정입니다. 이 K는 다음과 같은 형태를 지닙니다.
$$
K=\begin{bmatrix}f_x & 0 & c_x\\0 & f_y & c_y\\0 & 0 & 1\end{bmatrix}
$$
(왜곡 계수에 대해서는 다루지 않습니다.)

카메라가 원점이며, 렌즈가 향하는 방향과 카메라 오른쪽 방향이 각각 $z,x$축인 오른손 (직교) 좌표계를 __카메라 좌표계__라고 합니다. 이 카메라 좌표계 상의 어떤 점 $X=(x,y,z)$는 __동차 좌표계__ 변환으로 인해 $P=({x\over z},{y\over z},{z\over z})=(a,b,1)$가 됩니다. 이는 카메라로부터 거리가 1만큼인 거리에 형성된 __상__입니다. 

$K$는 이 상에 간단한 일차 변환을 적용하여 __이미지 픽셀 좌표상의 점으로 변환__합니다.
$$
KP=\begin{bmatrix}f_x & 0 & c_x\\0 & f_y & c_y\\0 & 0 & 1\end{bmatrix}\begin{bmatrix}a\\b\\1\end{bmatrix}=\begin{bmatrix}f_xa+c_x & f_yb+c_y & 1\end{bmatrix}
$$
이미 동차 좌표계로 변환된 시점에서 $z$축 데이터는 큰 의미가 없습니다. 따라서 $K$에 의한 변환은 카메라로부터 거리 1만큼 떨어진 곳에 형성된 상 $(a,b)$을 이미지 픽셀 좌표 $(u,v)=(f_xa+c_x, f_yb+c_y)$로 바꿔주는 변환이라고 할 수 있습니다. 이 $(u,v)$는 실제 이미지상의 픽셀입니다.

$f$를 초점 길이(focal length)라고 하는데, 일반적으로 이해되는 범용적인 의미의 초점길이가 아니고, 카메라 렌즈의 중심으로부터 수광센서까지의 거리를 말합니다. 이는 카메라 좌표계상에서 이미지 좌표계로의 __scale factor__ 정도로 이해하면 편합니다. 카메라 좌표계의 공간 단위를 이미지 픽셀 공간 단위로 변환시켜주는 것입니다. 일반적으로 이미지 해상도와 비슷한 영역대의 단위를 가집니다. (이미지가 $1000\times1000$ 해상도라면 $f$는 $600$ 또는 $1500$과 같은 값을 가질 수 있습니다.)

$c$는 주점(principal point) 좌표인데, 실제 수광 센서의 중앙과 렌즈의 중앙 사이의 차이를 모델링함과 동시의 이미지에서 원점이 중앙이 아닌 좌상단에 위치하도록 합니다. $c$를 적당히 이미지 중앙 좌표라고 가정하는 게 처음 공부하는 입장에서 타당해 보일 수 있습니다. 그러나 카메라 캘리브레이션은 매우 작은 숫자에도 민감한 분야이며, 공장에서 제어하지 못하는 수준의 위치 오차는 캘리브레이션에 큰 영향을 미칩니다. 

특히, $c$에서 발생하는 오차로 인해 캘리브레이션 과정에서 발생한 $f$의 오차는 최소 몇 배 이상이라고 알려져 있습니다 [1]. 따라서 $c$ 역시 캘리브레이션 대상에 포함됩니다.

$K$에 속하는 모든 미지수는 "렌즈"와 "수광 센서"에 종속적이며, 카메라를 어떤 장소에 어떻게 위치시켜도 불변하는 요소들입니다. 따라서 이를 카메라의 "내부 파라미터"라고 부르며, 내부 파라미터를 찾는 과정을 Intrinsic Calibration이라고 하는 것입니다.



#### 구현 방법

> OpenCV에서는 calibrateCamera라는 매우 쉽고 편한 함수를 제공하므로, 관련 [튜토리얼](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d)을 참고하기 바랍니다. OpenCV는 매우 훌륭한 툴이기에, 괜스레 이론을 이해하겠다고 소중한 인생을 낭비하는 행위를 하지 마십시오. 후술되는 코드를 읽을 바에 OpenCV Docs를 읽는 게 이득일 것입니다. 물론 독자께서 관련 연구실 대학원생이라면 얘기가 달라지겠지만요...

Intrinsic Calibration에 활용되는 알고리즘들은 대부분 [2]에서 제시된 방식에 기반으로, co-planar points들의 real world points와 image points가 주어졌을 때 적용할 수 있습니다. **일반적으로 사용되는 체스보드를 통한 캘리브레이션이 이에 해당합니다.** 지금부터 설명할 코드 역시 [2]에서 제시된 알고리즘을 구현한 예시입니다. 

예시 이미지는 EasyMocap의 [레포지토리](https://github.com/zju3dv/EasyMocap/blob/master/apps/calibration/Readme.md)에 업로드된 예시 이미지를 활용하였습니다. 아래와 같이 다운로드합니다.

```python
import os
import numpy as np
from scipy.optimize import least_squares

def donwload_image():
  url = "https://raw.githubusercontent.com/zju3dv/EasyMocap/master/apps/calibration/assets/intri_sample.png"
  os.system("curl " + url + " > image.png")

donwload_image()
```

체스보드로부터 이미지상의 픽셀 좌표를 찾고, 이에 대응하는 가상의 실제 좌표를 만들겠습니다.

```python
objp = np.zeros((6 * 9, 3), np.float32)
objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) + 1 # zero divsion을 방지하고자 +1
img = cv2.imread('image1.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
assert ret
m = corners # image points
M = objp # real world points
```

Intrinsic Calibration을 위해서는 먼저 real world plane과 image plane 사이의 Homography Matrix를 구해야 합니다. scipy의 `least_squares`를 통해 이를 구하는 함수를 작성합니다.

```python
def estimate_homography(m, M):
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

  res = least_squares(fun, x0, method='lm')
  return res.x.reshape(3, 3), res.fun

H, fun = estimate_homography(m.T, M.T)
```



작성 중.



## References

[1] Hartley, Richard I., and Robert Kaucic. "Sensitivity of calibration to principal point position." *European Conference on Computer Vision*. Springer, Berlin, Heidelberg, 2002.

[2] Zhang, Zhengyou. "A flexible new technique for camera calibration." *IEEE Transactions on pattern analysis and machine intelligence* 22.11 (2000): 1330-1334.

