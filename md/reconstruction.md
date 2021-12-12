## Reconstruction

캘리브레이션 과정을 통해 모든 카메라의 Intrinsic/Extrinsic Parameters를 구했다면, 촬영된 영상으로부터 실제 3D Points의 좌표를 구하는 일만 남았습니다. 막상 생각하면 간단할 것 같은데, 은근히 수고스럽습니다. 본문의 과정은 대부분 [1]을 참조하였습니다.

### Linear Triangulation

먼저 2차원 점으로부터 3차원 점을 복원하는 방법에 대해 이해할 필요가 있습니다. 잘 알려져 있는 방법 중 하나는 **선형 삼각측량법**입니다.

캘리브레이션 과정을 통해 어떤 카메라의 $K, R, T$를 모두 구했다고 합시다. Projection Matrix $P=K[R|T]$와 2차원 픽셀 좌표 점 $\mathbf m$, 3차원 실제 좌표 점 $\mathbf M$ 사이에는 $\mathbf m=P\mathbf M$이라는 간단한 관계성이 생깁니다. 그러나 불행하게도 $P^{-1}\mathbf m=\mathbf M$이라는 식으로 바로 $\mathbf M$을 구하지 못합니다. 사영 과정에서 동차 좌표계가 사용되기 때문입니다. 따라서 동차 좌표계 상에서의 점만을 구할 수 있고, 이는 실제 좌표계에서는 카메라의 중심과 $\mathbf M$을 잇는 직선이 됩니다.

따라서 복수의 카메라와 그 파라미터들이 필요합니다. 간단하게 생각해서, 두 카메라의 중심 $C_1, C_2$이 있을 때 이 두 점과 $\mathbf M$을 잇는 두 직선이 있다고 할 때, $\mathbf M$은 두 직선의 교점입니다. 

위와 같이 직선의 교점으로 구할 수 있다면 참 편하겠으나, 캘리브레이션은 최적화 과정이므로 완벽하게 $C_i$와 $\mathbf M$를 잇는 직선을 구할 수는 없습니다. 따라서 여러가지 카메라의 Projection Matrix $P_i$가 주어졌을 때 가장 적절한 $\mathbf M$을 구하는 과정이 필요합니다. 본문에서는 그 도구로 Linear Triangulation을 사용합니다.

$N$개의 카메라에 대한 Projection Matrix $P_i=\begin{bmatrix}p_1&p_2&p_3&p_4\end{bmatrix}$와 점 $M$에 대한 개별 카메라의 관측 (픽셀) 좌표 $\mathbf m_i=(u_i, v_i)$가 있습니다. $p_i$는 길이가 3인 열벡터입니다. 이로부터 다음과 같은 행렬을 구현합니다.
$$
A=\begin{pmatrix}
u_1p_3^\intercal-p_1^\intercal\\v_1p_3^\intercal-p_2^\intercal\\
u_2p_3^\intercal-p_1^\intercal\\v_2p_3^\intercal-p_2^\intercal\\
...\\
u_Np_3^\intercal-p_1^\intercal\\v_Np_3^\intercal-p_2^\intercal\\
\end{pmatrix}
$$
행렬 $A$는 행벡터들의 stack으로 구해지며, 그 사이즈는 $N\times4$입니다. 이 행렬로 특이값 분해(SVD)를 진행합니다.
$$
A=U\Sigma V^\intercal
$$
$V^\intercal$ 행렬은 $4\times4$ 크기이고, 이 행렬의 마지막 행 벡터 $\mathbf v_4$로 3차원 점 $\mathbf M$의 추정치를 구할 수 있습니다. $\mathbf v_4$의 4번째 원소로 1~3번째 원소를 나누면 그 3개의 원소가 순서대로 $\mathbf M$의 $xyz$좌표입니다.

이를 코드로 구현하면 아래와 같습니다. 아래 코드는 하나의 3차원 점 $\mathbf M$이 아니라 여러개의 점을 동시에 복원할 수 있는 코드입니다.

```python
import numpy as np

def linear_triangulation(points, Pmats):
  	# points: (n_cameras, n_points, 2)
    # Pmats: (n_cameras, 3, 4)
    n_cameras = Pmats.shape[0]
    n_points = points.shape[1]

    A = []
    for i in range(n_cameras):
        # x: (n_points, 2)   | x[:, ?]: (n_points,)
        # P: (3, 4)          | P[?, :]: (4,)
      	x = points[i]
        P = Pmats[i]
        A.append(x[:, 0, np.newaxis] * P[np.newaxis, 2, :] - P[np.newaxis, 0, :] )
        A.append(x[:, 1, np.newaxis] * P[np.newaxis, 2, :] - P[np.newaxis, 1, :] )

    # Calculate best point
    A = np.array(A) # (n_cameras, n_points, 4)
    A = A.transpose(1, 0, 2) # (n_points, n_cameras, 4)
    U, D, Vt = np.linalg.svd(A) # Singular Value Decomposition
    X = Vt[:, -1, 0:3] / Vt[:, -1, 3, np.newaxis] # normalize
    return X # (n_points, 3)
```



### Marker Labeling

리컨스트럭션 과정에서 해결해야할 문제 중 가장 중요하고 어려운 것은 **Marker Labeling** 혹은 **Points Cloud Matching** 등으로 불리는 작업입니다. 심지어 2D Marker Labeling 과정과 3D Marker Labeling 과정이 별도로 존재합니다. 방법론 및 알고리즘에 따라 두 문제를 동시에 해결할 수도 있습니다. 최근에는 ML 모델을 적용한 Auto-Labeling 등이 연구되고 있습니다.

#### 2D Marker Labeling

2D Marker Labeling은 개별 카메라에서 인식된 점들을 매칭하는 것입니다.

$N$개의 카메라와 $M$개의 마커가 있다고 가정하겠습니다. 총 $T$개의 이미지 시퀀스(=비디오)에서 $i-\text{th}$ 프레임의 데이터는 총 이미지 $N$장입니다. 각각의 이미지에는 $p_j\le M(j=1, ... ,N)$개의 마커가 인식됩니다. ($M$개보다 작거나 같은 이유는 시야각과 자세로 인한 Occlusion Issue 혹은 광량 부족으로 인해 카메라에서 측정된 마커의 빛이 Threshold를 달성하지 못했기 때문입니다.)

$j$ 번째 카메라에서 인식된 2D marker들 $\{\mathbf m_{jk}\}(k=1,...,p_j)$가 있을 때, 이들은 마땅한 정렬 기준이 없습니다. 따라서 1번 카메라에서 촬영되어 인식된 첫 번째 점 $\mathbf m_{11}$과 2번 카메라에서 촬영되어 인식된 첫 번째 점 $\mathbf m_{21}$이 서로 같은 점인지 아닌지 알 수 없습니다. 따라서 $\mathbf m_{11}$과 $\mathbf m_{21}$이 같은 점인지 확인할 수 있는 방법이 필요합니다.

쉽게 떠올릴 수 있는 방법 중 하나는 두 카메라의 Epipolar Geometry를 활용하는 방법입니다. 카메라 1, 2에는 각각 $p_1, p_2$개의 마커가 촬영됩니다. 따라서 카메라 2에는 $p_1$개의 epipolar plane을 그릴 수 있으며, 반대의 경우에도 같습니다. 캘리브레이션이 잘 되었다는 가정 하에 각각의 epiploar plane에는 정확히 걸치는 점이 하나 이상 존재하게 됩니다. 따라서 2번 카메라에 그려진 $p_1$개의 epiploar plane과 $p_2$개의 점들로부터 **점과 직선 사이의 거리**를 구해 그 거리가 가장 짧은 점들끼리 순서대로 매칭합니다. 이 때 적절한 상한값을 설정할 필요가 있습니다.

슬프게도 상기한 방법이 실제 상황에서 좋능을 내지 못합니다. 하나의 epiploar plane에 여러 개의 점들이 매칭될 수 있기 때문입니다. 3차원 정보를 2차원에서 다루는 것이기에 당연하게도 정보의 손실이 발생하며, 그 과정에서 완벽한 복원이 어려운 것입니다. 작성자의 경험상 준수한 캘리브레이션 오차를 달성했음에도 불구하고 이 방법으로 정교한 point matching를 달성하기는 어려웠습니다.

[1]에서는 삼각측량법(Linear Triangulation)을 반복적으로 적용하여 최소 오차를 만들어내는 point set을 구합니다. $n$개의 이웃한 카메라에서 임의의 2D points들을 뽑아 Linear Triangulation으로 3D reconstruction을 적용합니다. 같은 마커가 아닐지라도 계산으로부터 어떤 3차원상의 점이 도출되는데, 만약 다른 마커일 경우 이들 좌표에 평균을 적용한 점이 계산됩니다. 따라서 모든 경우의 수에 대해 reconstrunction을 진행해 point cloud를 만든 뒤, 이들을 다시 reproject합니다. 이 때 reprojection error가 낮은 set들이 적절하게 같은 마커로 매칭된 그룹입니다.

좀 더 자세한 예시로, [1]에서는 $n=3$인 경우를 다루었습니다. 서로 이웃한 3개의 카메라 1, 2, 3들로부터 임의의 픽셀 좌표 점 $(\mathbf m_m, \mathbf m_n, \mathbf m_o)$가 있습니다. 그리고 세 카메라에 대응되는 Projection Matrix $P_1, P_2, P_3$가 있습니다. 해당 점들과 이 Projection Matrix로부터 Linear Triangulation을 실행하면 3차원 공간의 점 $M$이 도출됩니다. 따라서 $p_1\times p_2\times p_3$만큼의 triplet을 구성할 수 있는 경우의 수가 생기고, 해당 triplet들 중 reconstruct를 진행했을 때 reprojection error가 가장 적은 triplet을 고르면 해당 2차원 점들들은 모두 동일한 마커라고 말할 수 있다는 겁니다.

이 과정의 결과는 **3D Point Cloud입니다.**

#### 3D Marker Labeling

작성 중.

## References

[1] Guerra-Filho, Gutemberg. "Optical Motion Capture: Theory and Implementation." *RITA* 12.2 (2005): 61-90.