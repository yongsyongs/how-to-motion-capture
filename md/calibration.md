## Calibration

캘리브레이션은 **조정**이라는 뜻을 가지고 있습니다만, 카메라 분야에서는 다른 뜻을 가지고 있습니다. 대략적으로 말하자면 "기하학적으로 실제 촬영 환경을 모델링한 뒤, 최적화 방법론을 통해 미지수 값을 찾아내는 과정"입니다. 따라서 모션캡쳐 작업에는 꽤 어려운 수학적 이론이 활용되며, 이들을 이해해야 모션캡쳐가 가능합니다.

**Camera Geometry**는 카메라로 real world를 촬영하는 환경을 기하학적으로 모델링하는 학문입니다. 이를 이해하는 데에는 최소한 선형대수학의 지식이 필요하며, 특히 선형대수학의 기하학적 접근을 잘 이해해야 합니다.

Optimization은 주어진 조건과 목적함수가 있을 때, 목적함수를 최소화(혹은 최대화)하는 미지수를 찾는 일련의 과정과 이를 다룬 학문입니다. 최적화는 제대로 파고 들어가면 매우 깊고 넓은 분야이므로 상당히 어려운 학문입니다. 다만 캘리브레이션 분야에서 쓰이는 내용 한정으로 미적분학과 선형대수학 정도의 지식만이 요구됩니다.

이후 내용을 읽기 전에 아래 내용을 서치하여 이해해두시길 추천합니다.

- (Camera Geometry) Pinhole 카메라 모델
- (Camera Geometry) 좌표계 변환, 회전 변환, 호모그라피 행렬

- (Camera Geometry) 동차 좌표계, Epipolar Geometry
- (Optimization) 최소제곱 최적화, RANSAC

OpenCV라는 훌륭한 라이브러리가 있기 때문에 최적화에 대한 이해는 부족해도 모션캡쳐를 구현하는 데에 큰 문제가 없습니다. 다만, 카메라 좌표계와 동차 좌표계 등 Camera Geometry에 대한 이해는 __필수__적이니 반드시 이해하시기 바랍니다.

필자는 [다크프로그래머 블로그](https://darkpgmr.tistory.com/32)에서 관련 내용을 처음으로 공부했습니다. 해당 블로그가 잘 설명되어 있어 입문하기에 좋았습니다.



### Intrinsic Parameters

Intrinsic Calibration은 내부 파라미터(Intrinsic Parameter)를 구합니다. 카메라의 내부 파라미터는 렌즈에 의해 맺히는 상과 실제 이미지 사이의 관계를 나타냅니다.

핀홀 카메라 모델에서 Intrinsic Calibration은 다음과 같은 Intrinsic Parameter Matrix $K$를 구합니다. 

$$ K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} $$

(왜곡 계수에 대해서는 다루지 않습니다.)

카메라가 원점이며, 렌즈가 향하는 방향과 카메라 오른쪽 방향이 각각 $z,x$축인 오른손 (직교) 좌표계를 **카메라 좌표계**라고 합니다. 이 카메라 좌표계 상의 어떤 점 $X=(x,y,z)$는 **동차 좌표계** 변환으로 인해 $P=({x\over z},{y\over z},{z\over z})=(a,b,1)$가 됩니다. 이는 카메라로부터 거리가 1만큼인 거리에 형성된 **상**입니다. 

$K$는 이 상에 간단한 일차 변환을 적용하여 **이미지 픽셀 좌표상의 점**으로 변환합니다.

$KP=\begin{bmatrix}f_x & 0 & c_x\\0 & f_y & c_y\\0 & 0 & 1\end{bmatrix}\begin{bmatrix}a\\b\\1\end{bmatrix}=\begin{bmatrix}f_xa+c_x & f_yb+c_y & 1\end{bmatrix}$

이미 동차 좌표계로 변환된 시점에서 $z$축 데이터는 큰 의미가 없습니다. 따라서 $K$에 의한 변환은 카메라로부터 거리 1만큼 떨어진 곳에 형성된 상 $(a,b)$을 이미지 픽셀 좌표 $(u,v)=(f_xa+c_x, f_yb+c_y)$로 바꿔주는 변환이라고 할 수 있습니다. 이 $(u,v)$는 실제 이미지상의 픽셀입니다.

$f$를 초점 길이(focal length)라고 하는데, 일반적으로 이해되는 범용적인 의미의 초점길이가 아니고, 카메라 렌즈의 중심으로부터 수광센서까지의 거리를 말합니다. 이는 카메라 좌표계상에서 이미지 좌표계로의 **scale factor** 정도로 이해하면 편합니다. 카메라 좌표계의 공간 단위를 이미지 픽셀 공간 단위로 변환시켜주는 것입니다. 일반적으로 이미지 해상도와 비슷한 영역대의 단위를 가집니다. (이미지가 $1000\times1000$ 해상도라면 $f$는 $600$ 또는 $1500$과 같은 값을 가질 수 있습니다.)

$c$는 주점(principal point) 좌표인데, 실제 수광 센서의 중앙과 렌즈의 중앙 사이의 차이를 모델링함과 동시의 이미지에서 원점이 중앙이 아닌 좌상단에 위치하도록 합니다. $c$를 적당히 이미지 중앙 좌표라고 가정하는 게 처음 공부하는 입장에서 타당해 보일 수 있습니다. 그러나 카메라 캘리브레이션은 매우 작은 숫자에도 민감한 분야이며, 공장에서 제어하지 못하는 수준의 위치 오차는 캘리브레이션에 큰 영향을 미칩니다. 

특히, $c$에서 발생하는 오차로 인해 캘리브레이션 과정에서 발생한 $f$의 오차는 최소 몇 배 이상이라고 알려져 있습니다 [1]. 따라서 $c$ 역시 캘리브레이션 대상에 포함됩니다.

$K$에 속하는 모든 미지수는 "렌즈"와 "수광 센서"에 종속적이며, 카메라를 어떤 장소에 어떻게 위치시켜도 불변하는 요소들입니다.

### Extrinsic Parameters

외부 파라미터는 카메라와 카메라 외부 세상 사이의 관계를 나타냅니다. 구체적으로, 실제 세계의 3차원 직교 좌표계와 카메라 좌표계(이 역시 3차원 직교 좌표계) 사이의 관계를 나타냅니다. 

서로 다른 직교 좌표계가 있을 경우 둘 사이의 관계를 나타내는 것은 단 두 가지입니다. 기저의 방향 차이와 원점의 위치 차이입니다. 이는 간단하게 회전 행렬 $R$과 위치 벡터 $T$로 표현할 수 있습니다.

실제 좌표계 상에 위치한 점 $M=(X,Y,Z)^T$가 있을 때, 다음과 같은 변환을 통해 카메라 상의 좌표계로 표현할 수 있습니다.

$M^C=RM+T$

주의할 것은, 좌표계가 바뀌기 때문에 이에 맞춰 **표현이 변하는 것이지 점은 그대로라는 것입니다.**



이렇게 카메라 좌표계로 점을 표현하면 상을 구하고 바로 이미지로 투영할 수 있습니다. 픽셀상의 좌표를 $m$이라고 하면

$\lambda m=KM^C=K(RM+T)$

좀 더 구체적으로 살펴보겠습니다. $m$과 $M$의 끝에 1씩을 붙여 $m=(u,v,1)^T, M=(X,Y,Z,1)^T$로 열벡터를 재정의하겠습니다. 그러면 다음과 같이 표현할 수 있습니다.

$\lambda m=K[R|T]M$

$[R|T]$는 3 by 3 행렬과 3 by 1 열벡터를 가로로 이어 붙인(concatenate) 것입니다. 행렬의 shape을 계산하다보면 $\lambda m$의 크기의 3 by 1임을 알 수 있습니다. 즉 $\lambda m=(\lambda u, \lambda v, \lambda)^T$의 좌표를 가지는 동차 좌표계 상의 점입니다. 따라서 $\lambda$로 나누어주면 이미지상의 픽셀 좌표 $m$이 도출됩니다.



skew 관련 계수가 없을 경우 $K$의 자유도는 4이며, $R$는 회전 행렬이므로 자유도가 3입니다. 따라서 캘리브레이션은 전체 자유도가 10인 변수들을 구하는 최적화 과정입니다.



### Basic Calibration Technique

캘리브레이션 과정의 구체적인 진행을 서술합니다. 본문에서는 [2]에서 소개된 방법을 설명합니다. Co-Planar한(한 평면 위에 있는) 점들을 촬영했을 때 캘리브레이션하는 방법을 제시하는데, OpenCV 예제로 잘 알려진 체스보드가 대표적인 그 예시입니다.



먼저 Real World 상에 존재하는 평면의 3차원 점들을 $\mathbf M$, 이를 촬영하여 이미지로 투영된 2차원 픽셀 좌표를 $\mathbf m$이라 합시다. $N$개의 점이 있을 때 이들은 각각 $3\times N, 2\times N$의 크기를 같습니다.
$\mathbf M=\begin{pmatrix}
x_1&x_2&...&x_N\\
y_1&y_2&...&y_N\\
1&1&...&1\\
\end{pmatrix}
\\
\\
\mathbf m=\begin{pmatrix}
u_1&u_2&...&u_N\\
v_1&v_2&...&v_N\\
\end{pmatrix}
$

$\mathbf M$의 세 번째 행이 모두 1인 이유는 Co-Planar한 점들이라 가정했기 때문입니다. 따라서 해당 평면이 $xy$평면과 나란하다고 가정하면 1 이외의 어떤 상수로 설정해도 상관 없습니다. $z$값이 모두 같다는 게 중요합니다. 실제로 그러한 점들을 촬영해야만 합니다.

##### Calculate Homography Matrix

위 점들로부터 다음과 같은 행렬 $L$을 구성합니다. $L$의 크기는 $2N\times9$입니다.
$L=\begin{pmatrix}
x_1&y_1&1&0&0&0&-u_1x_1&-u_1y_1&-u_1z_1\\
0&0&0&x_1&y_1&1&-v_1x_1&-v_1y_1&-v_1z_1\\
...\\
x_N&y_N&1&0&0&0&-u_Nx_N&-u_Ny_N&-u_Nz_N\\
0&0&0&x_N&y_N&1&-v_Nx_N&-v_Ny_N&-v_Nz_N\\
\end{pmatrix}
$

크기가 $9\times9$인 $L^\intercal L$ 행렬의 Eigen Values와 Eigen Vectors를 구했을 때, 가장 작은 Eigen Value에 대응되는 Eigen Vector가 호모그라피 행렬 $H$의 근사치입니다. 이 벡터는 길이가 9인데, 이를 $3\times3$ 크기의 행렬로 바꾸면 호모그라피 행렬이 됩니다.(1~3/4~6/7~9 번째 원소가 1/2/3 번째 행이 됩니다.)

최적의 값이라기보다는 적절한 근사치라고 할 수 있으므로, 이를 초기값으로 삼아 최적화하는 과정이 필요합니다. 변수는 $H$로 설정하고, 주어진 $H$에 대해 $\lambda\mathbf{\hat m}=H\mathbf M$을 구합니다. 그러면 목적함수를 다음과 같이 설정할 수 있습니다.
$F(H, \mathbf M, \mathbf m)=|\mathbf m-\mathbf{\hat m}/\lambda|$
단일 점이 아니라 여러개의 점이므로 $\lambda$는 스칼라가 아닌 벡터 혹은 행렬입니다. 개별 점마다 다른 값을 가질 수 있습니다.



최적화 과정은 Least Square Optimization이며, **Levenberg-Marquardt Algorithm**[4]을 사용할 수 있습니다. 최적화 과정에 대해서는 자세히 서술하지 않습니다. 구체적이 구현이 궁금한 경우 Implementation을 참조하십시오.



##### Calculate Intrinsic Parameters

이제 적절한 $H^*$값을 얻었으면, 이로부터 내부 파라미터 $K$를 도출해야 합니다. 계산 과정이 복잡하고, 그 증명이나 이유를 서술하기엔 본문의 범위를 벗어나므로 대략적인 과정만 나열합니다. 따라서 다음의 계산 과정은 읽지 않아도 됩니다. 코드 구현은 Implementation에 있으며, 구체적인 과정과 수식 도출의 이유가 궁금하신 분은 [2]를 참조하십시오.

행렬 $B$를 $B=K^\intercal K^{-1}$로 정의합니다. 그 $i$ 번째 행의 $j$번째 원소를 $B_{ij}$라 하고, 다음과 같이 벡터 $\mathbf b$를 정의합니다.

$\mathbf b=\begin{bmatrix}B_{11}&B_{12}&B_{22}&B_{13}&B_{23}&B_{33}\end{bmatrix}^\intercal$

호모그라피 행렬 $H$의 $i$번째 열벡터를 $\mathbf h_i=\begin{bmatrix}h_{i1}&h_{i2}&h_{i3}\end{bmatrix}^\intercal$로 둡니다. 그러면 다음의 식이 성립합니다.

$\mathbf h_i^\intercal B\mathbf h_j=\mathbf v_{ij}^\intercal\mathbf b\\
\text{with }\mathbf v_{ij}=\begin{bmatrix}h_{i1}h_{j1},&h_{i1}h_{j2}+h_{i2}h_{j1},&h_{i2}h_{j2},&h_{i3}h_{j1}+h_{i1}h_{j3},&h_{i3}h_{j2}+h_{i2}h_{j3},&h_{i3}h_{j3}\end{bmatrix}^\intercal$

$\mathbf h_1^\intercal K^{-\intercal}K^{-1}\mathbf h_2=0$이고 $\mathbf h_1^\intercal K^{-\intercal}K^{-1}\mathbf h_1=\mathbf h_2^\intercal K^{-\intercal}K^{-1}\mathbf h_2$이므로, 다음이 성립합니다.

$\begin{bmatrix}\mathbf v_{12}^\intercal\\(\mathbf v_{11}-\mathbf v_{22})^\intercal\end{bmatrix}\mathbf b=0$

$n$개의 이미지로부터 각각의 호모그라피 행렬이 계산되므로, 위 식의 왼쪽 첫 항 행렬을 $n$번 쌓아 $\mathbf V$라고 표기하면 $\mathbf V\mathbf b=0$입니다.

$L$의 경우와 마찬가지로 $\mathbf V^\intercal\mathbf V$에서 가장 작은 Eigen Value에 대응하는 Eigen Vector가 $\mathbf b$가 됩니다.

$\mathbf b$를 구했을 때 $K$는 다음과 같이 구합니다.

$c_y=(B_{12}B_{13}-B_{11}B_{23})/(B_{11}B_{22}-B_{12}^2)\\
\lambda=B_{33}-[B_{13}^2+c_y(B_{12}B_{13}-B_{11}B_{23})]/B_{11}\\
f_x=\sqrt{\lambda/B_{11}}\\
f_y=\sqrt{\lambda B_{11}/(B_{11}B_{22}-B_{12}^2)}\\
\gamma=-B_{12}f_x^2f_y/\lambda\\
c_x=\gamma c_x/f_y-B_{13}f_x^2/\lambda\\
\\
K=\begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}$

##### Calculate Extrinsic Parameters

호모그라피 행렬 $H$와 카메라 내부 파라미터 $K$가 있을 때 다음과 같이 외부 파라미터 $R=\begin{bmatrix}\mathbf r_1&\mathbf r_2&\mathbf r_3\end{bmatrix}, T=\mathbf t$를 구할 수 있습니다.

$\mathbf r_1=\lambda K^{-1}\mathbf h_1\\
\mathbf r_2=\lambda K^{-1}\mathbf h_2\\
\mathbf r_3=\mathbf r_1\times\mathbf r_2\\
\mathbf t=\lambda K^{-1}\mathbf h_3\\
\text{with } \lambda=1/||K^{-1}\mathbf h_1||=1/||K^{-1}\mathbf h_2||$

##### Optimize

위와 같이 구한 $K$값을 초기값으로 사용하는 최적화를 실행할 수 있습니다. 앞서서는 Levenberg-Marquardt Algorithm을 사용했는데, RANSAC을 이용할 수도 있습니다.  RANSAC은 갑자기 데이터 값이 튀는 경우가 많은 영상 처리나 신호 처리에서 유용하게 사용되는 알고리즘입니다.

영상이 촬영되어 총 $T$개의 이미지 프레임이 있고, 개별 프레임마다 호모그라피 행렬을 계산합니다. $T$개의 호모그라피 행렬 중 임의로 $s$개를 선택하여 상기한 **Calculate Intrinsic Parameters & Calculate Intrinsic Parameters** 챕터의 방식으로 파라미터들을 구합니다. 이 과정을 여러번 반복하면 반복 횟수만큼 파라미터 후보군들이 생깁니다. 개별 파라미터마다 전체 점들의 Reprojection Error를 계산하여 inlier의 개수가 가장 많은 파라미터 후보를 최종 파라미터로 설정합니다. 자세한 이론은 [5]를, 구현은 Implementation을 참조하십시오.



## Calibration for Motion Capture Studio

단순히 캘리브레이션 알고리즘을 작성한다고 모션캡쳐를 진행할 수 있는 것은 아닙니다. 구체적으로 모션캡쳐 촬영을 위해 세팅해야하는 것들이 있고, 캘리브레이션 과정에도 변화가 필요합니다.

모션캡쳐 환경은 중앙의 피사체(사람)을 원형으로 둘러싸면서 렌즈가 모두 중앙으로 향하도록 합니다.

TODO: 사진

카메라의 수가 많을수록 모션캡쳐 과정이 길어지며, 정확도가 증가합니다.

단순히 개별 카메라를 통해 chess board와 같은 피사체를 촬영하여 각각의 내부 파라미터를 구할 수는 있습니다. 그러나 외부 파라미터는 모든 카메라가 종속된 하나의 공간에 대해서 구해야 합니다.

따라서 방향이 구분되지 않는 chess board보다는, 방향이 구분되면서 planar한 물체를 calibration의 피사체로 활용해야 합니다.

아래 영상은 그 예시 중 하나입니다.

<iframe width="560" height="315" src="https://www.youtube.com/embed/6ysHjP5t5_E" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

위 피사체의 경우 마커의 위치를 적절히 조절하여 어느 방향에서 촬영하든 각각의 마커를 **식별**할 수 있습니다.(마커를 가리키는 사각형 테두리의 색이 고유합니다.)

앞으로의 캘리브레이션의 예제는 아래와 같은 데이터를 활용합니다. 아래의 영상은 작성자가 자체적으로 제작한 planar board이며, 환경은 (640, 360) 해상도의 카메라 8대를 원형으로 배치하였습니다.

<iframe width="560" height="315" src="https://www.youtube.com/embed/Xmrd3r8AMPg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

적절히 크게 잘 배치된 캘리브레이션용 피사체를 한 번 촬영하여도 되지만, 최적화 과정에서 더 정확한 근사해를 구하고자 영상으로 찍었습니다. 어떤 카메라에서 보드를 잘 인식했다면 해당 데이터로 Intrinisic Calibration을 진행하며, 두 카메라에 동시에 촬영되었다면 이로부터 두 카메라의 상대적 관계를 구할 수 있습니다. $i$번 카메라의 좌표계로부터 $j$번 카메라의 좌표계로 변환하는 파라미터들을 $R_{ij}, T_{ij}$라 하면, 4번 카메라의 외부 파라미터는 $R_4=R_{34}R_{23}R_{12}R_1$, $T_4=R_{23}T_{34}+R_{12}T_{23}+R_1T_{12}+T_1$이 됩니다. 

여기서 $R_1=I, T_1=\bold 0$으로 설정하면 1번 카메라의 좌표계와 실제 좌표계가 일치하게 되면서 계산이 편해집니다. 모든 과정이 끝난 뒤 단순한 후처리를 통해 좌표계를 보정하여 실제 좌표계의 데이터로 복원할 수 있습니다.



개별 카메라의 내부 파라미터를 먼저 계산하여 $K_1,...,K_8$을 구했다고 가정합시다. 카메라 $i,j$에 피사체가 동시에 촬영된 프레임이 $N$개 있을 때, 두 카메라로부터 $R, T$가 각각 $N$개씩 계산됩니다. 이를 $R_{ik}, T_{ik}(k=1,...,N)$라 표현합시다. 따라서 $R_{ij}$는 $R_{ij}=R_{jk}R_{ik}^{-1}$를 만족합니다. $i$ 카메라 좌표계에서 월드 좌표계로 변환, 다시 $j$ 카메라 좌표계로 변환하는 과정입니다. 이를 $T$와 함께 $N$개의 프레임에 대하여 최적화를 진행할 수 있습니다. 



## References

[1] Hartley, Richard I., and Robert Kaucic. "Sensitivity of calibration to principal point position." *European Conference on Computer Vision*. Springer, Berlin, Heidelberg, 2002.

[2] Zhang, Zhengyou. "A flexible new technique for camera calibration." *IEEE Transactions on pattern analysis and machine intelligence* 22.11 (2000): 1330-1334.

[3] Mitchelson, Joel, and Adrian Hilton. "Wand-based multiple camera studio calibration." *Center Vision, Speech and Signal Process* (2003).

[4] Gavin, Henri P. "The Levenberg-Marquardt algorithm for nonlinear least squares curve-fitting problems." *Department of Civil and Environmental Engineering, Duke University* (2019): 1-19.

[5] Fischler, Martin A., and Robert C. Bolles. "Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography." *Communications of the ACM* 24.6 (1981): 381-395.
