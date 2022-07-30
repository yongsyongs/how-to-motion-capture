# Implmentation of Motion Capture

**물리적인 촬영 과정을 제외한** 모션 캡쳐 과정의 파이썬 구현입니다.

`implementation/data/' 위치에서 'download.py'를 실행하면 작성자가 직접 촬영한 영상들이 다운로드됩니다.

*해당 데이터들의 저작권은 작성자(나용수, yongsyongs)에게 있습니다.*

`src` 폴더의 코드는 다음과 같이 구성되어 있습니다.

### 영상 처리
- `0.0_binarize_video.py`: 영상을 이진화합니다.
- `0.1_detect_marker.py`: 이진화된 영상에서 촬영된 마커 좌표를 추출합니다.
- `0.2_identify_marker.py`: 캘리브레이션을 위해 마커를 식별합니다.
### 캘리브레이션
- `1.0_calibrate_intr.py`: 식별된 마커로부터 내부 파라미터를 계산합니다.
- `1.1_calibrate_stereo.py`: 두 카메라에 동시에 촬영된 마커로부터 상대적 외부 파라미터를 계산합니다.
- `1.2_join_params.py`: 여러 파일에 따로 저장된 파라미터들을 한 파일에 모아 저장합니다.
### 복원(Reconstruction)
- `2.0_synchronize.py`: 모션을 캡쳐할 액션씬의 마커 데이터들을 한 파일에 모아 저장합니다.
- `2.1_make_point_cloud.py`: 스테레오 환경에서 식별된 모든 마커들을 복원하여 포인트 클라우드를 생성합니다.
- `visualize_point_cloud.py`: 생성된 포인트 클라우드를 시각화합니다. -> 현재 데이터셋의 낮은 퀄리티로 인해 불완전한 결과가 나타남
- TODO: 추가 촬영 및 Spatial & Temporal Correspondence -> Reconstruction 코드 추가


## 사용법
```
$ cd /implementation/data
$ python download.py

$ cd ../src
$ python 0.0_binarize_video.py ../data/video/calib/01.avi ../data/video/calib/bin/01.avi
$ python 0.0_binarize_video.py ../data/video/calib/02.avi ../data/video/calib/bin/02.avi
$ python 0.0_binarize_video.py ../data/video/calib/03.avi ../data/video/calib/bin/03.avi
$ python 0.0_binarize_video.py ../data/video/act/01.avi ../data/video/act/bin/01.avi
$ python 0.0_binarize_video.py ../data/video/act/02.avi ../data/video/act/bin/02.avi
$ python 0.0_binarize_video.py ../data/video/act/03.avi ../data/video/act/bin/03.avi

$ python 0.1_detect_marker.py ../data/video/calib/bin/01.avi ../data/marker/calib/01.npy
$ python 0.1_detect_marker.py ../data/video/calib/bin/02.avi ../data/marker/calib/02.npy
$ python 0.1_detect_marker.py ../data/video/calib/bin/03.avi ../data/marker/calib/03.npy
$ python 0.1_detect_marker.py ../data/video/act/bin/01.avi ../data/marker/act/01.npy
$ python 0.1_detect_marker.py ../data/video/act/bin/02.avi ../data/marker/act/02.npy
$ python 0.1_detect_marker.py ../data/video/act/bin/03.avi ../data/marker/act/03.npy

$ python 0.2_identify_marker.py ../data/marker/calib/01.npy ../data/marker/calib/id/01.npy
$ python 0.2_identify_marker.py ../data/marker/calib/02.npy ../data/marker/calib/id/02.npy
$ python 0.2_identify_marker.py ../data/marker/calib/03.npy ../data/marker/calib/id/03.npy

$ python 1.0_calibrate_intr.py ../data/marker/calib/id/01.npy ../data/params/intr/01.npy
$ python 1.0_calibrate_intr.py ../data/marker/calib/id/02.npy ../data/params/intr/02.npy
$ python 1.0_calibrate_intr.py ../data/marker/calib/id/03.npy ../data/params/intr/03.npy

$ python 1.1_calibrate_stereo.py \
    ../data/marker/calib/id/01.npy \
    ../data/marker/calib/id/02.npy \
    ../data/params/intr/01.npy \
    ../data/params/intr/02.npy \
    ../data/params/extr/01_02.npy
$ python 1.1_calibrate_stereo.py \
    ../data/marker/calib/id/02.npy \
    ../data/marker/calib/id/03.npy \
    ../data/params/intr/02.npy \
    ../data/params/intr/03.npy \
    ../data/params/extr/02_03.npy

$ python 1.2_join_params.py --config-path join-params-config.json
$ python 2.0_synchronize.py --config-path reconstruct-config.json
$ python 2.1_make_point_cloud.py --config-path reconstruct-config.json

$ python visualize_point_cloud.py ../data/viz.avi 240 640 360 --config-path reconstruct-config.json
```