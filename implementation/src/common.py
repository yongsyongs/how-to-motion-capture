import cv2
import numpy as np
from tqdm import tqdm

# 캘리브레이션 보드의 점 좌표들. 점들이 Co-plannar한 좌표여야 한다.
# 보드를 제작하는 방식에 맞추어 수정해야 한다.
POSITIONS = np.array(
    [
        [0, 0],
        [1, 0],
        [2, 0],
        [0, 1],
        [1, 1],
        [2, 1],
        [0, 2],
        [1, 2],
        [2, 2],
        [4, 2.5],
    ]
)


class InvalidArgumentError(ValueError):
    pass


def video_apply(video_path, func, description=None, *args, **kwargs):
    """영상의 매 프레임에 함수를 적용한다.

    Parameters
    ----------
    video_path : str
        영상 경로
    func : function
        이미지를 입력 받는 함수
    description : str, optional
        tqdm progress bar에 출력할 문자열, by default None

    Returns
    -------
    results : list
        프레임별 함수 실행 결과
    """

    cap = cv2.VideoCapture(video_path)
    length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    results = [None] * length

    for i in (progress := tqdm(range(length))):
        ret, image = cap.read()
        if not ret or image is None:
            break

        results[i] = func(image, *args, **kwargs)
        if description is not None:
            progress.set_description(description)

    cap.release()
    return results
