"""
이진화된 영상으로부터 마커를 식별한다.

사용 예시
$ python 0.1_detect_marker.py ../data/video/calib/bin/01.avi ../data/marker/calib/01.npy

"""


import os
import cv2
import argparse
import numpy as np
from common import *
from tqdm import tqdm
from itertools import permutations


def find_markers(image: np.ndarray, area_min_size: int = 1) -> np.ndarray:
    """이미지에서 마커를 찾는다.
    신호가 있는 픽셀들이 연속되어 하나의 영역을 이룰 경우 이를 묶어서 하나의 점으로 계산한다.

    Parameters
    ----------
    image : np.ndarray
        대상 이미지
    area_min_size : int, optional
        영역의 최소 넓이. 이 값보다 넓이가 적은 영역은 무시한다. 기본값은 1.

    Returns
    -------
    markers : np.ndarray
        마커 좌표
    """
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = list(contours)
    markers = []
    for c in contours:
        if len(c) >= area_min_size:
            markers.append(c.reshape(-1, 2).mean(axis=0).astype(np.int32))

    return np.array(markers)


def main(video_path, output_path, area_min_size):
    # 완벽히 이진화된 데이터가 아닌 인코딩된 영상에서 데이터를 가져오므로 다시 이진화한다.(잡음 제거)
    func = lambda x: find_markers(
        ((cv2.cvtColor(x, cv2.COLOR_BGR2GRAY) > 200) * 255).astype(np.uint8),
        area_min_size=area_min_size,
    )

    markers = []
    frame_numbers = []

    found_points = video_apply(video_path, func, "Detecting Markers...")

    print(
        "The number of frames that captured markers' count is valid:",
        len(list(filter(lambda x: len(x) > 0, found_points))),
    )

    for i, points in enumerate(found_points):
        if points is None or len(points) == 0:
            continue

        frame_numbers.append(i)
        markers.append(points)

    print("Detected Frame Count:", len(frame_numbers))

    os.makedirs(os.path.split(output_path)[0], exist_ok=True)
    np.save(output_path, {"frame_numbers": frame_numbers, "markers": markers})


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("video_path", type=str)
    parser.add_argument("output_path", type=str)
    parser.add_argument("--area-min-size", type=int, default=3)
    args = parser.parse_args()

    if not os.path.isfile(args.video_path):
        raise InvalidArgumentError("입력 영상 경로가 잘못되었습니다.")
    if args.area_min_size < 1:
        raise InvalidArgumentError("최소 영역 넓이는 1 이상이어야 합니다.")

    main(args.video_path, args.output_path, args.area_min_size)
