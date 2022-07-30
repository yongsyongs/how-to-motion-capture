"""
이진화된 영상으로부터 마커를 식별한다.

사용 예시
$ python 0.2_identify_marker.py ../data/marker/calib/01.npy ../data/marker/calib/id/01.npy

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


def identify_custom(points: np.ndarray) -> np.ndarray:
    """마커 번호를 식별하는 함수. 상황에 따라 사용자가 수정하여 사용한다.

    Parameters
    ----------
    points : np.ndarray
        식별되지 않은 마커 좌표

    Returns
    -------
    points : np.ndarray
        식별되어 정렬된 마커 좌표
    """

    order = [-1 for _ in range(points.shape[0])]
    far_index = np.argmax(points[..., 0])
    order[9] = far_index
    remain_indices = [i for i in range(points.shape[0]) if i not in order]
    bias = points[remain_indices, 1] - points[remain_indices, 0]
    order[8] = remain_indices[np.argmin(bias)]
    remain_indices = [i for i in range(points.shape[0]) if i not in order]
    bias = points[remain_indices, 1] - points[remain_indices, 0]
    order[0] = remain_indices[np.argmax(bias)]
    remain_indices = [i for i in range(points.shape[0]) if i not in order]
    box_center = (points[order[0]] + points[order[8]]) / 2
    d_from_center = points[remain_indices] - box_center[np.newaxis, ...]
    d_from_center = np.linalg.norm(d_from_center, axis=-1)
    assert d_from_center.shape == (7,), d_from_center.shape
    order[4] = remain_indices[np.argmin(d_from_center)]
    remain_indices = [i for i in range(points.shape[0]) if i not in order]
    bias = points[remain_indices, 1] + points[remain_indices, 0]
    order[6] = remain_indices[np.argmin(bias)]
    order[2] = remain_indices[np.argmax(bias)]
    # now remain 1 3 5 7
    remain_indices = [i for i in range(points.shape[0]) if i not in order]
    order[1] = remain_indices[np.argmax(points[remain_indices, 1])]
    order[7] = remain_indices[np.argmin(points[remain_indices, 1])]
    order[3] = remain_indices[np.argmin(points[remain_indices, 0])]
    order[5] = remain_indices[np.argmax(points[remain_indices, 0])]

    y = points[order]
    val = list()

    val.append(np.linalg.norm((y[0] + y[8]) / 2 - y[4]) < 10)
    val.append(np.linalg.norm((y[0] + y[6]) / 2 - y[3]) < 10)
    val.append(np.linalg.norm((y[0] + y[2]) / 2 - y[1]) < 10)
    val.append(np.linalg.norm((y[1] + y[7]) / 2 - y[4]) < 10)
    val.append(np.linalg.norm((y[2] + y[6]) / 2 - y[4]) < 10)
    val.append(np.linalg.norm((y[2] + y[8]) / 2 - y[5]) < 10)
    val.append(np.linalg.norm((y[3] + y[5]) / 2 - y[4]) < 10)
    val.append(np.linalg.norm((y[6] + y[8]) / 2 - y[7]) < 10)
    d = np.stack(
        [np.linalg.norm(y - y[i][np.newaxis, ...], axis=-1) for i in range(y.shape[0])]
    )
    assert d.shape == (10, 10), d.shape
    val.append(np.logical_not((np.sum(np.isclose(d, 0), axis=-1) > 1).any()))
    val = np.array(val)

    return points[order] if val.all() else None


def main(data_path, output_path):
    data = np.load(data_path, allow_pickle=True).tolist()
    markers = data["markers"]
    frame_numbers = data["frame_numbers"]

    # common에 정의된 캘리브레이션 보드의 마커 정보를 가져옴
    MARKER_POSITIONS = POSITIONS
    marker_count = MARKER_POSITIONS.shape[0]

    print("Marker Count:", marker_count)

    valid_frame_numbers = []
    valid_markers = []

    progress = tqdm(enumerate(markers), total=len(markers))

    for i, points in progress:
        progress.set_description("Identifying Markers...")
        if points is None or points.shape != MARKER_POSITIONS.shape:
            continue

        # 식별에 성공한 경우 해당 프레임 번호와 마커 좌표 저장
        if (points := identify_custom(points)) is not None:
            valid_frame_numbers.append(frame_numbers[i])
            valid_markers.append(points)

    valid_frame_numbers = np.array(valid_frame_numbers)
    valid_markers = np.array(valid_markers)

    print("Detected Valid Frame Count:", len(valid_frame_numbers))

    os.makedirs(os.path.split(output_path)[0], exist_ok=True)
    np.save(
        output_path,
        {"frame_numbers": valid_frame_numbers, "markers": valid_markers},
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_path", type=str)
    parser.add_argument("output_path", type=str)
    args = parser.parse_args()

    if not os.path.isfile(args.input_path):
        raise InvalidArgumentError("입력 파일 경로가 잘못되었습니다.")

    main(args.input_path, args.output_path)
