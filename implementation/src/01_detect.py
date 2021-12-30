"""
영상에서 마커를 찾고 식별한다.

1. 배경 이미지를 계산한다.
2. 영상에서 배경을 제거한다.
3. 마커를 찾는다.
4. 마커를 식별한다.

사용 예시
$ python 01_detect.py ../data/video/wand/03.avi ../data/results/markers/wand_03 --area-min-size 3 --identification custom --remove-background

"""


import os
import cv2
import argparse
import numpy as np
from tqdm import tqdm
from itertools import permutations


class InvalidArgumentError(ValueError):
    pass


POSITIONS = np.array(
    [
        [0, 0],
        [0, 1],
        [0, 3],
    ]
)  # (N, 2)
if len(POSITIONS.shape) == 2:
    np.random.seed(42)
    t = np.random.uniform(1e-1, np.pi / 2 - 1e-1)
    R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    POSITIONS = (R @ POSITIONS.T).T
    N = POSITIONS.shape[0]
    L = N * (N - 1) // 2
    VECS = POSITIONS.reshape(1, -1, 2) - POSITIONS.reshape(-1, 1, 2)  # (N, N, 2)
    VECS = VECS[np.triu_indices(N, 1)]  # (L, 2)


def video_apply(video_path, func, description=None):
    """영상의 매 프레임에 함수를 적용한다.

    Parameters
    ----------
    video_path : str
        영상 경로
    func : functoin
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

    for i in (progress := tqdm.tqdm(range(length))):
        ret, image = cap.read()
        if not ret or image is None:
            break

        results[i] = func(image)
        if description is not None:
            progress.set_description(description)

    cap.release()
    return results


def binary(x: np.ndarray, t=0.5) -> np.ndarray:
    """이미지를 이진화한다.
    모든 픽실의 값이 구간 [0, 1]에 속하는 이미지를 입력으로 받아, 문턱값보다 크면 True, 작으면 False를 리턴한다.

    Parameters
    ----------
    x : np.ndarray
        이미지, 혹은 임의의 numpy.ndarray
    t : float, optional
        문턱값, by default 0.5

    Returns
    -------
    binary_image : np.ndarray
        이진화된 이미지
    """

    y = x.copy()
    return (y > t).astype(np.bool_)


def calculate_background(args: argparse.Namespace) -> np.ndarray:
    """동영상의 배경 이미지를 계산한다.

    Parameters
    ----------
    args : argparse.Namespace
        파이썬 실행 명령어에서 전달 받은 인자

    Returns
    -------
    background : np.ndarray
        배경 이미지(흑백)
    """

    def func(x):
        x = cv2.cvtColor(x, cv2.COLOR_BGR2GRAY)  # 이미지 흑백화(채널 단일화: 3 -> 1)
        x = x.astype(np.float64) / 255  # 0 ~ 1 사이의 실수값으로 스케일링
        return binary(x, args.threshold)  # 이미지 이진화

    text = "Calculating Background..."

    # 이진화된 영상을 가져온다.
    binary_images = np.stack(video_apply(args.input_path, func, text))
    # 평균으로 단일 이미지를 계산한다.
    background = binary_images.mean(axis=0)

    # 필터를 적용해 제거 범위를 더 넓힌다.
    k = 5
    kernel = np.ones((k, k)) / k ** 2
    background = cv2.filter2D(background, -1, kernel)

    # 배경 이미지를 이진화
    background = binary(background, 0.3)
    return background


def remove_background(x: np.ndarray, background: np.ndarray) -> np.ndarray:
    """이미지에서 배경을 제거한다."""

    assert x.dtype == background.dtype == np.bool_
    return x > background  # 어떤 픽셀에 대해, 배경에는 신호가 없고 이미지에만 신호가 있으면 배경이 아닌 마커이다.


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

    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
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

    if len(points.shape) != 2 or points.shape != (3, 2):
        return None

    distances = points.reshape(1, 3, 2) - points.reshape(3, 1, 2)
    distances = np.linalg.norm(distances, axis=-1)
    args = np.argsort(distances.sum(axis=0))[[1, 0, 2]]
    points = points[args]
    distances = distances[args][:, args]

    validities = []

    a = points[1] - points[0]
    b = points[2] - points[0]
    cos_sim = (a * b).sum() / (np.linalg.norm(a) * np.linalg.norm(b))
    validities.append(np.abs(cos_sim - 1) < 1e-3)
    validities.append(np.abs(distances[0, 1] * 2 - distances[1, 2]) < 2)

    is_valid = all(validities)

    return points if is_valid else None


def identify_position(points: np.ndarray) -> np.ndarray:
    """마커 번호를 식별하는 함수. 마커들이 Co-Planar하고 마커들 사이의 상대적인 좌표가 정적일 경우 자동으로 식별한다.

    Parameters
    ----------
    points : np.ndarray
        식별되지 않은 마커 좌표

    Returns
    -------
    points : np.ndarray
        식별되어 정렬된 마커 좌표
    """

    if points.shape != POSITIONS.shape:
        return None

    # 순열로 모든 경우의 수를 계산한다.
    perms = permutations(range(points.shape[0]), points.shape[0])
    perms = np.array(list(perms))  # (N!, N)

    # 상삼각행렬의 인덱스(대각 성분 미포함)
    triu_indices = np.triu_indices(N, 1)  # (L,)

    X = points[perms]  # (N!, N, 2)
    # 모든 점들에 대해 점과 점 사이 벡터를 계산한다.
    V = X[:, np.newaxis, ...] - X[..., np.newaxis, :]  # (N!, N, N, 2)

    # 종복이나 0값이 포함되어 있으므로 상삼각행렬을 flatten한 것만 남긴다.
    V = V[:, triu_indices[0], triu_indices[1]]  # (N!, L, 2)

    # 실제 마커를 어떤 평면에 투영할 경우 점들 사이 벡터의 비율이 일정하다.(차원별로 일정하다. 거리는 일정하지 않다.)
    R = np.linalg.norm(V / VECS[np.newaxis, ...], axis=-1)  # (N!, L)
    m = R.mean(axis=1)[..., np.newaxis]  # (N!, 1)
    error = (R - m) / m  # (N!, L)
    is_valid = (error < 3e-2).all(axis=1)  # (N!,)

    if is_valid.sum() == 1:
        index = perms[is_valid]  # (1, N)
        return points[index[0]]  # (N,)

    return None


def main(args: argparse.Namespace):
    os.makedirs("tmp", exist_ok=True)
    if args.remove_background:
        # 이미 배경 이미지가 계산되어 tmp에 저장되었다면 해당 데이터를 불러오고 아니면 새로 계산하여 tmp에 저장한다.
        bg_path = os.path.join(
            "tmp", "bg_" + args.input_path.replace("/", "-") + ".npy"
        )
        if os.path.isfile(bg_path):
            background = np.load(bg_path)
        else:
            background = calculate_background(args)
            np.save(bg_path, background)

        # 이미 배경이 제거된 영상이 tmp에 저장되었다면 해당 데이터를 불러오고 아니면 새로 계산하여 tmp에 저장한다.
        rm_path = os.path.join(
            "tmp", "rm_" + args.input_path.replace("/", "-") + ".npy"
        )
        if os.path.isfile(rm_path):
            images = np.load(rm_path)
        else:
            func = lambda x: remove_background(
                binary(cv2.cvtColor(x, cv2.COLOR_BGR2GRAY) / 255, args.threshold),
                background,
            )

            images = np.stack(
                video_apply(args.input_path, func, "Removing Background...")
            )
            np.save(rm_path, images)
    else:
        # 이미 원본 영상이 tmp에 저장되었다면 해당 데이터를 불러오고 아니면 새로 계산하여 tmp에 저장한다.
        path = os.path.join("tmp", args.input_path.replace("/", "-") + ".npy")
        if os.path.isfile(path):
            images = np.load(path)
        else:
            func = lambda x: binary(
                cv2.cvtColor(x, cv2.COLOR_BGR2GRAY) / 255, args.threshold
            )

            images = np.stack(video_apply(args.input_path, func, "Loading Images..."))
            np.save(path, images)

    images = images.astype(np.uint8) * 255
    wrapper = lambda x: find_markers(x, area_min_size=args.area_min_size)

    identify_function = lambda x: None if len(x) == 0 else x
    if args.identification == "custom":
        identify_function = identify_custom
    elif args.identification == "position":
        identify_function = identify_position

    markers = []
    frame_numbers = []
    progress = tqdm(enumerate(map(wrapper, images)), total=images.shape[0])
    for i, points in progress:
        if (points := identify_function(points)) is not None:
            frame_numbers.append(i)
            markers.append(points)

        progress.set_description("Detecting Markers...")

    frame_numbers = np.array(frame_numbers)
    markers = np.array(markers)
    print("Detected Frame Count:", markers.shape[0])
    os.makedirs(os.path.split(args.output_path)[0], exist_ok=True)
    np.save(args.output_path, {"frame_numbers": frame_numbers, "markers": markers})


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_path", type=str)
    parser.add_argument("output_path", type=str)
    parser.add_argument("-t", "--threshold", type=float, default=0.7)
    parser.add_argument("--area-min-size", type=int, default=1)
    parser.add_argument("--identification", type=str, default="custom")
    parser.add_argument("--remove-background", action="store_true")
    args = parser.parse_args()

    if not os.path.isfile(args.input_path):
        raise InvalidArgumentError("입력 영상 경로가 잘못되었습니다.")
    if not (0 <= args.threshold <= 1):
        raise InvalidArgumentError("이미지 이진화 문턱값은 구간 [0, 1]에 존재해야 합니다.")
    if args.area_min_size < 1:
        raise InvalidArgumentError("최소 영역 넓이는 1 이상이어야 합니다.")
    if args.identification not in ["custom", "position"]:
        raise InvalidArgumentError("식별 모드(identification)은 custom 또는 postition만 가능합니다.")
    if args.identification == "position":
        if POSITIONS is None:
            raise InvalidArgumentError(
                "postition 식별 모드를 사용하려면 전역 변수 POSITION에 값을 설정해야 합니다."
            )
        elif len(POSITIONS.shape) != 2 or POSITIONS.shape[1] != 2:
            raise InvalidArgumentError("잘못된 POSITION 형식입니다. :" + str(POSITIONS.shape))

    main(args)
