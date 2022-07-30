"""
두 카메라에서 동시에 식별된 마커 좌표로부터 외부 캘리브레이션을 진행한다.
    -> 결과값 : 카메라 외부 파라미터 [R|T]

사용 예시
$ python 1.1_calibrate_stereo.py \
    ../data/marker/calib/id/01.npy \
    ../data/marker/calib/id/02.npy \
    ../data/params/intr/01.npy \
    ../data/params/intr/02.npy \
    ../data/params/extr/01_02.npy
"""

import os
import argparse
import numpy as np
from common import *
from tqdm import tqdm
from utils.camera import (
    estimate_homography,
    calculate_relative_extr,
    calculate_reprojection_error,
    NotPerpendicularError,
)
from utils.linalg import exp_map, log_map


def find_initial_extr_params(points_2d, K1, K2, H_mats):
    T = points_2d.shape[1]
    N = points_2d.shape[3]

    progress = tqdm(range(T))

    minimum_error = None
    best_RT = None
    valid_args = list(range(T))

    for i in progress:
        H1 = H_mats[0, i]
        H2 = H_mats[1, i]
        try:
            R1, T1 = calculate_relative_extr(K1, H1)
            R2, T2 = calculate_relative_extr(K2, H2)
        except NotPerpendicularError:
            valid_args.remove(i)
            continue

        P1 = K1 @ np.concatenate([R1, T1], axis=1)
        P2 = K2 @ np.concatenate([R2, T2], axis=1)
        project_matrices = np.stack([P1, P2])

        U = len(valid_args)
        X = points_2d[:, valid_args]  # (2, U, 2, N)
        X = X.transpose(0, 1, 3, 2)  # (2, U, N, 2)
        X = X.reshape(2, -1, 2)  # (2, UN, 2)
        error, m, M = calculate_reprojection_error(
            X, project_matrices, with_points=True
        )

        if minimum_error is None or minimum_error > error:
            R_rel = R2 @ np.linalg.inv(R1)
            T_rel = -R2 @ np.linalg.inv(R1) @ T1 + T2

            minimum_error = error
            best_RT = np.concatenate([R_rel, T_rel], axis=1)

        progress.set_description("Finding Initial Extrinsic Parameters...")

    assert best_RT is not None

    return best_RT, minimum_error


def calibrate_extr(points_2d, K1, K2, init_RT):
    from scipy.optimize import least_squares

    T = points_2d.shape[1]
    N = points_2d.shape[3]

    T = init_RT[:, 3:]
    w = log_map(init_RT[:, :3])

    x0 = np.concatenate([w, T]).reshape(-1)

    def fun(x):
        R1, T1 = np.eye(3), np.zeros((3, 1))
        R2, T2 = exp_map(x[:3, np.newaxis]), x[3:, np.newaxis]
        P1 = K1 @ np.concatenate([R1, T1], axis=1)
        P2 = K2 @ np.concatenate([R2, T2], axis=1)
        project_matrices = np.stack([P1, P2])

        U = points_2d.shape[1]
        X = points_2d  # (2, U, 2, N)
        X = X.transpose(0, 1, 3, 2)  # (2, U, N, 2)
        X = X.reshape(2, -1, 2)  # (2, UN, 2)
        error = calculate_reprojection_error(X, project_matrices, to_scalar=None)
        error = error.reshape(2, U, N).mean(axis=(0, 2))

        return error

    res = least_squares(fun, x0, method="lm", verbose=0, xtol=1e-8, ftol=1e-8)

    R = exp_map(res.x[:3, np.newaxis])
    T = res.x[3:, np.newaxis]
    RT = np.concatenate([R, T], axis=1)
    error = fun(res.x)

    return RT, error


def main(input_path1, input_path2, intr_path1, intr_path2, output_path):
    """main 스테레오 카메라의 상대적 외부 파라미터를 계산한다.

    Parameters
    ----------
    points_2d : np.ndarray
        스테레오 카메라에서 시간 T동안 관측된 픽셀 점 좌표. (2, T, 2, N)
    points_3d : np.ndarray
        실제 점 좌표. (3, N)
    K1 : np.ndarray
        첫 번째 카메라의 카메라 내부 파라미터 행렬
    K2 : np.ndarray
        두 번째 카메라의 카메라 내부 파라미터 행렬
    """

    data = np.load(input_path1, allow_pickle=True).tolist()
    K1 = np.load(intr_path1)
    frame_numbers1 = list(data["frame_numbers"])
    m1 = data["markers"].transpose(0, 2, 1)

    data = np.load(input_path2, allow_pickle=True).tolist()
    K2 = np.load(intr_path2)
    frame_numbers2 = list(data["frame_numbers"])
    m2 = data["markers"].transpose(0, 2, 1)

    frame_numbers = list(set(frame_numbers1) & set(frame_numbers2))

    indices1 = [frame_numbers1.index(n) for n in frame_numbers]
    indices2 = [frame_numbers2.index(n) for n in frame_numbers]

    m1 = m1[indices1]
    m2 = m2[indices2]

    points_2d = np.stack([m1, m2])
    points_3d = np.concatenate([POSITIONS, np.ones((len(POSITIONS), 1))], axis=1).T

    T = points_2d.shape[1]
    N = points_2d.shape[3]

    H_mats = [[], []]
    progress = tqdm(range(T))

    for i in progress:
        H_mats[0].append(estimate_homography(points_2d[0, i], points_3d))
        H_mats[1].append(estimate_homography(points_2d[1, i], points_3d))
        progress.set_description("Caculating Homography Matrices...")

    H_mats = np.array(H_mats)

    init_RT, error = find_initial_extr_params(points_2d, K1, K2, H_mats)
    print("Optimizing Parameters...", end="")
    RT, error = calibrate_extr(points_2d, K1, K2, init_RT)
    print("Done!")

    text = f"""캘리브레이션 오차: {error.mean()}"""
    print(text)

    os.makedirs(os.path.split(output_path)[0], exist_ok=True)
    np.save(output_path, RT)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_path1", type=str)
    parser.add_argument("input_path2", type=str)
    parser.add_argument("intr_path1", type=str)
    parser.add_argument("intr_path2", type=str)
    parser.add_argument("output_path", type=str)
    args = parser.parse_args()

    main(
        args.input_path1,
        args.input_path2,
        args.intr_path1,
        args.intr_path2,
        args.output_path,
    )
