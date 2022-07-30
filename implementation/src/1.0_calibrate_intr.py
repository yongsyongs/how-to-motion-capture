"""
식별된 마커 좌표로부터 내부 캘리브레이션을 진행한다.
    -> 결과값 : 카메라 내부 파라미터 행렬 K

사용 예시
$ python 1.0_calibrate_intr.py ../data/marker/calib/id/01.npy ../data/params/intr/01.npy

"""

import os
import argparse
import numpy as np
from common import *
from tqdm import tqdm
from scipy.optimize import least_squares
from utils.camera import (
    estimate_homography,
    calculate_relative_extr,
    NotPerpendicularError,
)


def estimate_intr_params(H_mats):
    N_image = H_mats.shape[0]

    h = lambda k, i, j: H_mats[k, j - 1, i - 1]
    v = lambda k, i, j: np.array(
        [
            [
                h(k, i, 1) * h(k, j, 1),
                h(k, i, 1) * h(k, j, 2) + h(k, i, 2) * h(k, j, 1),
                h(k, i, 2) * h(k, j, 2),
                h(k, i, 3) * h(k, j, 1) + h(k, i, 1) * h(k, j, 3),
                h(k, i, 3) * h(k, j, 2) + h(k, i, 2) * h(k, j, 3),
                h(k, i, 3) * h(k, j, 3),
            ]
        ]
    ).T

    V = np.concatenate(
        [
            np.concatenate([v(i, 1, 2).T, (v(i, 1, 1) - v(i, 2, 2)).T], axis=0)
            for i in range(H_mats.shape[0])
        ]
    )

    if N_image == 2:
        V = np.concatenate([V, np.array([[0, 1, 0, 0, 0, 0]])], axis=0)

    w, v = np.linalg.eig(V.T @ V)
    b = v[:, np.argmin(w)]
    b11, b12, b22, b13, b23, b33 = b
    v0 = (b12 * b13 - b11 * b23) / (b11 * b22 - b12**2)
    lambda_ = b33 - (b13**2 + v0 * (b12 * b13 - b11 * b23)) / b11
    alpha = np.sqrt(lambda_ / b11)
    beta = np.sqrt(lambda_ * b11 / (b11 * b22 - b12**2))
    gamma = -b12 * alpha**2 * beta / lambda_
    u0 = gamma * v0 / beta - b13 * alpha**2 / lambda_
    # gamma = 0
    # u0 = gamma * v0 / beta - b13 * alpha**2 / lambda_
    # K = np.array([[alpha, gamma, u0], [0, beta, v0], [0, 0, 1]])
    K = np.array([[alpha, 0, u0], [0, beta, v0], [0, 0, 1]])

    return K


def calibrate_intr(
    points_2d: np.ndarray,
    points_3d: np.ndarray,
    H_mats: np.ndarray,
    n_iter: int = 100,
    n_samples: int = -1,
    inlier_error: float = 10,
) -> np.ndarray:
    """RANSAC으로 카메라 내부 파라미터 행렬 K를 계산한다.

    Parameters
    ----------
    points_2d : np.ndarray
        매 프레임마다 영상에서 촬영된 점. (T, 2, N)
    points_3d : np.ndarray
        실제 점 좌표. (3, N)
    H_mats : np.ndarray
        매 프레임마다 계산된 호모그라피 행렬. (T, 3, 3)
    n_iter : int
        RANSAC 반복 횟수
    n_samples : int
        RANSAC 반복마다 추출할 샘플의 수

    Returns
    -------
    np.ndarray
        계산된 내부 파라미터 행렬 K. (3, 3)
    """

    assert points_2d.shape[0] == H_mats.shape[0]
    assert points_2d.shape[2] == points_3d.shape[1]
    T = points_2d.shape[0]
    N = points_2d.shape[2]

    if n_samples < 1:
        n_samples = T // 10

    def proj_cam(m, M, K, R_mats, T_vecs):
        """3차원 점들을 카메라로 사영해 실제 관측된 점과의 오차를 계산한다.

        Parameters
        ----------
        m : np.ndarray
            2차원에서 관측된 점
        M : np.ndarray
            실제 3차원 점 좌표
        K : np.ndarray
            카메라 내부 파라미터 행렬
        R_mats : np.ndarray
            카메라 외부 파라미터. Rotation Matrix
        T_vecs : np.ndarray
            카메라 외부 파라미터. Translation Vector

        Returns
        -------
        float
            계산된 Reprojection Error(RMSE)
        """

        X = np.einsum("nij, jk -> nik", R_mats, M) + T_vecs
        X /= X[:, -1:]
        X = np.einsum("ij, njk -> nik", K, X)
        error = np.sqrt(((X[:, :-1] - m) ** 2).sum(axis=1)).mean(axis=1)

        return error

    max_inlier_count = 0
    best_K = None
    minimum_error = None
    for i in tqdm(range(n_iter), leave=False):
        args = np.random.choice(np.arange(T), n_samples)
        sample_H_mats = H_mats[args]

        K = estimate_intr_params(sample_H_mats)
        RT_mats = []
        valid_args = []
        for i, H in enumerate(H_mats):
            try:
                RT = np.concatenate(calculate_relative_extr(K, H), axis=1)
                RT_mats.append(RT)
                valid_args.append(i)
            except NotPerpendicularError as e:
                continue

        RT_mats = np.stack(RT_mats)
        error = proj_cam(
            points_2d[valid_args],
            points_3d,
            K,
            RT_mats[..., :3],
            RT_mats[..., 3:],
        )
        inlier_count = (error < inlier_error).sum()

        if inlier_count > max_inlier_count:
            max_inlier_count = inlier_count
            best_K = K
            minimum_error = error

    assert best_K is not None

    return best_K, minimum_error


def main(input_path, output_path):
    data = np.load(args.input_path, allow_pickle=True).tolist()
    points_2d = data["markers"].transpose(0, 2, 1)

    # 피사체에 알맞게 POSITIONS 수정 필요.
    points_3d = np.concatenate([POSITIONS, np.ones((len(POSITIONS), 1))], axis=1).T

    # 영상의 매 프레임 사진마다 호모그라피 행렬을 따로 계산한다.
    progress = tqdm(range(points_2d.shape[0]))
    homography_matrices = [None] * points_2d.shape[0]

    for i in progress:
        homography_matrices[i] = estimate_homography(points_2d[i], points_3d)
        progress.set_description("Caculating Homography Matrices...")

    homography_matrices = np.stack(homography_matrices)

    K, error = calibrate_intr(points_2d, points_3d, homography_matrices)

    text = f"""캘리브레이션 오차
    mean: {error.mean()}
    std: {error.std()}
    min: {error.min()}
    max: {error.max()}
    """
    print(text)
    print(K)

    os.makedirs(os.path.split(output_path)[0], exist_ok=True)
    np.save(output_path, K)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_path", type=str)
    parser.add_argument("output_path", type=str)
    args = parser.parse_args()

    main(args.input_path, args.output_path)
