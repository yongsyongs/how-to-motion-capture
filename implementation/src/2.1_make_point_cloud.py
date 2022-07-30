"""
촬영된 액션씬의 마커들의 3D 좌표를 복원하여 3차원 Points Cloud를 만든다.

사용 예시
$ python 2.1_make_point_cloud.py --config-path reconstruct-config.json

"""

import os
import json
import argparse
import numpy as np
from tqdm import tqdm
from collections import defaultdict
from utils.camera import calculate_reprojection_error, linear_triangulation


def points_permutation_indices(n_points):
    n_cam = len(n_points)
    indices = []
    for i in range(n_cam):
        a = np.prod(n_points)
        b = np.prod(n_points[i + 1 :])
        x = np.arange(n_points[i])
        x = np.repeat(x, b)
        x = np.tile(x, int(a / (b * n_points[i])))
        indices.append(x)

    indices = np.stack(indices)

    return indices


def match_points_spatially(points_2d, P_mats, threshold):
    n_points = np.array([x.shape[0] for x in points_2d])
    n_perms = np.prod(n_points)

    perm_indices = points_permutation_indices(n_points)
    perm_points = np.stack(
        [points_2d[0][perm_indices[0]], points_2d[1][perm_indices[1]]]
    )  # (2, n_perm, 2)

    error_val, _, points_3d = calculate_reprojection_error(
        perm_points,
        P_mats,
        to_scalar=None,
        with_points=True,
    )
    error_val = error_val.mean(axis=0)
    error = np.ones(n_points) * 1e10

    error[perm_indices[0], perm_indices[1]] = error_val

    is_valid = error < threshold
    if not is_valid.any():
        return [[], []]

    error[np.logical_not(is_valid)] = 1e10
    valid_indices = [
        (a, b)
        for x in np.argsort(error, axis=None)
        if is_valid[
            (a := x // n_points[1]),
            (b := x - x // n_points[1] * n_points[1]),
        ]
    ]

    minimum_args = [set(), set()]
    unique_valid_indices = []

    for idx in valid_indices:
        if idx[0] in minimum_args[0] or idx[1] in minimum_args[1]:
            continue

        unique_valid_indices.append(idx)
        minimum_args[0].add(idx[0])
        minimum_args[1].add(idx[1])

    # valid_points_3d = np.stack(
    #     [points_3d[idx[0] * n_points[1] + idx[1]] for idx in unique_valid_indices]
    # )
    # unique_valid_indices = np.array(unique_valid_indices)

    return unique_valid_indices


# region TODO: calcualte temporal correspondence
"""
def match_points_temporally(points_2d, points_2d_next, threshold):
    N1, N2 = points_2d.shape[0], points_2d_next.shape[0]
    error = points_2d[:, np.newaxis, ...] - points_2d_next[np.newaxis, ...]
    error = np.linalg.norm(error, axis=-1)
    is_valid = error < threshold
    if not is_valid.any():
        return []

    error[np.logical_not(is_valid)] = 1e10
    valid_indices = [
        (a, b)
        for x in np.argsort(error, axis=None)
        if is_valid[
            (a := x // N2),
            (b := x - x // N2 * N2),
        ]
    ]

    minimum_args = [set(), set()]
    unique_valid_indices = []

    for idx in valid_indices:
        if idx[0] in minimum_args[0] or idx[1] in minimum_args[1]:
            continue

        unique_valid_indices.append(idx)
        minimum_args[0].add(idx[0])
        minimum_args[1].add(idx[1])

    # unique_valid_indices = np.array(unique_valid_indices)

    return unique_valid_indices
"""


def main(
    n_cameras: int,
    params_path: str,
    marker_path: str,
    threshold_spatial: int = 10,
    point_cloud_path: str = "./point_cloud.npy",
):
    assert n_cameras >= 2

    params = np.load(params_path, allow_pickle=True).tolist()["CameraParameters"]
    projection_matrices = np.stack(
        [x["K"] @ np.concatenate([x["R"], x["T"]], axis=1) for x in params]
    )

    markers = np.load(marker_path, allow_pickle=True).tolist()
    n_frames = len(markers[0])

    camera_pairs = [[i, i + 1] for i in range(n_cameras - 1)]

    points = [None] * n_frames

    progress = tqdm(range(n_frames))

    for i in progress:
        points[i] = []
        for pair_idx, cam_pair in enumerate(camera_pairs):
            points_2d = [markers[j][i] for j in cam_pair]
            P_mats = projection_matrices[cam_pair]

            spatial_indices = match_points_spatially(
                points_2d,
                P_mats,
                threshold_spatial,
            )

            for corrs in spatial_indices:
                if len(corrs) != 2:
                    continue

                p2d = np.stack(
                    [points_2d[0][corrs[0]], points_2d[1][corrs[1]]]
                ).reshape(2, 1, 2)
                point = linear_triangulation(p2d, P_mats)
                points[i].append(point)
        progress.set_description("Calculating Correspondence")

    np.save(point_cloud_path, points)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config-path",
        type=str,
        default="reconstruct-config.json",
    )

    args = parser.parse_args()

    with open(args.config_path, "r") as f:
        config = json.load(f)

    main(
        config["CameraCount"],
        config["CameraParameters"],
        config["SynchronizedMarkerPath"],
        config["SpatialThreshold"],
        config["OutputPath"],
    )
