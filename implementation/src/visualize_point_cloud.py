"""
촬영된 액션씬의 마커들의 3D 좌표를 복원하여 3차원 Points Cloud를 만든다.

사용 예시
$ python 2.1_make_point_cloud.py --config-path reconstruct-config.json

"""

import os
import cv2
import json
import argparse
import numpy as np
from tqdm import tqdm
from utils.camera import project


def main(
    n_cameras: int,
    params_path: str,
    marker_path: str,
    output_path: str,
    fps: float = 240,
    resolution: tuple = (640, 360),
    point_cloud_path: str = "./point_cloud.npy",
):
    params = np.load(params_path, allow_pickle=True).tolist()["CameraParameters"]
    projection_matrices = np.stack(
        [x["K"] @ np.concatenate([x["R"], x["T"]], axis=1) for x in params]
    )

    cloud = np.load(point_cloud_path, allow_pickle=True).tolist()[:10000]

    out = cv2.VideoWriter(
        output_path,
        cv2.VideoWriter_fourcc(*"DIVX"),
        fps,
        (resolution[0] * n_cameras, resolution[1]),
    )

    n_frames = len(cloud)
    progress = tqdm(enumerate(cloud), total=n_frames)
    for frame, points in progress:
        image = np.zeros((resolution[1], resolution[0] * n_cameras, 3), dtype=np.uint8)

        n_points = len(points)
        if n_points > 0:
            points = np.concatenate(points)  # (n_points, 3)
            points_2d = project(points, projection_matrices)  # (n_cameras, n_points, 2)

            for camera, points_in_cam in enumerate(points_2d):
                image[:, camera * resolution[0] + (np.arange(3) - 1), :] = 255
                for i, p in enumerate(points_in_cam):
                    p = (p + (camera * resolution[0], 0)).astype("int")
                    image = cv2.circle(image, p, 5, (255, 255, 255), 1)

        out.write(image)
    out.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output_path", type=str)
    parser.add_argument("fps", type=float, default=240)
    parser.add_argument("res_w", type=int, default=640)
    parser.add_argument("res_h", type=int, default=360)
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
        args.output_path,
        args.fps,
        (args.res_w, args.res_h),
        config["OutputPath"],
    )
