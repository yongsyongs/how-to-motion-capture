"""
개별 카메라별로 촬영된 액션씬의 마커들을 한 파일에 모아 저장한다.

사용 예시
$ python 2.0_synchronize.py --config-path reconstruct-config.json

"""

import os
import json
import argparse
import numpy as np
from tqdm import tqdm
from functools import reduce


def main(n_cameras, act_marker_paths, output_path):
    data = [np.load(path, allow_pickle=True).tolist() for path in act_marker_paths]
    frame_numbers = [x["frame_numbers"] for x in data]
    valid_frame_numbers = sorted(
        list(reduce(lambda a, b: a & b, map(set, frame_numbers)))
    )
    markers = [x["markers"] for x in data]
    synced_markers = [[] for _ in range(n_cameras)]

    for i in tqdm(valid_frame_numbers):
        for j in range(n_cameras):
            idx = frame_numbers[j].index(i)
            synced_markers[j].append(markers[j][idx])

    os.makedirs(os.path.split(output_path)[0], exist_ok=True)
    np.save(output_path, synced_markers)


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
        config["ActionMarkerPath"],
        config["SynchronizedMarkerPath"],
    )
