"""
카메라마다 따로 저장되어 있는 파라미터 데이터를 한 파일에 모아 저장한다.

사용 예시
$ python 1.2_join_data.py --config-path join-data-config.json

"""

import os
import json
import argparse
import numpy as np


def load_parametrs(intr_params_path, extr_params_path, extr_offset):
    n_cameras = len(intr_params_path)
    K_mats = [np.load(path) for path in intr_params_path]
    rel_RT_mats = [np.load(path) for path in extr_params_path]

    R_mats = [extr_offset["R"]]
    T_vecs = [extr_offset["T"]]

    # R[k] = R_rel@R[k-1]
    # T[k] = T_rel + R[k]@inv(R[k-1])@T[k-1]

    for rel_RT in rel_RT_mats:
        rel_R = rel_RT[:, :3]
        rel_T = rel_RT[:, 3:]
        R = rel_R @ R_mats[-1]
        T = rel_T + R @ np.linalg.inv(R_mats[-1]) @ T_vecs[-1]
        R_mats.append(R)
        T_vecs.append(T)

    K_mats = np.stack(K_mats)
    R_mats = np.stack(R_mats)
    T_vecs = np.stack(T_vecs)

    return K_mats, R_mats, T_vecs


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config-path",
        type=str,
        default="join-params-config.json",
    )

    args = parser.parse_args()

    with open(args.config_path, "r") as f:
        config = json.load(f)

    if config["ExtrinsicParameters"]["Offset"]["R"] == "I":
        config["ExtrinsicParameters"]["Offset"]["R"] = np.eye(3)
    else:
        config["ExtrinsicParameters"]["Offset"]["R"] = np.array(
            config["ExtrinsicParameters"]["Offset"]["R"]
        )

    if config["ExtrinsicParameters"]["Offset"]["T"] in [0, "0"]:
        config["ExtrinsicParameters"]["Offset"]["T"] = np.zeros((3, 1))
    else:
        config["ExtrinsicParameters"]["Offset"]["T"] = np.array(
            config["ExtrinsicParameters"]["Offset"]["T"]
        )

    K_mats, R_mats, T_vecs = load_parametrs(
        config["IntrinsicParameters"],
        config["ExtrinsicParameters"]["Relatives"],
        config["ExtrinsicParameters"]["Offset"],
    )

    data = {
        "Metadata": {"n_cameras": K_mats.shape[0]},
        "CameraParameters": [
            {"K": K_mats[i], "R": R_mats[i], "T": T_vecs[i]}
            for i in range(K_mats.shape[0])
        ],
    }

    np.save(config["OutputPath"], data)
