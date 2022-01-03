import os
import argparse
import numpy as np
from scipy.optimize import least_squares
from tqdm import tqdm


def find_K_from_M(m, M):
    n_iter = 100
    n_samples = m.shape[0] // 10

    homography_matrices = np.stack(
        [find_H_from_M(m[i], M) for i in tqdm(range(m.shape[0]), leave=False)]
    )

    def fun(K, R, T):
        X = np.einsum("nij, jk -> nik", R, M) + T
        X /= X[:, -1:]
        X = np.einsum("ij, njk -> nik", K, X)
        e = X[:, :-1] - m
        e = np.linalg.norm(e, axis=1) ** 2
        mse = e.mean(axis=1)
        rmse = np.sqrt(mse)
        return rmse

    max_inlier_count = 0
    best_K = None
    for i in tqdm(range(n_iter), leave=False):
        args = np.random.choice(np.arange(homography_matrices.shape[0]), n_samples)
        sample_homography_matrices = homography_matrices[args]
        try:
            K = find_K_from_H(sample_homography_matrices)
            RTs = np.stack(
                [
                    np.concatenate(find_RT_from_KH(K, H), axis=1)
                    for H in homography_matrices
                ]
            )
            rmse = fun(K, RTs[..., :3], RTs[..., 3:])
            inlier_count = (rmse < 10).sum()

            if inlier_count > max_inlier_count:
                max_inlier_count = inlier_count
                best_K = K
        except np.linalg.LinAlgError as e:
            continue
        except NotPerpendicularError as e:
            continue
    assert best_K is not None
    return best_K


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("marker_dir", type=str)
    parser.add_argument("intr_dir", type=str)
    parser.add_argument("output_dir", type=str)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    names = os.listdir(args.marker_dir)

    pairs = [[i, i + 1] for i in range(len(names) - 1)]

    for pair in tqdm(pairs):
        i, j = pair

        data = np.load(
            os.path.join(args.marker_dir, names[i]), allow_pickle=True
        ).tolist()
        K1 = np.load(os.path.join(args.intr_dir, names[i]))
        frame_numbers1 = list(data["frame_numbers"])
        m1 = data["markers"].transpose(0, 2, 1)

        data = np.load(
            os.path.join(args.marker_dir, names[j]), allow_pickle=True
        ).tolist()
        K2 = np.load(os.path.join(args.intr_dir, names[j]))
        frame_numbers2 = list(data["frame_numbers"])
        m2 = data["markers"].transpose(0, 2, 1)

        frame_numbers = sorted(list(set(frame_numbers1) & set(frame_numbers2)))

        indices1 = [frame_numbers1.index(n) for n in frame_numbers]
        indices2 = [frame_numbers2.index(n) for n in frame_numbers]

        m1 = m1[indices1]
        m2 = m2[indices2]

        # TODO
        # 1. 완드의 3차원 좌표 기반하여 각각의 카메라의 프레임마다의 RT를 계산, 상대 RT를 초기 값으로 설정
        # 2. 이를 최적화(RANSAC or LM)
