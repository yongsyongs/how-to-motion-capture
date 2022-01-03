import cv2
import argparse
import numpy as np
from tqdm import tqdm


def main(args: argparse.Namespace):
    result = np.load(args.result_path, allow_pickle=True).tolist()
    frame_numbers = result["frame_numbers"].tolist()
    points = result["markers"]

    marker_count = points.shape[1]
    colors = (
        np.array([[255, 0, 0]])
        + np.array([[-255, 255, 0]])
        * np.arange(marker_count).reshape(marker_count, 1)
        / marker_count
    )

    cap = cv2.VideoCapture(args.video_path)
    length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    for i in tqdm(range(length)):
        ret, image = cap.read()
        if not ret or image is None:
            break

        if i in frame_numbers:
            X = points[frame_numbers.index(i)].astype("int")
            for j, x in enumerate(X):
                image = cv2.rectangle(image, x - (5, 5), x + (5, 5), colors[j])
        cv2.imshow("", image)
        cv2.waitKey(1)

    cap.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("video_path", type=str)
    parser.add_argument("result_path", type=str)
    args = parser.parse_args()

    main(args)
