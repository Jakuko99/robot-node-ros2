import os
import json
import cv2 as cv
import numpy as np
import pickle as pkl
import matplotlib.pyplot as plt


def compress_map(file: str, output_file: str = None) -> np.ndarray:
    with open(
        file.replace("map_data", "map_metadata").replace(".txt", ".json"), "r"
    ) as f:
        metadata = json.load(f)

    with open(file, "r") as f:
        mat_raw = np.array([line.split(",") for line in f.readlines()], dtype=float)

        mat = np.reshape(mat_raw.astype(float), (metadata["height"], metadata["width"]))

    mat = cv.resize(mat, (300, 200), interpolation=cv.INTER_NEAREST)
    # compute new resolution to match format of old one
    metadata["resolution"] = metadata["resolution"] * (mat_raw.shape[1] / mat.shape[1])
    metadata["width"] = mat.shape[1]
    metadata["height"] = mat.shape[0]

    if output_file:
        cv.imwrite(output_file, mat)

    return mat


def load_from_pickle(file: str, output_path: str = None):
    with open(file, "rb") as f:
        data: dict = pkl.load(f)

    # mat_raw: np.ndarray = cv.resize(data["map"], (300, 200), interpolation=cv.INTER_NEAREST)
    # print(mat_raw.shape)

    mat = data["map"]
    # count values in mat
    unique, counts = np.unique(mat, return_counts=True)
    count_dict = dict(zip(unique, counts))
    print(f"Value counts in map: {count_dict}")

    for l in range(mat.shape[0]):
        for i in range(mat.shape[1]):
            if mat[l, i] == 0.5:
                mat[l, i] = 125
            elif mat[l, i] == 0.0:
                mat[l, i] = 255
            else:
                mat[l, i] = 0

    plt.figure()
    plt.imshow(mat, cmap="gray", origin="lower")
    plt.axis("off")

    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches="tight", pad_inches=0.1)


def parse_map(file: str, compressed_map: np.ndarray, output_path: str = None) -> int:
    with open(file, "r") as f:
        lines = f.readlines()
        discovered_pixels: int = 0

        mat = np.zeros((len(lines), len(lines[0].strip().split(","))), dtype=int)

        l = 0
        for line in lines:
            i = 0
            for val in line.strip().split(","):
                if val == "0.5":
                    mat[l, i] = 125
                elif val == "0.0":
                    mat[l, i] = 0
                else:
                    mat[l, i] = 255
                if val != "0.5":
                    discovered_pixels += 1
                i += 1
            l += 1

    compressed_map_processed = np.zeros_like(compressed_map, dtype=int)
    compressed_map_processed[np.isclose(compressed_map, 0.0)] = 0
    compressed_map_processed[np.isclose(compressed_map, 0.5)] = 125
    compressed_map_processed[np.isclose(compressed_map, 1.0)] = 255

    plt.subplot(2, 1, 1)
    plt.imshow(mat, cmap="gray", origin="lower")
    plt.axis("off")

    plt.subplot(2, 1, 2)
    plt.imshow(compressed_map_processed, cmap="gray", origin="lower")
    plt.axis("off")

    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches="tight", pad_inches=0.1)


if __name__ == "__main__":
    # dataset_dir = f"{os.path.dirname(__file__)}/../resource/dataset"

    # compressed = compress_map(
    #     os.path.join(dataset_dir, "kris_robot1", "map_data_1759758158.txt"),
    # )

    # print(f"Compressed map shape: {compressed.shape}")

    # parse_map(
    #     os.path.join(dataset_dir, "kris_robot1", "map_data_1759758158.txt"),
    #     compressed,
    #     output_path=f"{os.path.dirname(__file__)}/../resource/map_comparison.png",
    # )

    load_from_pickle(
        f"{os.path.dirname(__file__)}/../resource/dataset_compressed/kris_robot1/compressed_data_1759923566.pkl",
        output_path=f"{os.path.dirname(__file__)}/../resource/map_from_pickle.png",
    )
