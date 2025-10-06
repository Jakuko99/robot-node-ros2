import os
import matplotlib.pyplot as plt
import numpy as np


def parse_map(file_path: str, output_path: str = None) -> int:
    with open(file_path, "r") as f:
        lines = f.readlines()
        discovered_pixels: int = 0

        mat = np.zeros((len(lines), len(lines[0].strip().split(","))), dtype=float)

        l = 0
        for line in lines:
            i = 0
            for val in line.strip().split(","):
                mat[l, i] = int(125 if val == "0.5" else (0 if val == "0.0" else 255))
                if val != "0.5":
                    discovered_pixels += 1
                i += 1
            l += 1

    plt.imshow(mat, cmap="gray", origin="lower")
    plt.tight_layout()
    plt.axis("off")
    if output_path:
        plt.savefig(output_path, dpi=300)


def main():
    dataset_dir = f"{os.path.dirname(__file__)}/../resource/dataset"

    for file in os.listdir(os.path.join(dataset_dir, "kris_robot2")):
        if file.endswith(".txt"):
            parse_map(
                os.path.join(dataset_dir, "kris_robot2", file),
                os.path.join(dataset_dir, "kris_robot2", f"plot_{file[:-4]}.png"),
            )


if __name__ == "__main__":
    main()
