import os
import numpy as np
import pickle as pkl

DATASET_DIR = f"{os.path.dirname(__file__)}/../resource/dataset"
COMPRESSED_DIR = f"{os.path.dirname(__file__)}/../resource/dataset_compressed"

def compress_map_data(namespace: str):
    dataset_path = os.path.join(DATASET_DIR, namespace)
    compressed_path = os.path.join(COMPRESSED_DIR, namespace)
    os.makedirs(compressed_path, exist_ok=True)

    map_files = sorted(
        [f for f in os.listdir(dataset_path) if f.startswith("map_data_") and f.endswith(".txt")]
    )
    metadata_files = sorted(
        [f for f in os.listdir(dataset_path) if f.startswith("map_metadata_") and f.endswith(".json")]
    )
    odom_files = sorted(
        [f for f in os.listdir(dataset_path) if f.startswith("odom_data_") and f.endswith(".json")]
    )

    for map_file, metadata_file in zip(map_files, metadata_files):
        timestamp = map_file.split("_")[-1].split(".")[0]

        with open(os.path.join(dataset_path, map_file), "r") as f:
            mat = []
            for line in f:
                row = [float(val) for val in line.strip().split(",")]
                mat.append(row)
            mat = np.array(mat)

        with open(os.path.join(dataset_path, metadata_file), "r") as f:
            metadata = f.read()

        with open(os.path.join(dataset_path, f"odom_data_{timestamp}.json"), "r") as f:
            odom = f.read()

        compressed_data = {
            "map": mat,
            "metadata": metadata,
            "odom": odom,
        }

        with open(os.path.join(compressed_path, f"compressed_map_{timestamp}.pkl"), "wb") as f:
            pkl.dump(compressed_data, f)

        print(f"Compressed map data saved for timestamp {timestamp} in namespace '{namespace}'.")

if __name__ == "__main__":
    namespaces = ["kris_robot1", "kris_robot2"]  # Add more namespaces as needed
    for ns in namespaces:
        compress_map_data(ns)