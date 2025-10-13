import numpy as np
import cv2 as cv
import json
import os


def merge_maps(map1: np.ndarray, map2: np.ndarray) -> np.ndarray:
    sift = cv.SIFT_create()
    # Find keypoints and descriptors
    keypoints1, descriptors1 = sift.detectAndCompute(map1, None)
    keypoints2, descriptors2 = sift.detectAndCompute(map2, None)

    # Set FLANN parameters
    index_params = dict(algorithm=1, trees=5)
    search_params = dict()
    flann = cv.FlannBasedMatcher(index_params, search_params)

    # Match descriptors using KNN and apply Lowe's ratio test
    matches = flann.knnMatch(descriptors1, descriptors2, k=2)
    good_matches = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)

    if len(good_matches) > 4:
        src_pts = np.float32([keypoints1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Find homography
        H, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)

        # Warp map1 to map2's perspective
        height, width = map2.shape
        warped_map1 = cv.warpPerspective(map1, H, (width, height))

        # Merge the maps (simple max merge)
        merged_map = np.maximum(warped_map1, map2)

        return merged_map
    else:
        print("Not enough good matches to merge maps.")
        return None

def load_map(file: str) -> np.ndarray:
    with open(
        file.replace("map_data", "map_metadata").replace(".txt", ".json"), "r"
    ) as f:
        metadata = json.load(f)

    with open(file, "r") as f:
        mat_raw = np.array([line.split(",") for line in f.readlines()], dtype=float)

        mat = np.reshape(mat_raw.astype(float), (metadata["height"], metadata["width"]))
        mat[mat == 1.0] = 255
        mat[mat == 0.5] = 125
        mat[mat == 0] = 0
        mat = mat.astype(np.uint8)

    return mat


if __name__ == "__main__":
    map1 = load_map(
        "/home/ubuntu/ros_ws/src/robot_network/resource/dataset/kris_robot1/map_data_1759758138.txt"
    )
    map2 = load_map(
        "/home/ubuntu/ros_ws/src/robot_network/resource/dataset/kris_robot2/map_data_1759758139.txt"
    )
    # cv.imshow("Map 1", map1)
    # cv.imshow("Map 2", map2)
    # cv.waitKey(0)

    if map1.any() and map2.any():
        merged_map = merge_maps(map1, map2)
    else:
        merged_map = None
        print("Error: One or both maps could not be loaded.")

    if merged_map.any():
        cv.imwrite(
            "/home/ubuntu/ros_ws/src/robot_network/resource/merged_map.png", merged_map
        )
