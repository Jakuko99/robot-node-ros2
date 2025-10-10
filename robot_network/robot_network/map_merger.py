import numpy as np
import cv2 as cv
import os


def merge_maps(map1: np.ndarray, map2: np.ndarray) -> np.ndarray:
    features1 = cv.detectAndCompute(map1, None)
    features2 = cv.detectAndCompute(map2, None)

    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    matches = bf.match(features1[1], features2[1])
    matches = sorted(matches, key=lambda x: x.distance)
    good_matches = matches[:50]
    src_pts = np.float32([features1[0][m.queryIdx].pt for m in good_matches]).reshape(
        -1, 1, 2
    )
    dst_pts = np.float32([features2[0][m.trainIdx].pt for m in good_matches]).reshape(
        -1, 1, 2
    )
    M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
    h, w = map1.shape
    warped_map1 = cv.warpPerspective(map1, M, (w, h))
    merged_map = np.maximum(warped_map1, map2)
    return merged_map


if __name__ == "__main__":
    map1 = cv.imread(
        "/home/ubuntu/ros_ws/src/robot_network/resource/dataset/kris_robot2/map_data_1759758139.txt",
        cv.IMREAD_GRAYSCALE,
    )
    map2 = cv.imread(
        "/home/ubuntu/ros_ws/src/robot_network/resource/dataset/kris_robot1/map_data_1759758138.txt",
        cv.IMREAD_GRAYSCALE,
    )

    merged_map = merge_maps(map1, map2)

    cv.imwrite(
        "/home/ubuntu/ros_ws/src/robot_network/resource/merged_map.png", merged_map
    )
