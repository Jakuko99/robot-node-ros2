from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import math
import cv2


class TrajectoryRecorder:
    def __init__(self, static_transform_x: float = 0.0, static_transform_y: float = 0.0):
        self.trajectory: list[Path] = []
        self.odometry_history: list[Odometry] = []
        self.nav_goals: list[PoseStamped] = []
        self.static_transform_x: float = static_transform_x
        self.static_transform_y: float = static_transform_y

    def track_route(self, msg: Path):
        self.trajectory.append(msg)

        self.nav_goals.append(msg.poses[-1])  # track the final pose of each path as a goal

    def store_odometry(self, msg: Odometry, update_distance: float = 0.5):
        if not self.odometry_history:
            self.odometry_history.append(msg)
            return

        last_odom: Odometry = self.odometry_history[-1]
        distance = math.sqrt(
            (msg.pose.pose.position.x - last_odom.pose.pose.position.x) ** 2
            + (msg.pose.pose.position.y - last_odom.pose.pose.position.y) ** 2
        )

        if distance >= update_distance:
            self.odometry_history.append(msg)

    def export_odometry(self, filename: str, create_plot: bool = False):
        with open(filename, "w") as f:
            for odom in self.odometry_history:
                f.write(f"{odom.pose.pose.position.x},{odom.pose.pose.position.y}\n")

        if create_plot:
            x_values: list[float] = []
            y_values: list[float] = []
            for odom in self.odometry_history:
                x_values.append(odom.pose.pose.position.x)
                y_values.append(odom.pose.pose.position.y)

            plt.scatter(x_values, y_values)
            plt.xlabel("X Position")
            plt.ylabel("Y Position")
            plt.title("Robot Odometry")
            plt.savefig(filename.replace(".txt", ".png"))

    def export_trajectory(self, filename: str, create_plot: bool = False):
        with open(filename, "w") as f:
            for path in self.trajectory:
                for pose in path.poses:
                    f.write(f"{pose.pose.position.x},{pose.pose.position.y}\n")

        if create_plot:
            x_values: list[float] = []
            y_values: list[float] = []
            for path in self.trajectory:
                for pose in path.poses:
                    x_values.append(pose.pose.position.x)
                    y_values.append(pose.pose.position.y)

            plt.scatter(x_values, y_values)
            plt.xlabel("X Position")
            plt.ylabel("Y Position")
            plt.title("Robot Trajectory")
            plt.savefig(filename.replace(".txt", ".png"))

    def create_map_overlay(
        self,
        map: OccupancyGrid,
        filename: str,
        source: str = "trajectory",
        include_goals: bool = True,
    ):
        map_origin_x, map_origin_y = map.info.origin.position.x, map.info.origin.position.y
        map_resolution = map.info.resolution
        map_width, map_height = map.info.width, map.info.height

        map_array = np.array(map.data, dtype=np.int8).reshape((map_height, map_width))
        map_image = np.zeros((map_height, map_width, 3), dtype=np.uint8)
        map_image[map_array == 0] = [255, 255, 255]
        map_image[map_array == 100] = [0, 0, 0]
        map_image[map_array == -1] = [127, 127, 127]
        map_image[(map_array >= 10) & (map_array < 100)] = [255, 0, 0]
        map_image[map_array == -10] = [0, 0, 255]
        map_image[map_array == 110] = [0, 255, 0]

        if source == "odometry":
            for odom in self.odometry_history:
                map_x = int((odom.pose.pose.position.x - map_origin_x) / map_resolution)
                map_y = int((odom.pose.pose.position.y - map_origin_y) / map_resolution)
                if 0 <= map_x < map_width and 0 <= map_y < map_height:
                    map_image[map_height - 1 - map_y, map_x] = [255, 255, 0]

        elif source == "trajectory":
            for path in self.trajectory:
                for pose in path.poses:
                    map_x = int((pose.pose.position.x - map_origin_x) / map_resolution)
                    map_y = int((pose.pose.position.y - map_origin_y) / map_resolution)
                    if 0 <= map_x < map_width and 0 <= map_y < map_height:
                        map_image[map_height - 1 - map_y, map_x] = [255, 255, 0]

        if include_goals:
            for goal in self.nav_goals:
                map_x = int((goal.pose.position.x - map_origin_x) / map_resolution)
                map_y = int((goal.pose.position.y - map_origin_y) / map_resolution)
                if 0 <= map_x < map_width and 0 <= map_y < map_height:
                    map_image[map_height - 1 - map_y, map_x] = [255, 0, 255]

        cv2.imwrite(filename, map_image)
