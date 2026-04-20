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

    def plot_movement(self, filename: str):
        x_values: list[float] = []
        y_values: list[float] = []
        for path in self.trajectory:
            for pose in path.poses:
                x_values.append(pose.pose.position.x)
                y_values.append(pose.pose.position.y)

        odom_x_values: list[float] = []
        odom_y_values: list[float] = []
        for odom in self.odometry_history:
            odom_x_values.append(odom.pose.pose.position.x)
            odom_y_values.append(odom.pose.pose.position.y)

        plt.scatter(x_values, y_values)
        plt.scatter(odom_x_values, odom_y_values, c="red", marker="x")
        plt.scatter(odom_x_values[0], odom_y_values[0], c="lime", marker="o", label="Start")
        plt.scatter(odom_x_values[-1], odom_y_values[-1], c="purple", marker="o", label="End")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Robot Trajectory")
        plt.legend(["Trajectory", "Odometry", "Odometry start", "Odometry end"])
        plt.tight_layout()
        plt.savefig(filename)

    def load_trajectory(self, filename: str):
        self.trajectory.clear()
        with open(filename, "r") as f:
            path = Path()
            for line in f:
                x_str, y_str = line.strip().split(",")
                pose = PoseStamped()
                pose.pose.position.x = float(x_str)
                pose.pose.position.y = float(y_str)
                path.poses.append(pose)
            self.trajectory.append(path)

    def load_odometry(self, filename: str):
        self.odometry_history.clear()
        with open(filename, "r") as f:
            for line in f:
                x_str, y_str = line.strip().split(",")
                odom = Odometry()
                odom.pose.pose.position.x = float(x_str)
                odom.pose.pose.position.y = float(y_str)
                self.odometry_history.append(odom)

    def plot_trajectory(self, filename: str, include_odometry: bool = False):
        x_values: list[float] = []
        y_values: list[float] = []
        for path in self.trajectory:
            for pose in path.poses:
                x_values.append(pose.pose.position.x)
                y_values.append(pose.pose.position.y)

        if include_odometry:
            odom_x_values: list[float] = []
            odom_y_values: list[float] = []
            for odom in self.odometry_history:
                odom_x_values.append(odom.pose.pose.position.x)
                odom_y_values.append(odom.pose.pose.position.y)

        plt.scatter(x_values, y_values)
        if include_odometry:
            plt.scatter(odom_x_values, odom_y_values, c="red", marker="x")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Robot Trajectory")
        plt.legend(["Trajectory", "Odometry"] if include_odometry else ["Trajectory"])
        plt.savefig(filename)


if __name__ == "__main__":
    recorder = TrajectoryRecorder()
    recorder.load_trajectory("export/kris_robot2_trajectory_1.txt")
    recorder.load_odometry("export/kris_robot2_odometry_1.txt")
    recorder.plot_movement("export/kris_robot2_trajectory_1_plot.png")
