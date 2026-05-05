from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import math
import json
import cv2


class TrajectoryRecorder:
    def __init__(self, static_transform_x: float = 0.0, static_transform_y: float = 0.0):
        self.trajectory: list[Path] = []
        self.odometry_history: list[Odometry] = []
        self.nav_goals: list[PoseStamped] = []
        self.timeouted_goals: list[PoseStamped] = []
        self.static_transform_x: float = static_transform_x
        self.static_transform_y: float = static_transform_y

    def track_route(self, msg: Path):
        self.trajectory.append(msg)

        self.nav_goals.append(msg.poses[-1])  # track the final pose of each path as a goal

    def goal_timeout(self):
        try:
            self.timeouted_goals.append(self.nav_goals[-1])  # track the last goal as timeouted
        except IndexError:
            pass  # no goals to timeout

    def goal_timeout(self, goal: PoseStamped):
        self.timeouted_goals.append(goal)  # track the provided goal as timeouted

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
        trajectory_dict: dict[str, list] = {"paths": []}
        for path in self.trajectory:
            path_dict: dict[str, list] = {"points": []}
            for pose in path.poses:
                pose_dict: dict[str, float] = {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                }
                path_dict["points"].append(pose_dict)

            path_dict["start_point"] = {
                "x": path.poses[0].pose.position.x,
                "y": path.poses[0].pose.position.y,
            }
            path_dict["end_point"] = {
                "x": path.poses[-1].pose.position.x,
                "y": path.poses[-1].pose.position.y,
            }

            trajectory_dict["paths"].append(path_dict)

        with open(filename, "w") as f:
            json.dump(trajectory_dict, f, indent=2)

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

    @staticmethod
    def pos_to_map_index(
        map: OccupancyGrid,
        pos: list[float, float],
        static_transform_x: float = 0.0,
        static_transform_y: float = 0.0,
    ) -> tuple[int, int]:
        """
        Convert world coordinates to occupancy grid indices.
        Returns:
            (i, j) grid indices or None if out of bounds
        """
        # Apply static offset
        pos[0] += static_transform_x
        pos[1] += static_transform_y

        # Map origin
        origin_x = map.info.origin.position.x
        origin_y = map.info.origin.position.y

        # Extract yaw from quaternion
        q = map.info.origin.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Translate point into map frame
        dx = pos[0] - origin_x
        dy = pos[1] - origin_y

        # Rotate if needed
        if abs(theta) > 1e-6:
            cos_t = math.cos(-theta)
            sin_t = math.sin(-theta)
            mx = cos_t * dx - sin_t * dy
            my = sin_t * dx + cos_t * dy
        else:
            # No rotation → faster path
            mx = dx
            my = dy

        # Convert to grid indices
        resolution = map.info.resolution
        i = int(math.floor(mx / resolution))
        j = int(math.floor(my / resolution))

        # Bounds check
        if i < 0 or j < 0 or i >= map.info.width or j >= map.info.height:
            return None  # outside map

        return i, j

    def create_map_overlay(
        self,
        map: OccupancyGrid,
        filename: str,
        source: str = "trajectory",
        include_goals: bool = True,
    ):
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
                map_x, map_y = self.pos_to_map_index(
                    map,
                    [odom.pose.pose.position.x, odom.pose.pose.position.y],
                    static_transform_x=self.static_transform_x,
                    static_transform_y=self.static_transform_y,
                )
                if 0 <= map_x < map_width and 0 <= map_y < map_height:
                    map_image[map_y, map_x] = [255, 255, 0]

        elif source == "trajectory":
            for path in self.trajectory:
                for pose in path.poses:
                    map_x, map_y = self.pos_to_map_index(
                        map,
                        [pose.pose.position.x, pose.pose.position.y],
                        static_transform_x=self.static_transform_x,
                        static_transform_y=self.static_transform_y,
                    )
                    if 0 <= map_x < map_width and 0 <= map_y < map_height:
                        map_image[map_y, map_x] = [255, 255, 0]

        if include_goals:
            for goal in self.nav_goals:
                map_x, map_y = self.pos_to_map_index(
                    map,
                    [goal.pose.position.x, goal.pose.position.y],
                    static_transform_x=self.static_transform_x,
                    static_transform_y=self.static_transform_y,
                )
                if 0 <= map_x < map_width and 0 <= map_y < map_height:
                    map_image[map_y, map_x] = [255, 0, 255]

            for goal in self.timeouted_goals:
                map_x, map_y = self.pos_to_map_index(
                    map,
                    [goal.pose.position.x, goal.pose.position.y],
                    static_transform_x=self.static_transform_x,
                    static_transform_y=self.static_transform_y,
                )
                if 0 <= map_x < map_width and 0 <= map_y < map_height:
                    map_image[map_y, map_x] = [0, 172, 255]

        cv2.imwrite(filename, map_image)

    def plot_movement(self, filename: str):
        end_points_x: list[float] = []
        end_points_y: list[float] = []
        for path in self.trajectory:  # visualize goals
            end_points_x.append(path.poses[-1].pose.position.x + self.static_transform_x)
            end_points_y.append(path.poses[-1].pose.position.y + self.static_transform_y)

        odom_x_values: list[float] = []
        odom_y_values: list[float] = []
        for odom in self.odometry_history:
            odom_x_values.append(odom.pose.pose.position.x + self.static_transform_x)
            odom_y_values.append(odom.pose.pose.position.y + self.static_transform_y)

        plt.scatter(odom_x_values, odom_y_values, c="red", marker="x", label="Odometry")
        plt.scatter(odom_x_values[0], odom_y_values[0], c="lime", marker="o", label="Start")
        plt.scatter(odom_x_values[-1], odom_y_values[-1], c="purple", marker="o", label="End")
        plt.scatter(end_points_x, end_points_y, c="orange", marker="o", label="Goals")
        plt.scatter(
            [goal.pose.position.x + self.static_transform_x for goal in self.timeouted_goals],
            [goal.pose.position.y + self.static_transform_y for goal in self.timeouted_goals],
            c="black",
            marker="x",
            label="Timeouted Goals",
        )
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Robot Trajectory")
        plt.legend(["Odometry", "Odometry start", "Odometry end", "Goals", "Timeouted Goals"])
        plt.tight_layout()
        plt.savefig(filename)

    def load_trajectory(self, filename: str):
        self.trajectory.clear()
        with open(filename, "r") as f:
            data = json.load(f)
            for path_dict in data["paths"]:
                path = Path()
                for pose_dict in path_dict["points"]:
                    pose = PoseStamped()
                    pose.pose.position.x = pose_dict["x"]
                    pose.pose.position.y = pose_dict["y"]
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


if __name__ == "__main__":
    recorder = TrajectoryRecorder()
    recorder.load_trajectory("export/kris_robot1_trajectory_2.json")
    recorder.load_odometry("export/kris_robot1_odometry_2.txt")
    recorder.plot_movement("export/kris_robot1_trajectory_2_plot.png")
