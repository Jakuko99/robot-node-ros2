from nav_msgs.msg import OccupancyGrid, Path
import matplotlib.pyplot as plt
import cv2


class TrajectoryRecorder:
    def __init__(self):
        self.trajectory: list[Path] = []

    def track_route(self, msg: Path):
        self.trajectory.append(msg)

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

    def create_map_overlay(self, map: OccupancyGrid, filename: str):
        # This function can be implemented to create a visual overlay of the trajectory on the map
        pass
