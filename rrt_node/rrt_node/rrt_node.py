import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.task import Future
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node, Publisher, Subscription
from action_msgs.msg import GoalStatus
from rclpy.timer import Timer
from std_srvs.srv import Trigger
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)  # correct map QoS profile
import math
import random


class RRTStar(Node):
    def __init__(self):
        super().__init__("rrt_node")

        # ----- Parameters -----
        self.declare_parameter("robot_name", "kris_robot1")
        self.declare_parameter("tree_update_interval", 5.0)

        # ----- Variables -----
        self.robot_name: str = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.current_map: OccupancyGrid = None
        self.current_odom: Odometry = None
        self.map_frame_id: str = f"{self.robot_name}_map"
        self.action_server_connected: bool = False
        self.goal_future: Future = None
        self.moving: bool = False

        # ----- Frontier exploration state -----
        self.current_heading: float = 0.0  # radians, direction of last movement
        self.visited_frontiers: list[tuple[float, float]] = (
            []
        )  # recently reached frontiers
        self.visited_memory: int = 12  # how many past frontiers to remember
        self.consecutive_failures: int = 0  # how many times planning failed in a row

        map_qos: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Clients -----
        self.nav_client: ActionClient = ActionClient(
            self, NavigateToPose, f"{self.robot_name}/navigate_to_pose"
        )

        # ----- Services -----
        # self.save_model_service = self.create_service(
        #     Trigger,
        #     "save_model",
        #     self.save_all_models,
        # )
        # ros2 service call /save_model std_srvs/srv/Trigger

        # ----- Timers -----
        self.tree_update_timer: Timer = self.create_timer(
            self.get_parameter("tree_update_interval")
            .get_parameter_value()
            .double_value,
            callback=self.update_tree,
        )

        # ----- Subscribers -----
        self.map_sub: Subscription[OccupancyGrid] = self.create_subscription(
            OccupancyGrid,
            f"/{self.robot_name}/map",
            self.map_callback,
            map_qos,
        )

        self.odom_sub: Subscription[Odometry] = self.create_subscription(
            Odometry,
            f"/{self.robot_name}/odom",
            self.odom_callback,
            10,
        )

        # ----- Publishers -----
        self.point_pub: Publisher[PointStamped] = self.create_publisher(
            PointStamped,
            f"/{self.robot_name}/goal_marker",
            10,
        )

        self.goal_pub: Publisher[PoseStamped] = self.create_publisher(
            PoseStamped,
            f"/{self.robot_name}/goal_pose",
            10,
        )

        # ----- Initialization -----
        self.get_logger().info("Attempting to connect to action server")
        result: bool = self.nav_client.wait_for_server(timeout_sec=10.0)

        if result:
            self.get_logger().info("Successfully connected to action server")
            self.action_server_connected = True
        else:
            self.get_logger().error("Failed to connect to action server within timeout")

        self.get_logger().info("RRT node initialized")

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    def update_tree(self):
        if self.current_map is None or self.current_odom is None:
            self.get_logger().warn("Waiting for map and odometry data...")
            return

        if self.is_moving:
            return

        map_info = self.current_map.info
        width = int(map_info.width)
        height = int(map_info.height)

        if width <= 0 or height <= 0:
            self.get_logger().warn("Map has invalid dimensions")
            return

        robot_x = float(self.current_odom.pose.pose.position.x)
        robot_y = float(self.current_odom.pose.pose.position.y)

        robot_cell = self.world_to_grid(robot_x, robot_y)
        start_x, start_y = robot_x, robot_y

        if not self.is_cell_free(robot_cell[0], robot_cell[1]):
            self.get_logger().warn(
                "Robot is not in a free cell, searching for nearest safe position..."
            )
            safe_cell = self.find_nearest_free_cell(
                robot_cell[0], robot_cell[1], search_radius=15
            )
            if safe_cell is None:
                self.get_logger().error("No free cell found nearby, cannot plan")
                return
            start_x, start_y = self.grid_to_world(safe_cell[0], safe_cell[1])
            self.get_logger().info(
                f"Using nearby free cell as start: ({start_x:.2f}, {start_y:.2f})"
            )
            # Publish recovery goal to move robot to safe cell
            self.publish_point(start_x, start_y)
            self.publish_goal(start_x, start_y)
            return

        frontiers = self.detect_frontiers()
        if len(frontiers) == 0:
            self.get_logger().info(
                "No frontiers detected — environment may be fully mapped"
            )
            return

        # ----- Greedy Nearest-Frontier with Directional Momentum -----
        # Scoring weights
        w_distance = 1.0  # prefer reachable frontiers (not too far)
        w_momentum = 2.0  # strong preference for continuing in same heading
        w_visited = 3.0  # heavy penalty for revisiting explored areas
        min_dist = 0.3  # ignore frontiers closer than this (already there)
        max_dist = 8.0  # ignore frontiers further than this (too far to be useful now)
        visited_penalty_radius = 1.5  # radius within which a frontier is "visited"

        # On repeated failures, relax the distance cap to escape local dead-ends
        if self.consecutive_failures >= 3:
            max_dist = 20.0
            self.get_logger().warn(
                f"Relaxing range limit after {self.consecutive_failures} failures"
            )

        best_frontier = None
        best_score = float("-inf")

        for fx, fy in frontiers:
            dist = math.hypot(fx - start_x, fy - start_y)

            if dist < min_dist or dist > max_dist:
                continue

            # Quick collision check along the straight line to frontier
            if not self.is_segment_collision_free((start_x, start_y), (fx, fy)):
                continue

            # --- Distance score: prefer moderate distances (not trivially close) ---
            distance_score = 1.0 / (1.0 + dist)

            # --- Directional momentum score: prefer frontiers aligned with heading ---
            dx = fx - start_x
            dy = fy - start_y
            frontier_angle = math.atan2(dy, dx)
            angle_diff = abs(
                math.atan2(
                    math.sin(frontier_angle - self.current_heading),
                    math.cos(frontier_angle - self.current_heading),
                )
            )
            # cos maps 0 → 1.0 (perfect alignment), pi → -1.0 (backwards)
            momentum_score = math.cos(angle_diff)

            # --- Visited penalty: penalize frontiers near already-explored areas ---
            visited_penalty = 0.0
            for vx, vy in self.visited_frontiers:
                if math.hypot(fx - vx, fy - vy) < visited_penalty_radius:
                    visited_penalty += 1.0

            score = (
                w_distance * distance_score
                + w_momentum * momentum_score
                - w_visited * visited_penalty
            )

            if score > best_score:
                best_score = score
                best_frontier = (fx, fy)

        # Fallback: if no frontier passed filters, drop distance cap and take nearest
        if best_frontier is None:
            reachable = [
                (fx, fy)
                for fx, fy in frontiers
                if math.hypot(fx - start_x, fy - start_y) > min_dist
                and self.is_segment_collision_free((start_x, start_y), (fx, fy))
            ]
            if not reachable:
                self.consecutive_failures += 1
                self.get_logger().warn(
                    f"No reachable frontier found (failure #{self.consecutive_failures})"
                )
                return
            best_frontier = min(
                reachable, key=lambda f: math.hypot(f[0] - start_x, f[1] - start_y)
            )

        self.consecutive_failures = 0
        goal_x, goal_y = best_frontier

        # Update heading to match direction of new goal
        self.current_heading = math.atan2(goal_y - start_y, goal_x - start_x)

        # Record this frontier as visited
        self.visited_frontiers.append((goal_x, goal_y))
        if len(self.visited_frontiers) > self.visited_memory:
            self.visited_frontiers.pop(0)

        self.get_logger().info(
            f"Frontier goal: ({goal_x:.2f}, {goal_y:.2f}), "
            f"heading: {math.degrees(self.current_heading):.1f}°, "
            f"score: {best_score:.3f}"
        )

        self.publish_point(goal_x, goal_y)
        self.publish_goal(goal_x, goal_y)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        info = self.current_map.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        info = self.current_map.info
        wx = info.origin.position.x + (gx + 0.5) * info.resolution
        wy = info.origin.position.y + (gy + 0.5) * info.resolution
        return wx, wy

    def is_in_bounds(self, gx: int, gy: int) -> bool:
        width = int(self.current_map.info.width)
        height = int(self.current_map.info.height)
        return 0 <= gx < width and 0 <= gy < height

    def get_cell_value(self, gx: int, gy: int) -> int:
        width = int(self.current_map.info.width)
        index = gy * width + gx
        return int(self.current_map.data[index])

    def is_cell_free(self, gx: int, gy: int) -> bool:
        if not self.is_in_bounds(gx, gy):
            return False

        value = self.get_cell_value(gx, gy)
        # free=0, unknown=-1, occupied>=50
        return value == 0

    def is_segment_collision_free(
        self, start: tuple[float, float], end: tuple[float, float]
    ) -> bool:
        sx, sy = self.world_to_grid(start[0], start[1])
        ex, ey = self.world_to_grid(end[0], end[1])

        dx = ex - sx
        dy = ey - sy
        steps = max(abs(dx), abs(dy))
        if steps == 0:
            return self.is_cell_free(sx, sy)

        for i in range(steps + 1):
            t = i / float(steps)
            gx = int(round(sx + t * dx))
            gy = int(round(sy + t * dy))
            if not self.is_cell_free(gx, gy):
                return False

        return True

    def detect_frontiers(self) -> list[tuple[float, float]]:
        width = int(self.current_map.info.width)
        height = int(self.current_map.info.height)

        frontiers: list[tuple[float, float]] = []

        for gy in range(1, height - 1):
            for gx in range(1, width - 1):
                if self.get_cell_value(gx, gy) != 0:
                    continue

                has_unknown_neighbor = False
                has_occupied_neighbor = False
                for ny in range(gy - 1, gy + 2):
                    for nx in range(gx - 1, gx + 2):
                        if nx == gx and ny == gy:
                            continue
                        v = self.get_cell_value(nx, ny)
                        if v == -1:
                            has_unknown_neighbor = True
                        elif v >= 50:
                            has_occupied_neighbor = True

                if has_unknown_neighbor and not has_occupied_neighbor:
                    frontiers.append(self.grid_to_world(gx, gy))

        # Light subsampling for performance
        if len(frontiers) > 400:
            frontiers = random.sample(frontiers, 400)

        return frontiers

    def find_nearest_free_cell(
        self, gx: int, gy: int, search_radius: int = 15
    ) -> tuple[int, int] | None:
        """
        Find the nearest free cell to the given grid position using BFS.
        Returns (gx, gy) of the nearest free cell, or None if not found.
        """
        from collections import deque

        visited = set()
        queue = deque([(gx, gy, 0)])
        visited.add((gx, gy))

        while queue:
            cx, cy, dist = queue.popleft()

            if dist > search_radius:
                break

            if self.is_in_bounds(cx, cy) and self.is_cell_free(cx, cy):
                return (cx, cy)

            for dx, dy in [
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            ]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))

        return None

    def publish_point(self, x: float, y: float):
        point_msg: PointStamped = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.map_frame_id
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        self.point_pub.publish(point_msg)

    def publish_goal(self, x: float, y: float, theta: float = 0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        if theta == 0.0:  # compute theta based on current position and goal
            dx: float = x - self.current_odom.pose.pose.position.x
            dy: float = y - self.current_odom.pose.pose.position.y
            theta: float = math.atan2(dy, dx)

        # Convert theta to quaternion for orientation
        qz: float = math.sin(theta / 2.0)
        qw: float = math.cos(theta / 2.0)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.last_goal = msg

        if not self.action_server_connected:
            self.get_logger().warn(
                "Published goal, but action server is not connected, cannot receive completion from nav stack!"
            )
            self.goal_pub.publish(msg)

        else:
            self.goal_future = self.nav_client.send_goal_async(
                NavigateToPose.Goal(pose=msg)
            )
            self.goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(
                f"Published new goal for {self.robot_name} [{x}, {y}, {theta}]"
            )
            self.moving = True

    def goal_response_callback(self, future: Future):
        result = future.result()  # get result of sending the goal
        result_future = result.get_result_async()
        result_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future: Future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:  # SUCCEEDED
            self.get_logger().info(f"Goal completed successfully for {self.robot_name}")
        elif result.status == GoalStatus.STATUS_ABORTED:  # ABORTED
            self.get_logger().warn(f"Goal was aborted for {self.robot_name}")
        else:
            self.get_logger().warn(
                f"Goal failed with status {result.status} for {self.robot_name}"
            )

        self.moving = False  # goal is done, so we are no longer moving

    @property
    def is_moving(self) -> bool:
        return self.moving


def main():
    rclpy.init()
    rrt_node = RRTStar()
    while rclpy.ok():
        rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
