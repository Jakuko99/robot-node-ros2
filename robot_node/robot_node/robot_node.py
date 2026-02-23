import rclpy
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.task import Future
from rclpy.timer import Timer
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node, Publisher, Subscription
from nav_msgs.msg import OccupancyGrid, Odometry
from action_msgs.msg import GoalStatus
from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from time import time
import math
import numpy as np

from robot_node.exploration_actor import (
    ActionOutput,
    MapMeta,
    OccupancyGridData,
    RobotKinematics,
    SwarmExplorer,
    SwarmObservation,
)


class RobotNode(Node):
    def __init__(self):
        super().__init__(f"robot_node")

        # ----- Parameters ------
        self.declare_parameter("robot_name", "kris_robot1")
        self.declare_parameter("goal_process_interval", 5.0)
        self.declare_parameter("robot_discovery_interval", 10.0)
        self.declare_parameter("position_update_interval", 1.0)
        self.declare_parameter("goal_timeout", 60.0)
        self.declare_parameter("marl_training", True)
        self.declare_parameter("marl_update_every", 32)
        self.declare_parameter("marl_update_epochs", 4)
        self.declare_parameter("model_checkpoint_path", "/tmp/marl_checkpoint.pt")

        self.namespace: str = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.goal_process_interval: float = (
            self.get_parameter("goal_process_interval")
            .get_parameter_value()
            .double_value
        )
        self.robot_discovery_interval: float = (
            self.get_parameter("robot_discovery_interval")
            .get_parameter_value()
            .double_value
        )
        self.position_update_interval: float = (
            self.get_parameter("position_update_interval")
            .get_parameter_value()
            .double_value
        )
        self.goal_timeout: float = (
            self.get_parameter("goal_timeout").get_parameter_value().double_value
        )
        self.marl_training: bool = (
            self.get_parameter("marl_training").get_parameter_value().bool_value
        )
        self.marl_update_every: int = (
            self.get_parameter("marl_update_every").get_parameter_value().integer_value
        )
        self.marl_update_epochs: int = (
            self.get_parameter("marl_update_epochs").get_parameter_value().integer_value
        )
        self.model_checkpoint_path: str = (
            self.get_parameter("model_checkpoint_path")
            .get_parameter_value()
            .string_value
        )

        # ----- Variables -----
        self.goal_future: Future = None
        self.goal_handle: ClientGoalHandle = None
        self.last_odom: Odometry = None
        self.last_cmd_vel: Twist = None
        self.last_goal: PoseStamped = None
        self.current_map: OccupancyGrid = None
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.moving: bool = False
        self.action_server_connected: bool = False
        self.other_nodes: list[str] = []  # keep track of other robot nodes
        self.last_position_update_time: float = time()
        self.goal_publish_time: float = time()
        self.node_positions: dict[str, tuple[float, float]] = (
            {}
        )  # store positions of nodes
        self.swarm_explorer: SwarmExplorer = None
        self.previous_observation: SwarmObservation = None
        self.previous_action_output: ActionOutput = None
        map_qos: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )  # correct map QoS profile

        # ----- Timers -----
        self.process_goal_timer: Timer = self.create_timer(
            self.goal_process_interval,
            callback=self.process_goal_callback,
        )

        self.robot_discovery_timer: Timer = self.create_timer(
            self.robot_discovery_interval,
            callback=self.robot_discovery_callback,
        )

        # ----- Clients -----
        self.nav_client: ActionClient = ActionClient(
            self, NavigateToPose, f"{self.namespace}/navigate_to_pose"
        )

        # ----- Sevices -----
        self.save_service: Trigger = self.create_service(
            Trigger, f"{self.get_name()}/save_checkpoint", self.save_checkpoint_callback
        )
        # ros2 service call /<node_name>/save_checkpoint std_srvs/srv/Trigger

        # ----- Subscribers -----
        self.odom_sub: Subscription[Odometry] = self.create_subscription(
            Odometry,
            f"{self.namespace}/odom",
            self.odom_callback,
            10,
        )

        self.odom_subs: list[Subscription[Odometry]] = (
            []
        )  # subscribe to other robots' odom topics for position tracking

        self.map_sub: Subscription[OccupancyGrid] = self.create_subscription(
            OccupancyGrid,
            f"{self.namespace}/map",
            self.map_callback,
            map_qos,
        )

        # ----- Publishers -----
        self.goal_pub: Publisher[PoseStamped] = self.create_publisher(
            PoseStamped, f"{self.namespace}/goal_pose", 10
        )

        # ----- Initialization -----
        self.get_logger().info("Attempting to connect to action server")
        result: bool = self.nav_client.wait_for_server(timeout_sec=10.0)

        if result:
            self.get_logger().info("Successfully connected to action server")
            self.action_server_connected = True
        else:
            self.get_logger().error("Failed to connect to action server within timeout")

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2.0 * math.atan2(qz, qw)

    def other_node_odom_callback(self, msg: Odometry):
        # Update position of other robot nodes
        if time() - self.last_position_update_time < self.position_update_interval:
            return

        node_name = msg.header.frame_id.replace(
            "_odom", "_node"
        )  # extract node name from frame_id
        self.node_positions[node_name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        self.get_logger().debug(
            f"Updated position for {node_name}: {self.node_positions[node_name]}"
        )
        self.last_position_update_time = time()

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg

    def save_checkpoint_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        if self.swarm_explorer is not None:
            self.swarm_explorer.save_checkpoint(self.model_checkpoint_path)
            response.success = True
            response.message = f"Checkpoint saved to {self.model_checkpoint_path}"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "SwarmExplorer not initialized, cannot save checkpoint"
            self.get_logger().warn(response.message)

        return response

    def publish_goal(self, x: float, y: float, theta: float = 0.0):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = f"{self.namespace}_map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        if theta == 0.0:  # compute theta based on current position and goal
            dx: float = x - self.x
            dy: float = y - self.y
            theta: float = math.atan2(dy, dx)

        # Convert theta to quaternion for orientation
        qz: float = math.sin(theta / 2.0)
        qw: float = math.cos(theta / 2.0)
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        self.last_goal = goal_pose

        if not self.action_server_connected:
            self.get_logger().warn(
                "Published goal, but action server is not connected, cannot receive completion from nav stack!"
            )
            self.goal_pub.publish(goal_pose)

        else:
            self.goal_future = self.nav_client.send_goal_async(
                NavigateToPose.Goal(pose=goal_pose)
            )
            self.goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"Published new goal [{x}, {y}, {theta}]")
            self.goal_publish_time = time()
            self.moving = True

    def goal_response_callback(self, future: Future):
        self.goal_handle: ClientGoalHandle = future.result()  # goal result
        result_future: Future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future: Future):
        result: ClientGoalHandle = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:  # SUCCEEDED
            self.get_logger().info(f"Goal completed successfully")
        elif result.status in [
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        ]:  # ABORTED or CANCELED
            self.get_logger().warn(f"Goal was abandoned")
        else:
            self.get_logger().error(f"Goal failed with status {result.status}")

        self.moving = False  # goal is done, so we are no longer moving

    def process_goal_callback(self):
        if self.current_map is None or self.last_odom is None:
            return  # no data yet to make decisions with

        if (
            self.is_moving
            and self.action_server_connected
            and time() - self.goal_publish_time > self.goal_timeout
        ):
            self.get_logger().warn(f"Goal timeout exceeded, canceling goal")
            self.nav_client._cancel_goal_async(self.goal_handle)
            self.moving = False
            return  # robot is taking too long to reach goal

        if self.is_moving:
            return  # robot is moving towards goal

        robot_names = self._get_swarm_robot_names()
        if len(robot_names) == 0:
            return  # no other robots found

        if self.swarm_explorer is None and len(self.other_nodes) >= 1:
            self.swarm_explorer = SwarmExplorer(
                robot_names=robot_names,
                checkpoint_path=self.model_checkpoint_path,
                direction_consistency_bonus=2.0,
            )
            self.previous_observation = None
            self.previous_action_output = None
            self.get_logger().info(
                f"Initialized SwarmExplorer for robots: {robot_names}"
            )

        if self.swarm_explorer is None:
            return

        current_observation = self._build_swarm_observation(robot_names)
        if current_observation is None:
            return

        if (
            self.marl_training
            and self.previous_observation is not None
            and self.previous_action_output is not None
        ):
            metrics = self.swarm_explorer.training_loop_step(
                prev_observation=self.previous_observation,
                next_observation=current_observation,
                action_output=self.previous_action_output,
                done=False,
                update_every=self.marl_update_every,
                epochs=self.marl_update_epochs,
            )
            if metrics["actor_loss"] != 0.0 or metrics["critic_loss"] != 0.0:
                self.get_logger().info(
                    "MARL update "
                    f"reward={metrics['reward']:.3f}, "
                    f"actor_loss={metrics['actor_loss']:.4f}, "
                    f"critic_loss={metrics['critic_loss']:.4f}, "
                    f"entropy={metrics['entropy']:.4f}"
                )
                self.swarm_explorer.save_checkpoint(self.model_checkpoint_path)

        action_output = self.swarm_explorer.select_frontier_actions(current_observation)
        self.previous_observation = current_observation
        self.previous_action_output = action_output

        if self.namespace not in action_output.target_points:
            return

        target_x, target_y = action_output.target_points[self.namespace]
        action_index = action_output.action_indices.get(self.namespace, -1)

        if action_index < 0:
            self.get_logger().debug("No valid frontier cluster available for this step")
            return

        self.publish_goal(float(target_x), float(target_y))

    def _get_swarm_robot_names(self) -> list[str]:
        discovered = []
        for node_name in sorted(self.node_positions.keys()):
            if node_name.endswith("_node"):
                discovered.append(node_name.replace("_node", ""))
        return [self.namespace] + [
            name for name in discovered if name != self.namespace
        ]

    def _build_swarm_observation(
        self, robot_names: list[str]
    ) -> SwarmObservation | None:
        if self.current_map is None or self.last_odom is None:
            return None

        width = int(self.current_map.info.width)
        height = int(self.current_map.info.height)
        if width <= 0 or height <= 0:
            return None

        try:
            grid = np.array(self.current_map.data, dtype=np.int16).reshape(
                height, width
            )
        except ValueError:
            self.get_logger().warn(
                "Failed to reshape occupancy grid for SwarmObservation"
            )
            return None

        meta = MapMeta(
            resolution=float(self.current_map.info.resolution),
            origin_x=float(self.current_map.info.origin.position.x),
            origin_y=float(self.current_map.info.origin.position.y),
        )
        map_data = OccupancyGridData(grid=grid, meta=meta)

        robot_states: dict[str, RobotKinematics] = {}
        robot_states[self.namespace] = RobotKinematics(
            x=float(self.x),
            y=float(self.y),
            yaw=float(self.theta),
            vx=float(self.last_odom.twist.twist.linear.x),
            vy=float(self.last_odom.twist.twist.linear.y),
        )

        for robot_name in robot_names:
            if robot_name == self.namespace:
                continue

            node_key = f"{robot_name}_node"
            if node_key not in self.node_positions:
                continue

            other_x, other_y = self.node_positions[node_key]
            robot_states[robot_name] = RobotKinematics(
                x=float(other_x),
                y=float(other_y),
                yaw=0.0,
                vx=0.0,
                vy=0.0,
            )

        missing = [name for name in robot_names if name not in robot_states]
        if missing:
            self.get_logger().debug(f"Missing states for robots: {missing}")
            return None

        return SwarmObservation(map_data=map_data, robot_states=robot_states)

    def robot_discovery_callback(self):
        nodes: list[str] = self.get_node_names()
        topics: list[str] = [topic[0] for topic in self.get_topic_names_and_types()]

        for node in nodes:
            if (
                node.endswith("_node")
                and node != self.get_name()
                and node not in self.other_nodes
                and f"/{node.replace('_node', '')}/odom"
                in topics  # prevent subscribing to non-robot nodes that happen to end with _node
            ):
                if node not in [sub.topic_name for sub in self.odom_subs]:
                    odom_topic = f"/{node.replace('_node', '')}/odom"
                    self.get_logger().info(
                        f"{self.namespace} discovered new robot node: {node}, subscribing to {odom_topic}"
                    )
                    new_sub = self.create_subscription(
                        Odometry, odom_topic, self.other_node_odom_callback, 10
                    )  # subscribe to other robots odom topic
                    self.odom_subs.append(new_sub)
                    self.other_nodes.append(node)

    @property
    def is_moving(self) -> bool:
        return self.moving

    def get_local_map(self) -> OccupancyGrid:
        if self.current_map:
            return self.current_map

        self.get_logger().warn(f"No map received yet, returning empty map")
        return OccupancyGrid()


def main():
    rclpy.init()
    robot_node = RobotNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(robot_node)
    except KeyboardInterrupt:
        if robot_node.swarm_explorer is not None:
            robot_node.swarm_explorer.save_checkpoint(robot_node.model_checkpoint_path)
            robot_node.get_logger().info(
                f"Saved MARL checkpoint to {robot_node.model_checkpoint_path}"
            )
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
