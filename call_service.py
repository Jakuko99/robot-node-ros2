import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy import Future
from sim_srvs.srv import SimulationOutput, ExplorationStatus

rclpy.init()
node: Node = rclpy.create_node("training_launcher")
cli1: Client = node.create_client(SimulationOutput, "/kris_robot1/save_model")
cli2: Client = node.create_client(ExplorationStatus, "/kris_robot1/exploration_progress")

# while not cli1.wait_for_service(timeout_sec=1.0):
#     print(f"Waiting for service {cli1.srv_name} to become available...")

# try:
#     req: SimulationOutput.Request = SimulationOutput.Request(id=0)
#     future: Future = cli1.call_async(req)
#     rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
#     if future.result() is not None:
#         print(f"Service call to {cli1.srv_name} succeeded: {future.result().message}")

#     else:
#         print(f"Service call to {cli1.srv_name} failed")
# except Exception as e:
#     print(f"Service call to {cli1.srv_name} failed: {e}")

while not cli2.wait_for_service(timeout_sec=1.0):
    print(f"Waiting for service {cli2.srv_name} to become available...")

try:
    req: ExplorationStatus.Request = ExplorationStatus.Request()
    future: Future = cli2.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    if future.result() is not None:
        print(
            f"Service call to {cli2.srv_name} succeeded:",
            f"Exploration ratio: {future.result().explore_ratio:.2f}",
            f"Overlap ratio: {future.result().overlap_ratio:.2f}",
        )

    else:
        print(f"Service call to {cli2.srv_name} failed")
except Exception as e:
    print(f"Service call to {cli2.srv_name} failed: {e}")
