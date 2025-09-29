#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_control.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    std::cout << "Initializing RobotControl node...\n";
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControl>("robot_control_node1", "/kris_robot1/goal_pose", "/kris_robot1/odom", "/kris_robot1/map", "kris_robot1_map");
    auto node1 = std::make_shared<RobotControl>("robot_control_node2", "/kris_robot2/goal_pose", "/kris_robot2/odom", "/kris_robot2/map", "kris_robot2_map");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->update_state();
        rclcpp::spin_some(node1);
        node1->update_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    rclcpp::shutdown();

    return 0;
}