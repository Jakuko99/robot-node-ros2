#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_control.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    std::cout << "Initializing RobotControl node...\n";
    rclcpp::init(argc, argv);    
    auto node = std::make_shared<RobotControl>("robot_control_node1", "/kris_robot1/goal_pose", "/kris_robot1/odom", "/map1", "map1");
    auto node1 = std::make_shared<RobotControl>("robot_control_node2", "/kris_robot2/goal_pose", "/kris_robot2/odom", "/map2", "map2");
    auto node2 = std::make_shared<RobotControl>("robot_control_node3", "/kris_robot3/goal_pose", "/kris_robot3/odom", "/map3", "map3");
    auto node3 = std::make_shared<RobotControl>("robot_control_node4", "/kris_robot4/goal_pose", "/kris_robot4/odom", "/map4", "map4");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->update_state();
        rclcpp::spin_some(node1);
        node1->update_state();
        rclcpp::spin_some(node2);
        node2->update_state();
        rclcpp::spin_some(node3);
        node3->update_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    rclcpp::shutdown();

    return 0;
}