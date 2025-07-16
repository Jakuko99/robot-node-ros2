#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_control.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    std::cout << "Initializing RobotControl node...\n";
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControl>("robot_control_node");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->update_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();

    return 0;
}