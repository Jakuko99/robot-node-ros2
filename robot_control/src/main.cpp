#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_control.hpp"
#include "occupancy_grid_processor.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    std::cout << "Initializing RobotControl node...\n";
    rclcpp::init(argc, argv);
    auto occupancy_processor = std::make_shared<OccupancyGridProcessor>();
    auto node = std::make_shared<RobotControl>("robot_control_node", occupancy_processor);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rclcpp::spin_some(occupancy_processor);
        node->update_state();
        occupancy_processor->followPath();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();

    return 0;
}