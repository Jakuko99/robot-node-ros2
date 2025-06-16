#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "navigation.hpp"

#define DEBUG
using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    std::cout << "Initializing Navigation node...\n";
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Navigation>("navigation_node");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();

    return 0;
}