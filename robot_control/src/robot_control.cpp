#include "robot_control.hpp"

using std::placeholders::_1;
// #define DEBUG

RobotControl::RobotControl(std::string node_name) : rclcpp::Node(node_name)
{
  RCLCPP_INFO(this->get_logger(), "RobotControl node initialized");
}

RobotControl::~RobotControl() {}

void RobotControl::update_state()
{
  // Update the robot's state here
}
