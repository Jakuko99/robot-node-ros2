#include <cstdio>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class KRISRobot
{
public:
  KRISRobot(std::string);
  ~KRISRobot();
  void publish_scan();
  void publish_odometry();
  void publish_joint_state();
  void handle_vel_msg(const geometry_msgs::msg::Twist::SharedPtr msg);
  void mainloop();
  rclcpp::Node::SharedPtr get_node();

private:
  float x;
  float y;
  float theta;
  float v_linear;
  float v_angular;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
};

inline rclcpp::Node::SharedPtr KRISRobot::get_node()
{
  return this->node;
}