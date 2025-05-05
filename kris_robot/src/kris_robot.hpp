// #include <cstdio>
#include <string>
#include <sstream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

class KRISRobot : public rclcpp::Node
{
public:
  KRISRobot(std::string);
  ~KRISRobot();
  void publish_scan();
  void publish_odometry();
  void publish_urdf();
  void publish_tf();
  void publish_joint_state();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void update_state();

private:
  float x;
  float y;
  float theta;
  float v_linear;
  float v_angular;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
};