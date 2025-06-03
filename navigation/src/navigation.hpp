#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
// include cartographer somehow

class Navigation : public rclcpp::Node
{
public:
  Navigation(std::string);
  ~Navigation();
  void update_state();

private:
    void publish_cmd_vel();
    void receive_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
    void receive_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void receive_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
};