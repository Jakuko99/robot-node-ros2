#include "navigation.hpp"

using std::placeholders::_1;
#define DEBUG

Navigation::Navigation(std::string node_name) : Node(node_name)
{
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd", 10);
    cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Navigation::receive_cmd, this, _1));
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "laser_scan", 10, std::bind(&Navigation::receive_scan, this, _1));
    RCLCPP_INFO(this->get_logger(), "Navigation node initialized with name: %s", node_name.c_str());
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Navigation::receive_odom, this, _1));
}
Navigation::~Navigation()
{
}
void Navigation::update_state()
{
    // This function can be used to update the state of the navigation node
    // For now, it does nothing
}
void Navigation::publish_cmd_vel()
{
    // This function can be used to publish the cmd_vel message
    // For now, it does nothing
}

void Navigation::receive_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // This function receives the cmd message and publishes it to cmd_vel
    if (msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received cmd: linear.x: %f, angular.z: %f", msg->linear.x, msg->angular.z);
        cmd_vel_pub->publish(*msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received null cmd message");
    }
}

void Navigation::receive_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // This function receives the scan message and processes it
    if (msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received scan with %zu ranges", msg->ranges.size());
        // Process the scan data here
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received null scan message");
    }
}

void Navigation::receive_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // This function receives the odometry message and processes it
    if (msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received odom: position.x: %f, position.y: %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
        // Process the odometry data here
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received null odom message");
    }
}