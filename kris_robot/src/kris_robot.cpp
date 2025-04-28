#include "kris_robot.hpp"
#include <rclcpp/rclcpp.hpp>

#define DEBUG

KRISRobot::KRISRobot(std::string node_name)
{
  this->node = std::make_shared<rclcpp::Node>(node_name);
  this->laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", rclcpp::QoS(10));
  this->odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
  this->joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));
  this->cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10), [=](const geometry_msgs::msg::Twist::SharedPtr x)
      { handle_vel_msg(x); });
}

KRISRobot::~KRISRobot() {}

void KRISRobot::publish_scan()
{
}

void KRISRobot::publish_odometry()
{  
  #ifdef DEBUG
    printf("x = %f, y = %f, theta = %f\n", this->x, this->y, this->theta);
  #endif
}

void KRISRobot::publish_joint_state()
{
}

void KRISRobot::handle_vel_msg(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  printf("Recieved cmd_vel: linear.x = %f, angular.z = %f\n", msg->linear.x, msg->angular.z);
  this->x = msg->linear.x;
  this->y = msg->linear.y;
  this->theta = msg->angular.z;
}

void KRISRobot::mainloop()
{
  publish_scan();
  publish_odometry();
  publish_joint_state();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  KRISRobot node("kris_robot");
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    node.mainloop(); 
    rclcpp::spin_some(node.get_node());
    loop_rate.sleep();
  }
  printf("hello world kris_robot package\n");
  return 0;
}