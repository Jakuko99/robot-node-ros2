#include "kris_robot.hpp"

using std::placeholders::_1;
#define DEBUG

KRISRobot::KRISRobot(std::string node_name) : rclcpp::Node(node_name),
                                              x(0.0),
                                              y(0.0),
                                              theta(0.0),
                                              v_linear(0.0),
                                              v_angular(0.0)
{
  this->declare_parameter("robot_description", "");
  this->laser_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", rclcpp::QoS(10));
  this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
  this->joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));
  this->urdf_pub = this->create_publisher<std_msgs::msg::String>("robot_description", 10);
  this->tf_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("tf", rclcpp::QoS(10));
  this->cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10), std::bind(&KRISRobot::cmd_vel_callback, this, _1));
  publish_urdf();

  std::chrono::milliseconds rate(100);
  auto loopTimer = this->create_wall_timer(rate, std::bind(&KRISRobot::update_state, this));

  RCLCPP_INFO(this->get_logger(), "KRIS Robot node initialized");
}

KRISRobot::~KRISRobot() {}

void KRISRobot::publish_scan()
{
}

void KRISRobot::publish_odometry()
{
#ifdef DEBUG
  RCLCPP_DEBUG(this->get_logger(), "x = %f, y = %f, theta = %f\n", this->x, this->y, this->theta);
#endif

  nav_msgs::msg::Odometry msg = nav_msgs::msg::Odometry();
  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  msg.pose.pose.position.x = this->x;
  msg.pose.pose.position.y = this->y;
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, this->theta);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  msg.twist.twist.linear.x = this->v_linear;
  msg.twist.twist.linear.y = this->v_angular;

  this->odom_pub->publish(msg);
}

void KRISRobot::publish_urdf()
{
  std::ifstream urdf_file(this->get_parameter("robot_description").as_string());
  if (!urdf_file.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file");
    return;
  }
  std::stringstream buffer;
  buffer << urdf_file.rdbuf();
  urdf_file.close();

  auto message = std_msgs::msg::String();
  message.data = buffer.str();

  urdf_pub->publish(message);
}

void KRISRobot::publish_tf()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = "odom";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.translation.x = this->x;
  tf_msg.transform.translation.y = this->y;
  tf_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, this->theta);
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  this->tf_pub->publish(tf_msg);
}

void KRISRobot::publish_joint_state()
{
}

void KRISRobot::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
#ifdef DEBUG
  RCLCPP_DEBUG(this->get_logger(), "Recieved cmd_vel: linear.x = %f, angular.z = %f\n", msg->linear.x, msg->angular.z);
#endif
  this->x = msg->linear.x;
  this->y = msg->linear.y;
  this->theta = msg->angular.z;
}

void KRISRobot::update_state()
{
#ifdef DEBUG
  RCLCPP_DEBUG(this->get_logger(), "Updating node state");
#endif

  float dt = 0.1; // Time step
  this->theta += this->v_angular * dt;
  this->x += this->v_linear * cos(this->theta) * dt;
  this->y += this->v_linear * sin(this->theta) * dt;

  publish_odometry();
  publish_tf();
  publish_joint_state();
}