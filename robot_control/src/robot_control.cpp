#include "robot_control.hpp"

using std::placeholders::_1;
// #define DEBUG

RobotControl::RobotControl(std::string node_name, std::shared_ptr<OccupancyGridProcessor> occupancy_processor, std::string goal_topic, std::string marker_topic)
    : rclcpp::Node(node_name), occupancy_grid_processor_(occupancy_processor)
{
  // Initialize publishers and subscribers
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      occupancy_grid_processor_->odom_topic_, 10, std::bind(&RobotControl::odom_callback, this, _1));
  pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10, std::bind(&RobotControl::goal_callback, this, _1));
  marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);

  RCLCPP_INFO(this->get_logger(), "RobotControl node initialized");
}

RobotControl::~RobotControl() {}

void RobotControl::update_state() {}

void RobotControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
#ifdef DEBUG
  RCLCPP_INFO(this->get_logger(), "Received odometry data: position (%f, %f), orientation (%f, %f, %f, %f)",
              msg->pose.pose.position.x, msg->pose.pose.position.y,
              msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
#endif
  // Publish a marker at the robot's position
  publish_marker(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                 "kris_robot", 0, "merge_map");

  robot_position.x = msg->pose.pose.position.x; // save robot position
  robot_position.y = msg->pose.pose.position.y;
  robot_position.orientation_x = msg->pose.pose.orientation.x;
  robot_position.orientation_y = msg->pose.pose.orientation.y;
  robot_position.orientation_z = msg->pose.pose.orientation.z;
  robot_position.orientation_w = msg->pose.pose.orientation.w;
}

void RobotControl::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
#ifdef DEBUG
  RCLCPP_INFO(this->get_logger(), "Received goal: position (%f, %f), orientation (%f, %f, %f, %f)",
              msg->pose.position.x, msg->pose.position.y,
              msg->pose.orientation.x, msg->pose.orientation.y,
              msg->pose.orientation.z, msg->pose.orientation.w);
#endif
  last_clicked.x = msg->pose.position.x;
  last_clicked.y = msg->pose.position.y;

  geometry_msgs::msg::PoseStamped robot_pos = geometry_msgs::msg::PoseStamped();
  robot_pos.pose.position.x = robot_position.x;
  robot_pos.pose.position.y = robot_position.y;
  robot_pos.pose.position.z = 0.0;
  robot_pos.pose.orientation.x = robot_position.orientation_x;
  robot_pos.pose.orientation.y = robot_position.orientation_y;
  robot_pos.pose.orientation.z = robot_position.orientation_z;
  robot_pos.pose.orientation.w = robot_position.orientation_w;
  robot_pos.header.frame_id = "merge_map";
  robot_pos.header.stamp = this->now();

  auto path = occupancy_grid_processor_->aStarPath(robot_pos, last_clicked);
  occupancy_grid_processor_->publishPath(path);
  RCLCPP_INFO(this->get_logger(), "Path found with %zu points", path.size());
}

void RobotControl::publish_marker(double x, double y, double z, string name, int32_t id, string frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = this->now();
  marker.ns = name;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker_pub->publish(marker);
}
