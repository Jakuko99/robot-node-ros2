#include "robot_control.hpp"

using std::placeholders::_1;
// #define DEBUG

RobotControl::RobotControl(std::string node_name, std::shared_ptr<OccupancyGridProcessor> occupancy_processor)
    : rclcpp::Node(node_name), occupancy_grid_processor_(occupancy_processor)
{
  // Initialize publishers and subscribers
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&RobotControl::odom_callback, this, _1));
  odom_sub1 = this->create_subscription<nav_msgs::msg::Odometry>(
      "/kris_robot1/odom", 10, std::bind(&RobotControl::odom1_callback, this, _1));
  // map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  //     "/merge_map", 10, std::bind(&RobotControl::map_callback, this, _1));
  /*cmd_vel_pub1 = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  cmd_vel_pub2 = this->create_publisher<geometry_msgs::msg::Twist>("/kris_robot1/cmd_vel", 10);*/
  pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&RobotControl::goal_callback, this, _1));
  marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/robot_pos", 10);
  path_pub = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

  // path_planner.setDiagonalMovement(true);
  // path_planner.setHeuristic(AStar::Heuristic::manhattan);

  RCLCPP_INFO(this->get_logger(), "RobotControl node initialized");
}

RobotControl::~RobotControl() {}

void RobotControl::update_state()
{
  /*if (path_ready && (robot_position.x != last_clicked.x || robot_position.y != last_clicked.y))
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    // Calculate the direction to the clicked point
    double dx = last_clicked.x - robot_position.x;
    double dy = last_clicked.y - robot_position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > 0.1) // Threshold to avoid jittering
    {
      cmd_vel_msg.linear.x = dx / distance; // Normalize direction
      cmd_vel_msg.linear.y = dy / distance; // Normalize direction

      cmd_vel_pub1->publish(cmd_vel_msg);
    }
    else
    {
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.linear.y = 0.0;
      cmd_vel_pub1->publish(cmd_vel_msg);
      path_ready = false;
    }
  }*/
}

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

void RobotControl::odom1_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
#ifdef DEBUG
  RCLCPP_INFO(this->get_logger(), "Received odometry data from robot1: position (%f, %f), orientation (%f, %f, %f, %f)",
              msg->pose.pose.position.x, msg->pose.pose.position.y,
              msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
#endif
  // Publish a marker at the robot1's position
  publish_marker(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                 "kris_robot1", 1, "merge_map");

  robot1_position.x = msg->pose.pose.position.x;
  robot1_position.y = msg->pose.pose.position.y;
  robot1_position.orientation_x = msg->pose.pose.orientation.x;
  robot1_position.orientation_y = msg->pose.pose.orientation.y;
  robot1_position.orientation_z = msg->pose.pose.orientation.z;
  robot1_position.orientation_w = msg->pose.pose.orientation.w;
}

void RobotControl::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  /*#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "Received map with resolution: %f", msg->info.resolution);
  #endif
    if (path_planner.getWorldSize().x != msg->info.width ||
        path_planner.getWorldSize().y != msg->info.height) // update map size if it has changed
    {
      path_planner.setWorldSize(AStar::Vec2i{static_cast<int>(msg->info.width), static_cast<int>(msg->info.height)});
      RCLCPP_INFO(this->get_logger(), "Updated path planner world size to: (%d, %d)",
                  msg->info.width, msg->info.height);
    }

    // Clear previous walls
    //path_planner.clearCollisions();

    // Add new walls from the occupancy grid
    for (int y = 0; y < msg->info.height; ++y)
    {
      for (int x = 0; x < msg->info.width; ++x)
      {
        int index = x + y * msg->info.width;
        if (msg->data[index] > 50) // Assuming values > 50 are obstacles
        {
          path_planner.addCollision(AStar::Vec2i{x, y});
        }
      }
    }*/
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

  auto path = occupancy_grid_processor_->aStarPath(robot_pos, (geometry_msgs::msg::PoseStamped)*msg);
  occupancy_grid_processor_->publishPath(path);
  RCLCPP_INFO(this->get_logger(), "Path found with %zu points", path.size());
  /*this->path = path_planner.findPath(
      AStar::Vec2i{static_cast<int>(robot_position.x), static_cast<int>(robot_position.y)},
      AStar::Vec2i{static_cast<int>(last_clicked.x), static_cast<int>(last_clicked.y)});

  if (!this->path.empty() && path_ready == false)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "merge_map";
    path_msg.header.stamp = this->now();

    for (const auto &coord : this->path)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path_msg.header;
      pose_stamped.pose.position.x = coord.x;
      pose_stamped.pose.position.y = coord.y;
      pose_stamped.pose.position.z = 0.0;    // Assuming a flat plane
      pose_stamped.pose.orientation.w = 1.0; // Default orientation
      path_msg.poses.push_back(pose_stamped);
    }

    path_pub->publish(path_msg);
    path_ready = true;
    path_finished = false;
    RCLCPP_INFO(this->get_logger(), "Path ready with %zu points", this->path.size());
  }*/
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
