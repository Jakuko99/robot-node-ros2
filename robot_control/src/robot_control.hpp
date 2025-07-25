// #include <cstdio>
#include <string>
#include <sstream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include <vector>

#include "occupancy_grid_processor.hpp"

using namespace std;

struct ClickedPoint
{
  double x;
  double y;
};

struct RobotPosition
{
  double x;
  double y;
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
};

class RobotControl : public rclcpp::Node
{
public:
  RobotControl(std::string, std::shared_ptr<OccupancyGridProcessor>);
  ~RobotControl();
  void update_state();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odom1_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void publish_marker(double x, double y, double z, string name, int32_t id, string frame_id = "map");

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub1;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

  /*rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub1;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub2;*/
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

  ClickedPoint last_clicked;
  RobotPosition robot_position;
  RobotPosition robot1_position;

  std::shared_ptr<OccupancyGridProcessor> occupancy_grid_processor_;
  /*AStar::Generator path_planner;
  AStar::CoordinateList path;*/
  bool path_ready = false;
  bool path_finished = false;
};