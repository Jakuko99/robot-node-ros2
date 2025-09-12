#include <string>
#include <sstream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

struct RobotPosition
{
  double x;
  double y;
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
};

using Frontier = std::vector<std::pair<double, double>>;

class RobotControl : public rclcpp::Node
{
public:
  RobotControl(std::string node_name, std::string goal_topic, std::string odom_topic, std::string map_topic, std::string map_frame = "map");
  ~RobotControl();
  void update_state();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  std::vector<Frontier> get_frontiers(const nav_msgs::msg::OccupancyGrid::SharedPtr map, double cluster_distance = 0.5);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

  RobotPosition robot_position;  
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map;
  std::string map_frame_id;
  std::vector<Frontier> frontiers;
  geometry_msgs::msg::PoseStamped last_pose_msg;

  bool map_received = false;
  bool robot_moving = false;
};