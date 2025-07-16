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
#include <vector>

using namespace std;

class RobotControl : public rclcpp::Node
{
public:
  RobotControl(std::string);
  ~RobotControl();
  void update_state();

private:
  vector<rclcpp::Node::SharedPtr> node_list;
};