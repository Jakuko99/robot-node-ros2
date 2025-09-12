#include "robot_control.hpp"

using std::placeholders::_1;
// #define DEBUG

RobotControl::RobotControl(std::string node_name, std::string goal_topic, std::string odom_topic, std::string map_topic, std::string map_frame)
    : rclcpp::Node(node_name)
{
  // Initialize publishers and subscribers
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&RobotControl::odom_callback, this, _1));
  pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
  map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, 10, std::bind(&RobotControl::map_callback, this, _1));
  map_frame_id = map_frame;
  RCLCPP_INFO(this->get_logger(), "RobotControl node initialized");
}

RobotControl::~RobotControl() {}

void RobotControl::update_state()
{
  if (map_received)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    auto frontiers = get_frontiers(current_map);

    if (!frontiers.empty() && !robot_moving)
    {
      RCLCPP_INFO(this->get_logger(), "Found %zu frontiers", frontiers.size());
      // Select the first frontier for simplicity
      auto target = frontiers[0];
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = map_frame_id;
      pose_msg.pose.position.x = target[0].first;
      pose_msg.pose.position.y = target[0].second;
      pose_msg.pose.position.z = 0.0;

      if ((abs(last_pose_msg.pose.position.x - pose_msg.pose.position.x) < 0.5) &&
          (abs(last_pose_msg.pose.position.y - pose_msg.pose.position.y) < 0.5))
      {
        pose_msg.pose.position.x = frontiers[1][0].first;
        pose_msg.pose.position.y = frontiers[1][0].second; // use next point if same as last
        RCLCPP_INFO(this->get_logger(), "Same as last goal, switching to next point.");
      }

      pose_pub->publish(pose_msg);
      RCLCPP_INFO(this->get_logger(), "Published new goal: (%.2f, %.2f)", pose_msg.pose.position.x, pose_msg.pose.position.y);
      last_pose_msg = pose_msg; // Save the last published pose
      robot_moving = true;      // Set the moving flag
    }
  }
}

void RobotControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (robot_position.x == msg->pose.pose.position.x &&
      robot_position.y == msg->pose.pose.position.y)
  {
    robot_moving = false; // Robot has stopped
  }

  robot_position.x = msg->pose.pose.position.x; // save robot position
  robot_position.y = msg->pose.pose.position.y;
  robot_position.orientation_x = msg->pose.pose.orientation.x;
  robot_position.orientation_y = msg->pose.pose.orientation.y;
  robot_position.orientation_z = msg->pose.pose.orientation.z;
  robot_position.orientation_w = msg->pose.pose.orientation.w;
}

void RobotControl::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map = msg;   // save map data
  map_received = true; // set flag indicating map is ready to be processed
}

std::vector<Frontier> RobotControl::get_frontiers(const nav_msgs::msg::OccupancyGrid::SharedPtr map,
                                                  double cluster_distance)
{
  std::vector<Frontier> frontiers;

    if (!map) return frontiers;

    int width = static_cast<int>(map->info.width);
    int height = static_cast<int>(map->info.height);
    double res = map->info.resolution;
    double origin_x = map->info.origin.position.x;
    double origin_y = map->info.origin.position.y;

    auto idx_of = [&](int mx, int my) { return my * width + mx; };
    auto inBounds = [&](int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height;
    };

    struct Cell {
        int x, y;
        double wx, wy;
    };
    std::vector<Cell> frontier_cells;

    // Step 1: identify frontier cells (free cell adjacent to unknown)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = idx_of(x, y);
            if (map->data[idx] != 0) continue; // free only

            bool adjacent_unknown = false;
            for (int dy = -1; dy <= 1 && !adjacent_unknown; ++dy) {
                for (int dx = -1; dx <= 1 && !adjacent_unknown; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx;
                    int ny = y + dy;
                    if (!inBounds(nx, ny)) continue;
                    int nidx = idx_of(nx, ny);
                    if (map->data[nidx] == -1) {
                        adjacent_unknown = true;
                    }
                }
            }
            if (adjacent_unknown) {
                double wx = origin_x + (x + 0.5) * res;
                double wy = origin_y + (y + 0.5) * res;
                frontier_cells.push_back({x, y, wx, wy});
            }
        }
    }

    if (frontier_cells.empty()) return frontiers;

    // Safety check helper: require a margin around a point to be free
    auto isSafe = [&](int mx, int my, double margin_m = 0.4) {
        int radius_cells = static_cast<int>(std::ceil(margin_m / res));
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                int nx = mx + dx;
                int ny = my + dy;
                if (!inBounds(nx, ny)) continue;
                int nidx = idx_of(nx, ny);
                if (map->data[nidx] > 50) { // likely occupied
                    return false;
                }
            }
        }
        return true;
    };

    // Step 2: cluster frontier cells with BFS
    std::vector<bool> visited(frontier_cells.size(), false);

    for (size_t i = 0; i < frontier_cells.size(); ++i) {
        if (visited[i]) continue;
        Frontier cluster;
        std::queue<size_t> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            size_t cur = q.front();
            q.pop();
            auto &c = frontier_cells[cur];

            if (isSafe(c.x, c.y)) {
                cluster.push_back({c.wx, c.wy});
            }

            for (size_t j = 0; j < frontier_cells.size(); ++j) {
                if (visited[j]) continue;
                double dx = frontier_cells[cur].wx - frontier_cells[j].wx;
                double dy = frontier_cells[cur].wy - frontier_cells[j].wy;
                if ((dx * dx + dy * dy) <= cluster_distance * cluster_distance) {
                    visited[j] = true;
                    q.push(j);
                }
            }
        }

        if (!cluster.empty()) {
            frontiers.push_back(cluster);
        }
    }

    return frontiers;
}