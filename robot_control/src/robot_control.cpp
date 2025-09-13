#include "robot_control.hpp"

using std::placeholders::_1;
#define MIN_CLUSTER_SIZE 4
#define MIN_SAFETY_MARGIN 0.7

RobotControl::RobotControl(std::string node_name, std::string goal_topic, std::string odom_topic, std::string map_topic, std::string map_frame)
    : rclcpp::Node(node_name),
      goal_set(false),
      robot_moving(false),
      map_received(false),
      robot_stopped_count(0)
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
        if (current_map == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Current map is not initialized!");
            return;
        }

        std::vector<Frontier> frontiers = get_frontiers(current_map);

        if (!frontiers.empty() && !robot_moving && !goal_set)
        {
            if (!frontiers.empty())
            {
                auto target = frontiers[0];
                RCLCPP_INFO(this->get_logger(), "Found %zu frontiers", frontiers.size());
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
                goal_set = true; // Set the goal flag
                RCLCPP_INFO(this->get_logger(), "Published new goal: (%.2f, %.2f)", pose_msg.pose.position.x, pose_msg.pose.position.y);
                last_pose_msg = pose_msg; // Save the last published pose
                robot_moving = true;      // Set the moving flag
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No frontiers found!");
                return;
            }
        }
    }
}

void RobotControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Received null Odometry message");
        return;
    }

    if (robot_position.x == msg->pose.pose.position.x &&
        robot_position.y == msg->pose.pose.position.y)
    {
        robot_moving = false; // Robot has stopped
        robot_stopped_count++;

        if (robot_stopped_count > 5) // If stopped for several updates, clear
        {
            goal_set = false;        // Clear the goal flag to allow new goals
            robot_stopped_count = 0; // Reset counter
            RCLCPP_INFO(this->get_logger(), "Robot has stopped, ready for new goal.");
        }
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

    if (!map)
        return frontiers;

    int width = static_cast<int>(map->info.width);
    int height = static_cast<int>(map->info.height);
    double res = map->info.resolution;
    double origin_x = map->info.origin.position.x;
    double origin_y = map->info.origin.position.y;

    auto idx_of = [&](int mx, int my)
    { return my * width + mx; };
    auto inBounds = [&](int x, int y)
    {
        return x >= 0 && x < width && y >= 0 && y < height;
    };

    struct Cell
    {
        int x, y;
        double wx, wy;
    };
    std::vector<Cell> frontier_cells;

    // Step 1: identify frontier cells (free cell adjacent to unknown)
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int idx = idx_of(x, y);
            if (idx < 0 || idx >= map->data.size())
            {
                RCLCPP_WARN(this->get_logger(), "Index out of bounds: (%d, %d)", x, y);
                continue;
            }
            if (map->data[idx] != 0)
                continue;
            bool adjacent_unknown = false;
            bool adjacent_occupied = false;
            for (int dy = -1; dy <= 1 && !adjacent_unknown && !adjacent_occupied; ++dy)
            {
                for (int dx = -1; dx <= 1 && !adjacent_unknown && !adjacent_occupied; ++dx)
                {
                    if (dx == 0 && dy == 0)
                        continue;
                    int nx = x + dx;
                    int ny = y + dy;
                    if (!inBounds(nx, ny))
                        continue;
                    int nidx = idx_of(nx, ny);
                    if (map->data[nidx] == -1) // adjacent to unknown
                    {
                        adjacent_unknown = true;
                    }
                    else if (map->data[nidx] > 50) // adjacent to occupied (wall)
                    {
                        adjacent_occupied = true;
                    }
                }
            }
            if (adjacent_unknown && !adjacent_occupied)
            {
                double wx = origin_x + (x + 0.5) * res;
                double wy = origin_y + (y + 0.5) * res;
                frontier_cells.push_back({x, y, wx, wy});
            }
        }
    }

    if (frontier_cells.empty())
        return frontiers;

    // Safety check helper: require a margin around a point to be free
    auto isSafe = [&](int mx, int my, double margin_m = MIN_SAFETY_MARGIN) // Increased margin to 0.6m
    {
        int radius_cells = static_cast<int>(std::ceil(margin_m / res));
        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
        {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx)
            {
                int nx = mx + dx;
                int ny = my + dy;
                if (!inBounds(nx, ny))
                    return false; // out-of-bounds is treated as unsafe
                int nidx = idx_of(nx, ny);
                if (map->data[nidx] > 50) // likely occupied
                {
                    return false;
                }
            }
        }
        return true;
    };

    // Step 2: cluster frontier cells with BFS
    std::vector<bool> visited(frontier_cells.size(), false);

    for (size_t i = 0; i < frontier_cells.size(); ++i)
    {
        if (visited[i])
            continue;
        Frontier cluster;
        std::queue<size_t> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty())
        {
            size_t cur = q.front();
            q.pop();
            if (cur >= frontier_cells.size())
            {
                RCLCPP_WARN(this->get_logger(), "Invalid index in frontier_cells: %zu", cur);
                continue;
            }
            auto &c = frontier_cells[cur];

            if (isSafe(c.x, c.y)) // Only add to cluster if safe
            {
                cluster.push_back({c.wx, c.wy});
            }

            for (size_t j = 0; j < frontier_cells.size(); ++j)
            {
                if (visited[j])
                    continue;
                double dx = frontier_cells[cur].wx - frontier_cells[j].wx;
                double dy = frontier_cells[cur].wy - frontier_cells[j].wy;
                if ((dx * dx + dy * dy) <= cluster_distance * cluster_distance)
                {
                    visited[j] = true;
                    q.push(j);
                }
            }
        }

        if (cluster.size() >= MIN_CLUSTER_SIZE) // Only accept clusters with more than 5 points
        {
            // Compute centroid of cluster
            double centroid_x = 0.0, centroid_y = 0.0;
            for (const auto &p : cluster)
            {
                centroid_x += p.first;
                centroid_y += p.second;
            }
            centroid_x /= cluster.size();
            centroid_y /= cluster.size();

            // Add centroid to frontiers
            frontiers.push_back({{centroid_x, centroid_y}});
        }
    }

    return frontiers;
}