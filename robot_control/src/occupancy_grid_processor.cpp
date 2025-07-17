#include "occupancy_grid_processor.hpp"

OccupancyGridProcessor::OccupancyGridProcessor()
    : Node("occupancy_grid_processor")
{
    path_index_ = 0;
    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/merge_map", 10,
        std::bind(&OccupancyGridProcessor::occupancyGridCallback, this, std::placeholders::_1));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&OccupancyGridProcessor::odomCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void OccupancyGridProcessor::followPath()
{
    if (current_path_.empty())
    {
        // Stop robot if path complete
        /*geometry_msgs::msg::Twist stop;
        cmd_vel_publisher_->publish(stop);*/
        path_index_ = 0;
        return;
    }

    const auto &target = current_path_[path_index_];
    const auto &pose = current_pose_.pose;

    double dx = target.pose.position.x - pose.position.x;
    double dy = target.pose.position.y - pose.position.y;
    double distance = std::hypot(dx, dy);

    const double position_tolerance = 0.1; // meters
    const double max_linear_speed = 0.3;
    const double max_angular_speed = 1.0;

    if (distance < position_tolerance)
    {
        path_index_++;
        path_finished = (path_index_ >= current_path_.size());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Following path: index=%zu",
                path_index_);

    double target_yaw = std::atan2(dy, dx);

    // Get current yaw
    double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
    double current_yaw = std::atan2(siny_cosp, cosy_cosp);

    double yaw_error = target_yaw - current_yaw;
    while (yaw_error > M_PI)
        yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI)
        yaw_error += 2 * M_PI;

    geometry_msgs::msg::Twist cmd;

    cmd.linear.x = std::min(max_linear_speed, distance);
    cmd.angular.z = std::clamp(yaw_error, -max_angular_speed, max_angular_speed);
    cmd_vel_publisher_->publish(cmd);
}

void OccupancyGridProcessor::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_grid_ = msg;
    inflated_grid_ = msg->data; // Start with original data

    int width = msg->info.width;
    int height = msg->info.height;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int index = y * width + x;
            if (msg->data[index] > 50) // occupied
            {
                for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy)
                {
                    for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx)
                    {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                        {
                            int nindex = ny * width + nx;
                            double dist = std::hypot(dx, dy);
                            if (dist <= inflation_radius_)
                            {
                                // Assign inflated cost (artificially high)
                                int cost = std::max(100 - static_cast<int>(dist * 20), 50);
                                inflated_grid_[nindex] = std::max(inflated_grid_[nindex], static_cast<int8_t>(cost));
                            }
                        }
                    }
                }
            }
        }
    }
}

bool OccupancyGridProcessor::isValidCell(int x, int y) const
{
    if (!latest_grid_)
        return false;
    int width = latest_grid_->info.width;
    int height = latest_grid_->info.height;
    return x >= 0 && x < width && y >= 0 && y < height;
}

bool OccupancyGridProcessor::isCellOccupied(int x, int y) const
{
    if (!isValidCell(x, y))
        return true;
    int index = y * latest_grid_->info.width + x;
    return inflated_grid_[index] > 50;
}

double OccupancyGridProcessor::heuristic(int x1, int y1, int x2, int y2) const
{
    return std::hypot(x2 - x1, y2 - y1);
}

std::pair<int, int> OccupancyGridProcessor::worldToGrid(double x, double y) const
{
    double origin_x = latest_grid_->info.origin.position.x;
    double origin_y = latest_grid_->info.origin.position.y;
    double resolution = latest_grid_->info.resolution;

    int grid_x = static_cast<int>((x - origin_x) / resolution);
    int grid_y = static_cast<int>((y - origin_y) / resolution);
    return {grid_x, grid_y};
}

std::pair<double, double> OccupancyGridProcessor::gridToWorld(int x, int y) const
{
    double origin_x = latest_grid_->info.origin.position.x;
    double origin_y = latest_grid_->info.origin.position.y;
    double resolution = latest_grid_->info.resolution;

    double world_x = origin_x + (x + 0.5) * resolution;
    double world_y = origin_y + (y + 0.5) * resolution;
    return {world_x, world_y};
}

std::vector<geometry_msgs::msg::PoseStamped> OccupancyGridProcessor::aStarPath(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal)
{
    std::vector<geometry_msgs::msg::PoseStamped> path;
    if (!latest_grid_)
        return path;

    auto [start_x, start_y] = worldToGrid(start.pose.position.x, start.pose.position.y);
    auto [goal_x, goal_y] = worldToGrid(goal.pose.position.x, goal.pose.position.y);

    struct Node
    {
        int x, y;
        double cost;
        double priority;
        bool operator>(const Node &other) const { return priority > other.priority; }
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<int, std::pair<int, int>> came_from;
    std::unordered_map<int, double> cost_so_far;

    auto hash = [this](int x, int y)
    {
        return y * latest_grid_->info.width + x;
    };

    open_set.push({start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y)});
    cost_so_far[hash(start_x, start_y)] = 0.0;

    std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!open_set.empty())
    {
        Node current = open_set.top();
        open_set.pop();

        if (current.x == goal_x && current.y == goal_y)
        {
            // Reconstruct path
            int cx = goal_x, cy = goal_y;
            while (!(cx == start_x && cy == start_y))
            {
                auto [wx, wy] = gridToWorld(cx, cy);
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = latest_grid_->header.frame_id;
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.orientation.w = 1.0;
                path.push_back(pose);

                auto prev = came_from[hash(cx, cy)];
                cx = prev.first;
                cy = prev.second;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (auto [dx, dy] : directions)
        {
            int nx = current.x + dx;
            int ny = current.y + dy;
            if (!isValidCell(nx, ny) || isCellOccupied(nx, ny))
                continue;

            double obstacle_penalty = inflated_grid_[ny * latest_grid_->info.width + nx] / 100.0;
            double new_cost = cost_so_far[hash(current.x, current.y)] + std::hypot(dx, dy) + obstacle_penalty * 5.0;

            int neighbor_hash = hash(nx, ny);

            if (cost_so_far.find(neighbor_hash) == cost_so_far.end() ||
                new_cost < cost_so_far[neighbor_hash])
            {
                cost_so_far[neighbor_hash] = new_cost;
                double priority = new_cost + heuristic(nx, ny, goal_x, goal_y);
                open_set.push({nx, ny, new_cost, priority});
                came_from[neighbor_hash] = {current.x, current.y};
            }
        }
    }

    path_finished = false;
    return path; // Empty if no path found
}

void OccupancyGridProcessor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
}

void OccupancyGridProcessor::publishPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
{
    if (!latest_grid_)
        return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = latest_grid_->header.frame_id;
    path_msg.poses = path;

    path_publisher_->publish(path_msg);
}
