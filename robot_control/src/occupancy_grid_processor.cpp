#include "occupancy_grid_processor.hpp"

OccupancyGridProcessor::OccupancyGridProcessor(std::string node_name, std::string plan_topic, std::string odom_topic, std::string cmd_topic, std::string scan_topic)
    : Node(node_name)
{
    path_index_ = 0;
    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/merge_map", 10,
        std::bind(&OccupancyGridProcessor::occupancyGridCallback, this, std::placeholders::_1));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(plan_topic, 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&OccupancyGridProcessor::odomCallback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10,
        std::bind(&OccupancyGridProcessor::scanCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    plan_topic_ = plan_topic;
    odom_topic_ = odom_topic;
    cmd_topic_ = cmd_topic;
}

void OccupancyGridProcessor::followPath()
{
    if (current_path_.empty())
    {
        // Stop robot if path complete
        geometry_msgs::msg::Twist stop;
        cmd_vel_publisher_->publish(stop);
        // RCLCPP_INFO(this->get_logger(), "Empty path, stopping robot");
        path_index_ = 0;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Following path: index=%zu", path_index_);

    const auto &target = current_path_[path_index_];
    const auto &pose = current_pose_.pose;

    double dx = target.pose.position.x - pose.position.x;
    double dy = target.pose.position.y - pose.position.y;
    double distance = std::hypot(dx, dy);
    
    // --- Track progress ---
    if (distance < last_distance_to_goal_ - 0.02) {
        // Progress made (reduced distance > 2 cm)
        last_progress_time_ = this->now();
        last_distance_to_goal_ = distance;
    } else if (last_progress_time_.nanoseconds() == 0) {
        // First initialization
        last_progress_time_ = this->now();
        last_distance_to_goal_ = distance;
    }

    const double position_tolerance = 0.15; // meters
    const double max_linear_speed = 0.07;
    const double max_angular_speed = 0.5;

    if (distance < position_tolerance)
    {
        path_index_++;
        path_finished = (path_index_ >= current_path_.size());
        RCLCPP_INFO(this->get_logger(), "Reached target: index=%zu/%zu, path finished: %s",
                    path_index_, current_path_.size(), path_finished ? "true" : "false");
        auto path = this->aStarPath(current_pose_, last_clicked_); // recalculate path
        this->publishPath(path);
        return;
    }

    if (path_index_ >= this->current_path_.size())
    {
        this->current_path_.clear(); // Clear path after reaching target
        path_index_ = 0;             // Reset index
        RCLCPP_INFO(this->get_logger(), "Path completed, stopping robot");
        return;
    }

    if (isWallAhead(current_pose_))
    {
        RCLCPP_WARN(this->get_logger(), "Wall detected ahead, backing up");
        geometry_msgs::msg::Twist backup_cmd;
        backup_cmd.linear.x = -0.1; // move backward
        backup_cmd.angular.z = -0.5; // turn right while backing up
        cmd_vel_publisher_->publish(backup_cmd);
        auto path = this->aStarPath(current_pose_, last_clicked_); // recalculate path
        this->publishPath(path);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Following path: index: %zu/%zu distance to target=%.2f",
                path_index_, this->current_path_.size(), distance);

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
    const ClickedPoint goal)
{
    std::vector<geometry_msgs::msg::PoseStamped> path;
    if (!latest_grid_)
        return path;

    auto [start_x, start_y] = worldToGrid(start.pose.position.x, start.pose.position.y);
    auto [goal_x, goal_y] = worldToGrid(goal.x, goal.y);
    last_clicked_ = goal; // Store clicked point

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

bool OccupancyGridProcessor::isWallAhead(const geometry_msgs::msg::PoseStamped &pose, double distance)
{
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (latest_scan_.empty()) return false;

    const double narrow_fov = M_PI / 12.0;   // ±15°
    const double wide_fov   = M_PI / 4.0;    // ±45°
    const double stop_distance  = 0.35;
    const double clear_distance = 0.45;

    size_t narrow_hits = 0, narrow_total = 0;
    size_t wide_hits   = 0, wide_total   = 0;

    for (size_t i = 0; i < latest_scan_.size(); ++i) {
        double angle = angle_min_ + i * angle_increment_;
        float r = latest_scan_[i];
        if (!std::isfinite(r)) continue;

        if (std::fabs(angle) < narrow_fov) {
            narrow_total++;
            if (r < stop_distance) narrow_hits++;
        }
        if (std::fabs(angle) < wide_fov) {
            wide_total++;
            if (r < stop_distance) wide_hits++;
        }
    }

    bool narrow_blocked = (narrow_total > 0 && (double)narrow_hits / narrow_total > 0.5);
    bool wide_blocked   = (wide_total > 0 && (double)wide_hits / wide_total > 0.7);

    // Progress timeout (e.g. 2 seconds without progress)
    const double timeout_sec = 2.0;
    bool no_progress = (this->now() - last_progress_time_).seconds() > timeout_sec;

    // Combine conditions: wall + no progress
    if ((narrow_blocked || wide_blocked) && no_progress) {
        wall_ahead_ = true;
        return true;
    }

    // Hysteresis reset if area is clear
    if (wide_total > 0) {
        size_t safe_count = 0;
        for (size_t i = 0; i < latest_scan_.size(); ++i) {
            double angle = angle_min_ + i * angle_increment_;
            if (std::fabs(angle) < wide_fov &&
                (latest_scan_[i] > clear_distance || !std::isfinite(latest_scan_[i]))) {
                safe_count++;
            }
        }
        if ((double)safe_count / wide_total > 0.9) {
            wall_ahead_ = false;
        }
    }

    return wall_ahead_;
}

void OccupancyGridProcessor::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg->ranges;  // copy ranges into vector<float>
    angle_min_ = msg->angle_min;
    angle_increment_ = msg->angle_increment;
}

void OccupancyGridProcessor::publishPath(const std::vector<geometry_msgs::msg::PoseStamped> &path)
{
    if (!latest_grid_)
        return;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = latest_grid_->header.frame_id;
    path_msg.poses = path;

    this->current_path_ = path; // Store for following
    path_publisher_->publish(path_msg);
}
