#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <queue>
#include <unordered_map>
#include <utility>

class OccupancyGridProcessor : public rclcpp::Node
{
public:
    OccupancyGridProcessor();

    // A* pathfinding from start to goal in world coordinates
    std::vector<geometry_msgs::msg::PoseStamped> aStarPath(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal);
    void publishPath(const std::vector<geometry_msgs::msg::PoseStamped> &path);
    void followPath();

private:
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    bool isCellOccupied(int x, int y) const;
    bool isValidCell(int x, int y) const;
    double heuristic(int x1, int y1, int x2, int y2) const;

    std::pair<int, int> worldToGrid(double x, double y) const;
    std::pair<double, double> gridToWorld(int x, int y) const;
    std::vector<int8_t> inflated_grid_;
    int inflation_radius_ = 5; // in cells

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    geometry_msgs::msg::PoseStamped current_pose_;
    std::vector<geometry_msgs::msg::PoseStamped> current_path_;
    size_t path_index_ = 0;
    bool path_finished = false;    
};
