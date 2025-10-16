#include <thread>

#include "map_merge/map_merge.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cassert>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

namespace map_merge
{
  MapMerge::MapMerge() : rclcpp::Node("map_merge"),
                         subscriptions_size_(0)
  {
    std::string frame_id;
    std::string merged_map_topic;

    this->declare_parameter("merging_rate", 4.0);
    this->declare_parameter("discovery_rate", 0.05);
    this->declare_parameter("estimation_rate", 0.5);
    this->declare_parameter("known_init_poses", true);
    this->declare_parameter("estimation_confidence", 1.0);
    this->declare_parameter("robot_map_topic", "map");
    this->declare_parameter("robot_map_updates_topic", "map_updates");
    this->declare_parameter("robot_namespace", "");
    this->declare_parameter("merged_map_topic", "map");
    this->declare_parameter("world_frame", "world");

    /* publishing */
    merged_map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(this->get_parameter("merged_map_topic").as_string(), rclcpp::QoS(10));
  }

  /*
   * Subcribe to pose and map topics
   */
  void MapMerge::topicSubscribing()
  {
    RCLCPP_DEBUG(this->get_logger(), "Robot discovery started.");

    rclcpp::TopicEndpointInfo topic_infos;
    geometry_msgs::msg::Pose init_pose;
    std::string robot_name;
    std::string map_topic;
    std::string map_updates_topic;

    rclcpp::getTopics(topic_infos);
    // default msg constructor does no properly initialize quaternion
    init_pose.orientation.w = 1; // create identity quaternion

    for (const auto &topic : topic_infos)
    {
      // we check only map topic
      if (!isRobotMapTopic(topic))
      {
        continue;
      }

      robot_name = robotNameFromTopic(topic.name);
      if (robots_.count(robot_name))
      {
        // we already know this robot
        continue;
      }

      if (have_initial_poses_ && !getInitPose(robot_name, init_pose))
      {
        RCLCPP_WARN(this->get_logger(), "Couldn't get initial position for robot [%s]\n"
                                        "did you defined parameters map_merge/init_pose_[xyz]? in robot "
                                        "namespace? If you want to run merging without known initial "
                                        "positions of robots please set `known_init_poses` parameter "
                                        "to false. See relavant documentation for details.",
                    robot_name.c_str());
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "adding robot [%s] to system", robot_name.c_str());
      {
        std::lock_guard<boost::shared_mutex> lock(subscriptions_mutex_);
        subscriptions_.emplace_front();
        ++subscriptions_size_;
      }

      // no locking here. robots_ are used only in this procedure
      MapSubscription &subscription = subscriptions_.front();
      robots_.insert({robot_name, &subscription});
      subscription.initial_pose = init_pose;

      /* subscribe callbacks */
      map_topic = rclcpp::names::append(robot_name, robot_map_topic_);
      map_updates_topic =
          rclcpp::names::append(robot_name, robot_map_updates_topic_);
      RCLCPP_INFO(this->get_logger(), "Subscribing to MAP topic: %s.", map_topic.c_str());

      subscription.map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          map_topic, rclcpp::QoS(50),
          [this, &subscription](const nav_msgs::msg::OccupancyGrid::SharedPtr &msg)
          {
            fullMapUpdate(msg, subscription);
          });

      RCLCPP_INFO(this->get_logger(), "Subscribing to MAP updates topic: %s.",
                  map_updates_topic.c_str());
      subscription.map_updates_sub =
          this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
              map_updates_topic, 50,
              [this, &subscription](
                  const map_msgs::msg::OccupancyGridUpdate::SharedPtr &msg)
              {
                partialMapUpdate(msg, subscription);
              });
    }
  }

  /*
   * mapMerging()
   */
  void MapMerge::mapMerging()
  {
    RCLCPP_DEBUG(this->get_logger(), "Map merging started.");

    if (have_initial_poses_)
    {
      std::vector<nav_msgs::msg::OccupancyGrid::SharedPtr> grids;
      std::vector<geometry_msgs::msg::Pose> poses;
      grids.reserve(subscriptions_size_);
      poses.reserve(subscriptions_size_);
      {
        boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
        for (auto &subscription : subscriptions_)
        {
          std::lock_guard<std::mutex> s_lock(subscription.mutex);
          grids.push_back(subscription.readonly_map);
          poses.push_back(subscription.initial_pose);
        }
      }
      pipeline_.feed(grids.begin(), grids.end());
      pipeline_.setTransforms(poses.begin(), poses.end());
    }

    auto merged_map = pipeline_.composeGrids();
    if (!merged_map) {
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "all maps merged, publishing");
    merged_map->info.map_load_time = this->now();
    merged_map->header.stamp = this->now();
    merged_map->header.frame_id = world_frame_;
    assert(merged_map->info.resolution > 0.f);
    merged_map_publisher_->publish(*merged_map);
  }

  void MapMerge::poseEstimation()
  {
    RCLCPP_DEBUG(this->get_logger(), "Grid pose estimation started.");
    std::vector<geometry_msgs::msg::Pose> poses;
    std::vector<nav_msgs::msg::OccupancyGrid::SharedPtr> grids;
    grids.reserve(subscriptions_size_);
    poses.reserve(subscriptions_size_);
    {
      boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
      for (auto &subscription : subscriptions_)
      {
        std::lock_guard<std::mutex> s_lock(subscription.mutex);
        grids.push_back(subscription.readonly_map);
        poses.push_back(subscription.initial_pose);
      }
    }
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    pipeline_.feed(grids.begin(), grids.end());
    pipeline_.setTransforms(poses.begin(), poses.end());
    pipeline_.estimateTransforms(combine_grids::FeatureType::AKAZE,
                                 confidence_threshold_);
  }

  void MapMerge::fullMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr &msg,
                               MapSubscription &subscription)
  {
    RCLCPP_DEBUG(this->get_logger(), "received full map update");
    std::lock_guard<std::mutex> lock(subscription.mutex);
  if (subscription.readonly_map &&
    rclcpp::Time(subscription.readonly_map->header.stamp) > rclcpp::Time(msg->header.stamp))
    {
      // we have been overrunned by faster update. our work was useless.
      return;
    }

    subscription.readonly_map = msg;
    subscription.writable_map = nullptr;
  }

  void MapMerge::partialMapUpdate(
      const map_msgs::msg::OccupancyGridUpdate::SharedPtr &msg,
      MapSubscription &subscription)
  {
    RCLCPP_DEBUG(this->get_logger(), "received partial map update");

    if (msg->x < 0 || msg->y < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "negative coordinates, invalid update. x: %d, y: %d", msg->x,
                   msg->y);
      return;
    }

    size_t x0 = static_cast<size_t>(msg->x);
    size_t y0 = static_cast<size_t>(msg->y);
    size_t xn = msg->width + x0;
    size_t yn = msg->height + y0;

    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    nav_msgs::msg::OccupancyGrid::SharedPtr readonly_map; // local copy
    {
      // load maps
      std::lock_guard<std::mutex> lock(subscription.mutex);
      map = subscription.writable_map;
      readonly_map = subscription.readonly_map;
    }

    if (!readonly_map)
    {
      RCLCPP_WARN(this->get_logger(), "received partial map update, but don't have any full map to "
                                      "update. skipping.");
      return;
    }

    // we don't have partial map to take update, we must copy readonly map and
    // update new writable map
    if (!map)
    {
      map.reset(new nav_msgs::msg::OccupancyGrid(*readonly_map));
    }

    size_t grid_xn = map->info.width;
    size_t grid_yn = map->info.height;

    if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn)
    {
      RCLCPP_WARN(this->get_logger(), "received update doesn't fully fit into existing map, "
                                      "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
                                      "map is: [0, %lu], [0, %lu]",
                  x0, xn, y0, yn, grid_xn, grid_yn);
    }

    // update map with data
    size_t i = 0;
    for (size_t y = y0; y < yn && y < grid_yn; ++y)
    {
      for (size_t x = x0; x < xn && x < grid_xn; ++x)
      {
        size_t idx = y * grid_xn + x; // index to grid for this specified cell
        map->data[idx] = msg->data[i];
        ++i;
      }
    }
    // update time stamp
    map->header.stamp = msg->header.stamp;

    {
      // store back updated map
      std::lock_guard<std::mutex> lock(subscription.mutex);
    if (subscription.readonly_map &&
      rclcpp::Time(subscription.readonly_map->header.stamp) > rclcpp::Time(map->header.stamp))
      {
        // we have been overrunned by faster update. our work was useless.
        return;
      }
      subscription.writable_map = map;
      subscription.readonly_map = map;
    }
  }

  std::string MapMerge::robotNameFromTopic(const std::string &topic)
  {
  // ROS 2 does not have rclcpp::names::parentNamespace, so use string manipulation
  auto pos = topic.find_last_of('/');
  if (pos == std::string::npos || pos == 0) return "";
  return topic.substr(0, pos);
  }

  /* identifies topic via suffix */
  bool MapMerge::isRobotMapTopic(const rclcpp::TopicEndpointInfo &topic)
  {
    /* test whether topic is robot_map_topic_ */
    // ROS 2: TopicEndpointInfo is not used for topic discovery in the same way. Use topic name and datatype.
  return isRobotMapTopic(topic.get_topic_name(), topic.get_topic_type());

  // New signature for ROS 2
  bool MapMerge::isRobotMapTopic(const std::string &topic_name, const std::string &datatype)
  {
    // Check if topic ends with robot_map_topic_
    bool is_map_topic = topic_name.size() >= robot_map_topic_.size() &&
      topic_name.compare(topic_name.size() - robot_map_topic_.size(), robot_map_topic_.size(), robot_map_topic_) == 0;
    // Check if topic contains robot_namespace_
    bool contains_robot_namespace = topic_name.find(robot_namespace_) != std::string::npos;
    // Only occupancy grid topics
    bool is_occupancy_grid = datatype == "nav_msgs/msg/OccupancyGrid";
    // Don't subscribe to our own published topic
    // (Assume merged_map_publisher_ topic is stored in merged_map_topic_)
    bool is_our_topic = topic_name == merged_map_topic_;
    return is_occupancy_grid && !is_our_topic && contains_robot_namespace && is_map_topic;
  }
  }

  /*
   * Get robot's initial position
   */
  bool MapMerge::getInitPose(const std::string &name,
                             geometry_msgs::msg::Pose &pose)
  {
    std::string merging_namespace = name + "/map_merge";
    double yaw = 0.0;
    bool success =
      this->get_parameter(merging_namespace + "/init_pose_x", pose.position.x) &&
      this->get_parameter(merging_namespace + "/init_pose_y", pose.position.y) &&
      this->get_parameter(merging_namespace + "/init_pose_z", pose.position.z) &&
      this->get_parameter(merging_namespace + "/init_pose_yaw", yaw);
    tf2::Quaternion q;
    q.setEuler(0., 0., yaw);
    pose.orientation = tf2::toMsg(q);
    return success;
  }

  /*
   * execute()
   */
  void MapMerge::executemapMerging()
  {
    rclcpp::Rate r(merging_rate_);
    while (rclcpp::ok())
    {
      mapMerging();
      r.sleep();
    }
  }

  void MapMerge::executetopicSubscribing()
  {
    rclcpp::Rate r(discovery_rate_);
    while (rclcpp::ok())
    {
      topicSubscribing();
      r.sleep();
    }
  }

  void MapMerge::executeposeEstimation()
  {
    if (have_initial_poses_)
      return;

    rclcpp::Rate r(estimation_rate_);
    while (rclcpp::ok())
    {
      poseEstimation();
      r.sleep();
    }
  }

  /*
   * spin()
   */
  void MapMerge::spin()
  {
    std::thread merging_thr([this]()
                            { executemapMerging(); });
    std::thread subscribing_thr([this]()
                                { executetopicSubscribing(); });
    std::thread estimation_thr([this]()
                               { executeposeEstimation(); });
    estimation_thr.join();
    merging_thr.join();
    subscribing_thr.join();
  }

} // namespace map_merge

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto map_merging = std::make_shared<map_merge::MapMerge>();
  while (rclcpp::ok())
  {
    rclcpp::spin_some(map_merging);
    map_merging->spin();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  rclcpp::shutdown();

  return 0;
}
