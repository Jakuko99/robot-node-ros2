
#include "nav_msgs/msg/occupancy_grid.h"
#include "geometry_msgs/msg/pose.h"
#include "map_msgs/msg/occupancy_grid_update.h"
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <forward_list>
#include <mutex>
#include <unordered_map>
#include "combine_grids/merging_pipeline.h"
#include <boost/thread.hpp>
#ifndef MAP_MERGE_H_
#define MAP_MERGE_H_

namespace map_merge
{
  struct MapSubscription
  {
    // protects consistency of writable_map and readonly_map
    // also protects reads and writes of shared_ptrs
    std::mutex mutex;

  geometry_msgs::msg::Pose initial_pose;
    nav_msgs::msg::OccupancyGrid::SharedPtr writable_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr readonly_map;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_updates_sub;
  };

  class MapMerge : public rclcpp::Node
  {
  private:
    rclcpp::Node::SharedPtr node_;

    /* parameters */
    double merging_rate_;
    double discovery_rate_;
    double estimation_rate_;
    double confidence_threshold_;
    std::string robot_map_topic_;
    std::string robot_map_updates_topic_;
    std::string robot_namespace_;
    std::string world_frame_;
    bool have_initial_poses_;

    // publishing
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_publisher_;
    // maps robots namespaces to maps. does not own
    std::unordered_map<std::string, MapSubscription *> robots_;
    // owns maps -- iterator safe
    std::forward_list<MapSubscription> subscriptions_;
    size_t subscriptions_size_;
    boost::shared_mutex subscriptions_mutex_;
    combine_grids::MergingPipeline pipeline_;
    std::mutex pipeline_mutex_;

    std::string robotNameFromTopic(const std::string &topic);
  bool isRobotMapTopic(const rclcpp::TopicEndpointInfo &topic);
  bool isRobotMapTopic(const std::string &topic_name, const std::string &datatype);
  bool getInitPose(const std::string &name, geometry_msgs::msg::Pose &pose);

    void fullMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr &msg,
                       MapSubscription &map);
    void partialMapUpdate(const map_msgs::msg::OccupancyGridUpdate::SharedPtr &msg,
                          MapSubscription &map);

  public:
    MapMerge();

    void spin();
    void executetopicSubscribing();
    void executemapMerging();
    void executeposeEstimation();

    void topicSubscribing();
    void mapMerging();
    /**
     * @brief Estimates initial positions of grids
     * @details Relevant only if initial poses are not known
     */
    void poseEstimation();
  };

} // namespace map_merge

#endif /* MAP_MERGE_H_ */
