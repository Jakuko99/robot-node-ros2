#ifndef GRID_COMPOSITOR_H_
#define GRID_COMPOSITOR_H_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <opencv2/core/utility.hpp>

namespace combine_grids
{
  namespace internal
  {
    class GridCompositor
    {
    public:
      nav_msgs::msg::OccupancyGrid::SharedPtr compose(const std::vector<cv::Mat> &grids,
                                                      const std::vector<cv::Rect> &rois);
    };

  } // namespace internal

} // namespace combine_grids

#endif // GRID_COMPOSITOR_H_
