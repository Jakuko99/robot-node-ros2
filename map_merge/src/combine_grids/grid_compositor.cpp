#include "combine_grids/grid_compositor.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <opencv2/stitching/detail/util.hpp>

#include <cassert>
#include <vector>

namespace combine_grids
{
  namespace internal
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr GridCompositor::compose(
        const std::vector<cv::Mat> &grids, const std::vector<cv::Rect> &rois)
    {
      assert(grids.size() == rois.size());

      nav_msgs::msg::OccupancyGrid::SharedPtr result_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();

      std::vector<cv::Point> corners;
      corners.reserve(grids.size());
      std::vector<cv::Size> sizes;
      sizes.reserve(grids.size());
      for (auto &roi : rois)
      {
        corners.push_back(roi.tl());
        sizes.push_back(roi.size());
      }
      cv::Rect dst_roi = cv::detail::resultRoi(corners, sizes);

      result_grid->info.width = static_cast<uint>(dst_roi.width);
      result_grid->info.height = static_cast<uint>(dst_roi.height);
      result_grid->data.resize(static_cast<size_t>(dst_roi.area()), -1);
      // create view for opencv pointing to newly allocated grid
      cv::Mat result(dst_roi.size(), CV_8S, result_grid->data.data());

      for (size_t i = 0; i < grids.size(); ++i)
      {
        // we need to compensate global offset
        cv::Rect roi = cv::Rect(corners[i] - dst_roi.tl(), sizes[i]);
        cv::Mat result_roi(result, roi);
        // reinterpret warped matrix as signed
        // we will not change this matrix, but opencv does not support const matrices
        cv::Mat warped_signed(grids[i].size(), CV_8S, const_cast<uchar *>(grids[i].ptr()));
        // compose img into result matrix
        cv::max(result_roi, warped_signed, result_roi);
      }

      return result_grid;
    }

  } // namespace internal

} // namespace combine_grids
