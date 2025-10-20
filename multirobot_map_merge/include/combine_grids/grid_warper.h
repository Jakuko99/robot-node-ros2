#ifndef GRID_WARPER_H_
#define GRID_WARPER_H_

#include <opencv2/core/utility.hpp>

namespace combine_grids
{
  namespace internal
  {
    class GridWarper
    {
    public:
      cv::Rect warp(const cv::Mat &grid, const cv::Mat &transform,
                    cv::Mat &warped_grid);

    private:
      cv::Rect warpRoi(const cv::Mat &grid, const cv::Mat &transform);
    };

  } // namespace internal

} // namespace combine_grids

#endif // GRID_WARPER_H_
