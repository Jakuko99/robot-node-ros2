#include <combine_grids/grid_warper.h>
#include <opencv2/stitching/detail/warpers.hpp>
#include <cassert>

namespace combine_grids
{
  namespace internal
  {
    cv::Rect GridWarper::warp(const cv::Mat &grid, const cv::Mat &transform,
                              cv::Mat &warped_grid)
    {
      assert(transform.type() == CV_64F);
      cv::Mat H;
      invertAffineTransform(transform.rowRange(0, 2), H);
      cv::Rect roi = warpRoi(grid, H);
      // shift top left corner for warp affine (otherwise the image is cropped)
      H.at<double>(0, 2) -= roi.tl().x;
      H.at<double>(1, 2) -= roi.tl().y;
      warpAffine(grid, warped_grid, H, roi.size(), cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT,
                 cv::Scalar::all(255) /* this is -1 for signed char */);
      assert(roi.size() == warped_grid.size());

      return roi;
    }

    cv::Rect GridWarper::warpRoi(const cv::Mat &grid, const cv::Mat &transform)
    {
      cv::Ptr<cv::detail::PlaneWarper> warper =
          cv::makePtr<cv::detail::PlaneWarper>();
      cv::Mat H;
      transform.convertTo(H, CV_32F);

      // separate rotation and translation for plane warper
      // 3D translation
      cv::Mat T = cv::Mat::zeros(3, 1, CV_32F);
      H.colRange(2, 3).rowRange(0, 2).copyTo(T.rowRange(0, 2));
      // 3D rotation
      cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
      H.colRange(0, 2).copyTo(R.rowRange(0, 2).colRange(0, 2));

      return warper->warpRoi(grid.size(), cv::Mat::eye(3, 3, CV_32F), R, T);
    }

  } // namespace internal

} // namespace combine_grids
