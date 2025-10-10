#include <opencv2/opencv.hpp>

int main() {
    // Load images
    cv::Mat img1 = cv::imread("/home/ubuntu/ros_ws/src/robot_network/resource/dataset/kris_robot1/plot_map_data_1759758138.png");
    cv::Mat img2 = cv::imread("/home/ubuntu/ros_ws/src/robot_network/resource/dataset/kris_robot2/plot_map_data_1759758139.png");

    // Check if images were loaded successfully
    if (img1.empty() || img2.empty()) {
        std::cerr << "Error: Unable to load images." << std::endl;
        return 1;
    }

    // Detect features
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    sift->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
    sift->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);

    // Match features
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_L2, true);
    matcher.match(descriptors1, descriptors2, matches);

    // Estimate homography
    cv::Mat homography;
    std::vector<cv::Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }
    homography = cv::findHomography(points1, points2, cv::RANSAC);

    // Stitch images
    cv::Mat stitched_img;
    cv::warpPerspective(img1, stitched_img, homography, cv::Size(img1.cols + img2.cols, img1.rows));

    // Display stitched image
    cv::imshow("Stitched Image", stitched_img);

    // Wait for user input
    cv::waitKey(0);

    return 0;
}
