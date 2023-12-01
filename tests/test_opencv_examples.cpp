#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/features2d.hpp>

using namespace cv;

// A simple function to multiplication two numbers
int test_imushow()
{
    std::string image_path = "../data/lena.png";
    Mat img = imread(image_path, IMREAD_COLOR);

    imshow("Display window", img);
    int k = waitKey(0); // Wait for a keystroke in the window
    return 0;
}

int test_basic_image_op()
{
    std::string image_path = "../data/lena.png";
    Mat img = imread(image_path, IMREAD_COLOR);
    auto img_neg = 255. - img;

    imshow("Display window", img_neg);
    int k = waitKey(0); // Wait for a keystroke in the window
    return 0;
}

void test_kp_detector()
{
    // Read the image
    std::string image_path = "../data/SFMedu/B21.jpg";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    // Create a keypoint detector (e.g., ORB)
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Detect keypoints and compute descriptors
    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

    // Draw keypoints on the image
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(image, keypoints, image_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DEFAULT);

    // Display the image with keypoints
    cv::imshow("Image with Keypoints", image_with_keypoints);
    cv::waitKey(0);
}

TEST(OpenCVExamples, ImShow)
{
    test_imushow();
}

TEST(OpenCVExamples, ImageProcessing)
{
    test_basic_image_op();
}

TEST(OpenCVExamples, KeyPointDetector)
{
    test_kp_detector();
}