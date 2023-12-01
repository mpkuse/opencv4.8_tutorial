#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

// need opencv_
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
int test_viz3d()
{
    viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
    viz::WLine axis(Point3f(-1.0f, -1.0f, -1.0f), Point3f(1.0f, 1.0f, 1.0f));
    axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
    myWindow.showWidget("Line Widget", axis);
    viz::WCube cube_widget(Point3f(0.5, 0.5, 0.0), Point3f(0.0, 0.0, -0.5), true, viz::Color::blue());
    cube_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);
    myWindow.showWidget("Cube Widget", cube_widget);
    Mat rot_vec = Mat::zeros(1, 3, CV_32F);
    float translation_phase = 0.0, translation = 0.0;
    while (!myWindow.wasStopped())
    {
        /* Rotation using rodrigues */
        rot_vec.at<float>(0, 0) += (float)CV_PI * 0.01f;
        rot_vec.at<float>(0, 1) += (float)CV_PI * 0.01f;
        rot_vec.at<float>(0, 2) += (float)CV_PI * 0.01f;
        translation_phase += (float)CV_PI * 0.01f;
        translation = sin(translation_phase);
        Mat rot_mat;
        Rodrigues(rot_vec, rot_mat);
        Affine3f pose(rot_mat, Vec3f(translation, translation, translation));
        myWindow.setWidgetPose("Cube Widget", pose);
        myWindow.spinOnce(1, true);
    }
    return 0;
}


TEST(OpenCVExamples, Viz3d)
{
    // EXPECT_EQ(multiply(2, 3), 6);
    test_viz3d();
}
