#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <cassert>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

namespace helpers
{
    using namespace Eigen;
    using namespace std;
    class MiscUtils
    {
    public:
        static string type2str(int type);
        static string cvmat_info(const cv::Mat &mat);
        static std::vector<std::string>
        split(std::string const &original, char separator);

        template <typename MatType>
        static std::string eigen_info(const MatType m);

        //---------------------------- Conversions ---------------------------------//
        // convert from opencv format of keypoints to Eigen
        static void keypoint_2_eigen(const std::vector<cv::KeyPoint> &kp, MatrixXd &uv, bool make_homogeneous = true);
        static void point2f_2_eigen(const std::vector<cv::Point2f> &kp, MatrixXd &uv, bool make_homogeneous = true);

        // given opencv keypoints and DMatch will produce M1, and M2 the co-ordinates
        static void dmatch_2_eigen(const std::vector<cv::KeyPoint> &kp1, const std::vector<cv::KeyPoint> &kp2,
                                   const std::vector<cv::DMatch> matches,
                                   MatrixXd &M1, MatrixXd &M2,
                                   bool make_homogeneous = true);
        //---------------------------- Conversions ---------------------------------//

        //------------------------- Conversions to OpenCV ------------------------------//

        // extracts the keypoints (sans other info)
        static void keypoint_2_point2f(const std::vector<cv::KeyPoint> &kp, std::vector<cv::Point2f> &kp_2f);

        // eigen uv 2xN to vector of cv::Point2f
        static void eigen_2_point2f(const MatrixXd &uv, std::vector<cv::Point2f> &kp);

        //------------------------- Conversions to OpenCV ------------------------------//

        //---------------------- Inlier Masking --------------------------//
        static uint32_t count_inliers(const std::vector<uchar> &inliers);
        static void retain_inliers(const std::vector<cv::Point2f> &kp, const std::vector<uchar> &inliers, std::vector<cv::Point2f> &kp_retained);
        static void retain_non_inliers(const std::vector<cv::Point2f> &kp, const std::vector<uchar> &inliers, std::vector<cv::Point2f> &kp_retained);

        template <typename T>
        static void retain_inliers_g(const std::vector<T> &kp, const std::vector<uchar> &inliers, std::vector<T> &kp_retained);

        template <typename T>
        static void retain_when_true(const std::vector<T> &kp, const std::vector<bool> &mask, std::vector<T> &kp_retained);

        template <typename T>
        static void retain_when_false(const std::vector<T> &kp, const std::vector<bool> &mask, std::vector<T> &kp_retained);

        //--------------------- Plot Keypoints on Image ----------------------------//
        // Eigen Interace:  PLotting functions with Eigen Interfaces
        static void plot_point_sets(const cv::Mat &im, const MatrixXd &pts_set, cv::Mat &dst,
                                    const cv::Scalar &color, bool enable_keypoint_annotation = true, const string &msg = string(""));

        // cv::Mat Interfaces: Plotting Functions with cv::Mat Interfaces.
        static void plot_point_sets(const cv::Mat &im, const cv::Mat &pts_set, cv::Mat &dst,
                                    const cv::Scalar &color, bool enable_keypoint_annotation = true, const string &msg = string(""));

        // Inplace plotting. Here dont need to specify a separate destination. src is modified.
        static void plot_point_sets(cv::Mat &im, const MatrixXd &pts_set,
                                    const cv::Scalar &color, bool enable_keypoint_annotation, const string &msg);

        // Plotting with annotations specified by VectorXi
        static void plot_point_sets(cv::Mat &im, const MatrixXd &pts_set, cv::Mat &dst,
                                    const cv::Scalar &color, const VectorXi &annotations, const string &msg);

        // Plotting with annotations specified by VectorXi inplace
        static void plot_point_sets(cv::Mat &im, const MatrixXd &pts_set,
                                    const cv::Scalar &color, const VectorXi &annotations, const string &msg);

        // plot point with colors specified at every point. pts_set : 3xN or 2xN, len(color_annotations) == pts_set.cols()
        static void plot_point_sets(const cv::Mat &im, const MatrixXd &pts_set, cv::Mat &dst,
                                    vector<cv::Scalar> &color_annotations, float alpha = 0.8, const string &msg = string("N/A"));

        // END--------------------- Plot Keypoints on Image ----------------------------//

        //------------------------------- Plot Matchings on image pair -------------------------//

        // Plots [ imA | imaB ] with points correspondences
        // [Input]
        //    imA, imB : Images
        //    ptsA, ptsB : 2xN or 3xN
        //    idxA, idxB : Index of each of the image. This will appear in status part. No other imppact of these.
        //    color_marker : color of the point marker
        //    color_line   : color of the line
        //    annotate_pts : true with putText for each point. False will not putText.
        // [Output]
        //    outImg : Output image
        static void plot_point_pair(const cv::Mat &imA, const MatrixXd &ptsA, int idxA,
                                    const cv::Mat &imB, const MatrixXd &ptsB, int idxB,
                                    cv::Mat &dst,
                                    const cv::Scalar &color_marker,
                                    const cv::Scalar &color_line = cv::Scalar(0, 255, 0),
                                    bool annotate_pts = false,
                                    const string &msg = string("N.A"));

        // nearly same as the above, but will color every co-ordinate with different color
        // color_map_direction : 0 ==> // horizontal-gradiant
        //                       1 ==>  // vertical-gradiant
        //                       2 ==> // manhattan-gradiant
        //                       3 ==> // image centered manhattan-gradiant
        static void plot_point_pair(const cv::Mat &imA, const MatrixXd &ptsA, int idxA,
                                    const cv::Mat &imB, const MatrixXd &ptsB, int idxB,
                                    cv::Mat &dst,
                                    short color_map_direction,
                                    const string &msg = string("N.A"));

        //------------------------------- Plot Matchings on image pair -------------------------//

        //------------------------- Points and Lines on Images --------------------------------//

        // Given two image-points draw line between them, extend both ways. Infinite line-segments
        static void draw_fullLine(cv::Mat &img, cv::Point2f a, cv::Point2f b, cv::Scalar color);

        // draw line on the image, given a line equation in homogeneous co-ordinates. l = (a,b,c) for ax+by+c = 0
        static void draw_line(const Vector3d l, cv::Mat &im, cv::Scalar color);

        // mark point on the image, pt is in homogeneous co-ordinate.
        static void draw_point(const Vector3d pt, cv::Mat &im, cv::Scalar color);

        // mark point on image
        static void draw_point(const Vector2d pt, cv::Mat &im, cv::Scalar color);

        // END ------------------------- Points and Lines on Images --------------------------------//

    private:
        static double Slope(int x0, int y0, int x1, int y1);
    };

    template <typename MatType>
    std::string MiscUtils::eigen_info(const MatType m)
    {
        std::stringstream buffer;
        buffer << "shape=" << m.rows() << "x" << m.cols();
        return buffer.str();
    }

    template <typename T>
    void MiscUtils::retain_inliers_g(const std::vector<T> &kp, const std::vector<uchar> &inliers, std::vector<T> &kp_retained)
    {
        kp_retained.clear();
        assert(kp.size() == inliers.size() && "Expecting number of items to be equal to inliers mask");
        for (auto i = 0u; i < kp.size(); i++)
        {
            if (inliers[i] > 0u)
            {
                kp_retained.push_back(kp[i]);
            }
        }
    }

    template <typename T>
    static void retain_when_true(const std::vector<T> &kp, const std::vector<bool> &mask, std::vector<T> &kp_retained)
    {
        kp_retained.clear();
        assert(kp.size() == mask.size() && "Expecting number of items to be equal to mask");
        for (auto i = 0u; i < kp.size(); i++)
        {
            if (mask[i])
            {
                kp_retained.push_back(kp[i]);
            }
        }
    }

    template <typename T>
    static void retain_when_false(const std::vector<T> &kp, const std::vector<bool> &mask, std::vector<T> &kp_retained)
    {
        kp_retained.clear();
        assert(kp.size() == mask.size() && "Expecting number of items to be equal to mask");
        for (auto i = 0u; i < kp.size(); i++)
        {
            if (!mask[i])
            {
                kp_retained.push_back(kp[i]);
            }
        }
    }

}
