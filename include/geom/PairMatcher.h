#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/features2d.hpp>

#include <iostream>
#include <vector>

#include <helpers/MiscUtils.h>
#include <helpers/CustomAssert.h>
#include <geom/ImagedFeatures.h>

namespace geom
{
    class PairMatcher2
    {
    public:
        PairMatcher2(const ImagedFeatures &frame1_, const ImagedFeatures &frame2_);
        void PrintInfo(const uint8_t verbosity = 0u) const;

        const Eigen::MatrixXd M1() const;
        const std::vector<geom::FeatIdx> M1Idx() const;

        const Eigen::MatrixXd M2() const;
        const std::vector<geom::FeatIdx> M2Idx() const;

        void GetFundamentalMatrix(Eigen::Matrix<double, 3, 3> &F) const;
        const cv::Mat VizImage() const;

    private:
        const ImagedFeatures frame1,
            frame2;
        void basic_nn_matching();
        void ratio_test(const float nn_match_ratio);

        void compute_fundamental_matrix();
        void compute_fundamental_matrix_with_normalization();
        void fundamental_matrix_test();

        std::vector<std::vector<cv::DMatch>> nn_matches;       //< 2nn matches for every kpts of frame1
        std::vector<cv::KeyPoint> matched1, matched2;          //< results after ratiotest
        std::vector<geom::FeatIdx> matched1_idx, matched2_idx; //< idx of matched1 and matched2
        std::vector<cv::Point2f> m1, m2;

        std::vector<uchar> matched_inliers;
        std::vector<cv::Point2f> m1_retained, m2_retained;           //< result after f-test
        Eigen::MatrixXd m1_retained_eigen, m2_retained_eigen;        //< same as `m?_retained`
        std::vector<geom::FeatIdx> m1_retained_idx, m2_retained_idx; //< idx of matched1 and matched2

        cv::Mat fundamental_matrix;
        static bool constexpr normalize_for_f_matrix_computation = true;
    };

    // A basic, brute force matcher given two input images
    class PairMatcher
    {
    public:
        PairMatcher(const cv::Mat &im1_, const cv::Mat &im2_);

        void PrintInfo(const uint8_t verbosity = 0u) const;

        Eigen::MatrixXd M1()
        {
            return m1_retained_eigen;
        }

        Eigen::MatrixXd M2()
        {
            return m2_retained_eigen;
        }

        void get_fundamental_matrix(Eigen::Matrix<double, 3, 3> &F);

    private:
        const cv::Mat im1, im2;                          //< input images
        std::vector<cv::KeyPoint> kpts1, kpts2;          //< resulting keypoints
        cv::Mat desc1, desc2;                            //< resulting descriptors
        std::vector<std::vector<cv::DMatch>> nn_matches; //< 2nn matches for every kpts1

        std::vector<cv::KeyPoint> matched1, matched2; //< results after ratiotest
        std::vector<cv::Point2f> m1, m2;

        std::vector<uchar> matched_inliers;
        std::vector<cv::Point2f> m1_retained, m2_retained;    //< result after f-test
        Eigen::MatrixXd m1_retained_eigen, m2_retained_eigen; //< same as `m?_retained`

        cv::Mat fundamental_matrix;

        cv::Ptr<cv::Feature2D> feat_detector; // = cv::SIFT::create();
        cv::BFMatcher matcher;

        void init_feature_detector();
        void detect_features_and_compute_descriptors();

        void basic_nn_matching();
        void ratio_test(const float nn_match_ratio);
        void compute_fundamental_matrix();
        void compute_fundamental_matrix_with_normalization();
        void fundamental_matrix_test();

        static bool constexpr normalize_for_f_matrix_computation = true;
    };

} // namespace geom