#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/features2d.hpp>

#include <iostream>
#include <vector>

#include <helpers/MiscUtils.h>
#include <helpers/TermColor.h>
#include <helpers/ElapsedTime.h>

#include <geom/ImagedFeatures.h>
#include <geom/PairMatcher.h>
#include <geom/TwoViewGeometry.h>

void AKazeDetectionAndMatching()
{

    //-- read 2 images
    std::string im1_fname = "../data/SFMedu/B23.jpg";
    std::string im2_fname = "../data/SFMedu/B22.jpg";
    cv::Mat im1 = cv::imread(im1_fname, cv::IMREAD_GRAYSCALE);
    cv::Mat im2 = cv::imread(im2_fname, cv::IMREAD_GRAYSCALE);

    geom::PairMatcher p(im1, im2);
    if (true)
    {
        printf("%sim1.info: %s\n%s", helpers::TermColor::RED().c_str(), helpers::MiscUtils::cvmat_info(im1).c_str(), helpers::TermColor::RESET().c_str());
    }
    if (false)
    {
        cv::imshow("im1", im1);
        cv::imshow("im2", im2);
    }

    //-- feature detector
    std::vector<cv::KeyPoint> kpts1, kpts2;
    cv::Mat desc1, desc2;
    // cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
    cv::Ptr<cv::SIFT> akaze = cv::SIFT::create();

    //-- detect features between 2 images
    auto _t_dandc = helpers::ElapsedTime("DetectAndCompute");
    akaze->detectAndCompute(im1, cv::noArray(), kpts1, desc1);
    akaze->detectAndCompute(im2, cv::noArray(), kpts2, desc2);
    _t_dandc.toc_print();

    EXPECT_EQ(kpts1.size(), desc1.rows);
    EXPECT_EQ(kpts2.size(), desc2.rows);
    printf("im1:> nfeats: %lu \t", kpts1.size());
    printf("im2:> nfeats: %lu\t", kpts2.size());
    printf("\n");

    //-- Bruteforce Descriptor Matching
    // cv::BFMatcher matcher(cv::NORM_HAMMING);
    cv::BFMatcher matcher(cv::NORM_L1);
    std::vector<std::vector<cv::DMatch>> nn_matches;
    auto _t_knn = helpers::ElapsedTime("KNN");
    matcher.knnMatch(desc1, desc2, nn_matches, 2);
    _t_knn.toc_print();
    // ^^ gets nearest neighbour of every keypt in desc1. Here we requested 2 nearest neighbors.

    //-- Lowe's ratio test
    // loop over all the nearest neighbors, only keep those matches which pass the ratio test
    auto _t_lowe = helpers::ElapsedTime("Lowe Ratio Test");
    std::vector<cv::KeyPoint> matched1, matched2;
    float nn_match_ratio = 0.8;
    for (size_t i = 0; i < nn_matches.size(); i++)
    {
        cv::DMatch first = nn_matches[i][0]; // best match
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;
        if (dist1 < nn_match_ratio * dist2)
        {
            matched1.push_back(kpts1[first.queryIdx]);
            matched2.push_back(kpts2[first.trainIdx]);
        }
    }
    _t_lowe.toc_micro_print();

    if (true)
    {
        EXPECT_EQ(matched1.size(), matched2.size());
        printf("Lowe's ratio test retained %lu matches\n", matched1.size());
    }

    // plot matches after ratio test
    std::vector<cv::Point2f> m1, m2;
    helpers::MiscUtils::keypoint_2_point2f(matched1, m1);
    helpers::MiscUtils::keypoint_2_point2f(matched2, m2);

    Eigen::MatrixXd m1_eigen, m2_eigen;
    helpers::MiscUtils::point2f_2_eigen(m1, m1_eigen);
    helpers::MiscUtils::point2f_2_eigen(m2, m2_eigen);
    if (true)
    {
        cv::Mat dst;
        helpers::MiscUtils::plot_point_pair(im1, m1_eigen, 0, im2, m2_eigen, 1, dst, cv::Scalar(0, 0, 255));
        cv::imshow("dst", dst);
    }

    //-- Fundamental matrix test
    auto _t_fund_mat = helpers::ElapsedTime("Fundamental Matrix Test");

    // Find the fundamental matrix using RANSAC
    std::vector<uchar> inliers(m1.size(), 0);

    cv::Mat fundamentalMatrix = cv::findFundamentalMat(m1, m2, cv::FM_RANSAC, 1.5, 0.9, 4, inliers);
    _t_fund_mat.toc_micro_print();

    uint32_t n_inliers = helpers::MiscUtils::count_inliers(inliers);
    printf("Fundamental matrix test found %u inliers out of total %lu\n", n_inliers, m1.size());

    // only keep inlier matches based on fundamental matrix test
    std::vector<cv::Point2f> m1_retained, m2_retained;
    helpers::MiscUtils::retain_inliers(m1, inliers, m1_retained);
    helpers::MiscUtils::retain_inliers(m2, inliers, m2_retained);
    EXPECT_EQ(m1_retained.size(), n_inliers);
    EXPECT_EQ(m2_retained.size(), n_inliers);
    _t_fund_mat.toc_micro_print();

    // plot inlier matches
    Eigen::MatrixXd m1_retained_eigen, m2_retained_eigen;
    helpers::MiscUtils::point2f_2_eigen(m1_retained, m1_retained_eigen);
    helpers::MiscUtils::point2f_2_eigen(m2_retained, m2_retained_eigen);
    if (true)
    {
        cv::Mat dst;
        helpers::MiscUtils::plot_point_pair(im1, m1_retained_eigen, 0, im2, m2_retained_eigen, 1, dst, cv::Scalar(0, 0, 255));
        cv::imshow("dst retained", dst);
    }

    cv::waitKey(0);
}

void ImagePairMatcher(const std::string &im1_fname, const std::string &im2_fname)
{
    cv::Mat im1 = cv::imread(im1_fname, cv::IMREAD_GRAYSCALE);
    cv::Mat im2 = cv::imread(im2_fname, cv::IMREAD_GRAYSCALE);
    geom::PairMatcher p(im1, im2);
    p.PrintInfo();

    if (true)
    {
        cv::Mat dst;
        std::string info_str = im1_fname + " | " + im2_fname;
        helpers::MiscUtils::plot_point_pair(im1, p.M1(), 0, im2, p.M2(), 1, dst, cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), false, info_str);
        cv::imshow("dst retained", dst);
        cv::waitKey(0);
    }

    Eigen::MatrixXd M1 = p.M1();
    Eigen::MatrixXd M2 = p.M2();
    std::cout << "M1: " << helpers::MiscUtils::eigen_info(M1) << std::endl;
    std::cout << "M2: " << helpers::MiscUtils::eigen_info(M2) << std::endl;
    Eigen::Matrix3d F;
    p.get_fundamental_matrix(F);
    std::cout << "p.get_fundamental_matrix: \n"
              << F << std::endl;

    Eigen::Matrix3d K; // camera intrinsic
    double f = 719.5459;
    K << f, 0, 0,
        0, f, 0,
        0, 0, 1;

    geom::TwoViewGeometry g2view(M1, M2, F, K);
    const Matrix4d cam1_T_w = g2view.get_cam1_T_w();
    const Matrix4d cam2_T_w = g2view.get_cam2_T_w();
    const MatrixXd pts_w = g2view.get_pts_w();

    if constexpr (true)
    {
        // write results to file
        std::string outfname = "data/pts3d_w.txt";
        helpers::RawFileIO::write_eigen_matrix(outfname, pts_w.transpose());

        outfname = "data/wTc1.txt";
        helpers::RawFileIO::write_eigen_matrix(outfname, cam1_T_w.inverse());

        outfname = "data/wTc2.txt";
        helpers::RawFileIO::write_eigen_matrix(outfname, cam2_T_w.inverse());
    }
}

void PoseComputationFromEssentialMat(const std::string &im1_fname, const std::string &im2_fname)
{
    cv::Mat im1 = cv::imread(im1_fname, cv::IMREAD_GRAYSCALE);
    cv::Mat im2 = cv::imread(im2_fname, cv::IMREAD_GRAYSCALE);
    geom::PairMatcher p(im1, im2);
    p.PrintInfo();

    if (true)
    {
        cv::Mat dst;
        std::string info_str = im1_fname + " | " + im2_fname;
        helpers::MiscUtils::plot_point_pair(im1, p.M1(), 0, im2, p.M2(), 1, dst, cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), false, info_str);
        cv::imshow("dst retained", dst);
        cv::waitKey(0);
    }

    //-- playing with fundamental matrix, and the matches
    Eigen::Matrix<double, 3, 3> F = Eigen::Matrix<double, 3, 3>::Zero();
    p.get_fundamental_matrix(F);
    Eigen::MatrixXd M1 = p.M1();
    Eigen::MatrixXd M2 = p.M2();

    // std::cout << F << std::endl;;
    std::cout << "M1.col(0): \n"
              << M1.col(0) << std::endl; //( M1.col(0).transpose() * F ) * M2.col(0) << std::endl;
    std::cout << "M2.col(0): \n"
              << M2.col(0) << std::endl;
    std::cout << "F: \n"
              << F << std::endl;

    // for (auto i = 0u; i < M1.cols(); i++)
    //     std::cout << M1.col(i).transpose() * F * M2.col(i) << std::endl;

    //-- Camera Intrinsic
    Eigen::Matrix3d K; // camera intrinsic
    double f = 719.5459;
    K << f, 0, 0,
        0, f, 0,
        0, 0, 1;
    std::cout << "K:\n"
              << K << std::endl;

    //-- Essential Matrix
    Eigen::Matrix3d E;
    E = K.transpose() * F * K;
    std::cout << "E:\n"
              << E << std::endl;

    //-- SVD to decompose E
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Vector3d S = svd.singularValues();
    Eigen::Matrix3d V = svd.matrixV();
    std::cout << "U: " << U << std::endl;
    std::cout << "S: " << S << std::endl;
    std::cout << "V: " << V << std::endl;
    Eigen::Matrix3d W;
    W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    auto R1 = U * W.transpose() * V.transpose();
    auto R2 = U * W * V.transpose();
    auto t = U.col(2);

    //-- compute this with opencv
    cv::Mat E_cvmat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            E_cvmat.at<double>(i, j) = E(i, j);
        }
    }
    std::cout << "E_cvmat:\n"
              << E_cvmat << std::endl;

    cv::Mat R1_cvmat(3, 3, CV_64F), R2_cvmat(3, 3, CV_64F), t_cvmat(3, 1, CV_64F);
    cv::decomposeEssentialMat(E_cvmat, R1_cvmat, R2_cvmat, t_cvmat);
    std::cout << "R1:\n"
              << R1 << std::endl;
    std::cout << "R1_cvmat:\n"
              << R1_cvmat << std::endl;
    std::cout << "R2:\n"
              << R2 << std::endl;
    std::cout << "R2_cvmat:\n"
              << R2_cvmat << std::endl;
    std::cout << "t:\n"
              << t << std::endl;
    std::cout << "t_cvmat:\n"
              << t_cvmat << std::endl;

    // I get exact same result from opencv and my own with eigen

    // is using the F as is a good idea? I think fundamental matrix should be
    // computed with

    // 4 solutions: [R1|t], [R1|-t], [R2|t], [R2|-t]
}

TEST(OpenCVExamples, AKazeDetectionAndMatching)
{
    AKazeDetectionAndMatching();
}

TEST(OpenCVExamples, ImagePairMatcher)
{
    std::string im1_fname = "../data/SFMedu/B23.jpg";
    std::string im2_fname = "../data/SFMedu/B22.jpg";

    ImagePairMatcher(im1_fname, im2_fname);
}

TEST(OpenCVExamples, PoseComputationFromEssentialMat)
{
    std::string im1_fname = "../data/SFMedu/B23.jpg";
    std::string im2_fname = "../data/SFMedu/B22.jpg";

    PoseComputationFromEssentialMat(im1_fname, im2_fname);
}

TEST(OpenCVExamples, ImagedFeatures)
{
    cv::Ptr<cv::Feature2D> feat_detector = cv::SIFT::create();
    ;

    const auto frame1 = geom::ImagedFeatures("../data/SFMedu/B21.jpg", 0, feat_detector);
    const auto frame2 = geom::ImagedFeatures("../data/SFMedu/B22.jpg", 1, feat_detector);
    frame1.PrintInfo();
    frame2.PrintInfo();

    auto pair_matcher = geom::PairMatcher2(frame1, frame2);
    pair_matcher.PrintInfo();
    cv::imshow("pair", pair_matcher.VizImage());

    // 2view geometry
    if (true)
    {
        const auto M1 = pair_matcher.M1();
        const auto M2 = pair_matcher.M2();

        Eigen::Matrix3d K; // camera intrinsic
        double f = 719.5459;
        K << f, 0, 0,
            0, f, 0,
            0, 0, 1;
        Eigen::Matrix3d F;
        pair_matcher.GetFundamentalMatrix(F);

        geom::TwoViewGeometry g2view(M1, M2, F, K);
        const Matrix4d cam1_T_w = g2view.get_cam1_T_w();
        const Matrix4d cam2_T_w = g2view.get_cam2_T_w();
        const MatrixXd pts_w = g2view.get_pts_w();

        if constexpr (true)
        {
            // write results to file
            std::string outfname = "data2/pts3d_w.txt";
            helpers::RawFileIO::write_eigen_matrix(outfname, pts_w.transpose());

            outfname = "data2/wTc1.txt";
            helpers::RawFileIO::write_eigen_matrix(outfname, cam1_T_w.inverse());

            outfname = "data2/wTc2.txt";
            helpers::RawFileIO::write_eigen_matrix(outfname, cam2_T_w.inverse());
        }
    }

    // cv::imshow( "win1", frame1.Image() );
    cv::waitKey(0);
}