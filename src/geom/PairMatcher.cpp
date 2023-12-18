#include <geom/PairMatcher.h>

namespace geom
{
    PairMatcher2::PairMatcher2(const ImagedFeatures &frame1_, const ImagedFeatures &frame2_) : frame1(frame1_), frame2(frame2_)
    {
        basic_nn_matching();
        ratio_test(0.8);
        fundamental_matrix_test();
    }

    void PairMatcher2::PrintInfo(const uint8_t verbosity) const
    {
        printf("Seq#%u<>Seq#%u ; ", frame1.Seq(), frame2.Seq());
        if (verbosity == 0)
        {
            printf("Features: %u|%u ; ", frame1.DetectedFeaturesCount(), frame2.DetectedFeaturesCount());
        }
        else
        {
            printf("Input Features: %u|%u ; ", frame1.DetectedFeaturesCount(), frame2.DetectedFeaturesCount());
        }

        if (verbosity == 0)
        {
            printf("Inlier Matches: %lu ; ", m1_retained.size());
        }
        else
        {
            printf("After Ratio Test: %lu ; ", matched1.size());
            printf("After F-Test: %lu ; ", m1_retained.size());
        }
        printf("\n");
    }

    void PairMatcher2::GetFundamentalMatrix(Eigen::Matrix<double, 3, 3> &F) const
    {
        // note: `fundamental_matrix` is of type CV_64F.
        for (int i = 0; i < fundamental_matrix.rows; ++i)
        {
            for (int j = 0; j < fundamental_matrix.cols; ++j)
            {
                F(i, j) = fundamental_matrix.at<double>(i, j);
            }
        }
    }

    const Eigen::MatrixXd PairMatcher2::M1() const
    {
        return m1_retained_eigen;
    }

    const Eigen::MatrixXd PairMatcher2::M2() const
    {
        return m2_retained_eigen;
    }

    const std::vector<geom::FeatIdx> PairMatcher2::M1Idx() const
    {
        return m1_retained_idx;
    }

    const std::vector<geom::FeatIdx> PairMatcher2::M2Idx() const
    {
        M_ASSERT(m2_retained_eigen.cols() == m2_retained_idx.size(), "mismatch between idx size and eigenmat cols");
        return m2_retained_idx;
    }

    const cv::Mat PairMatcher2::VizImage() const
    {
        cv::Mat dst;
        std::string msg = frame1.FileName() + " | " + frame2.FileName();
        helpers::MiscUtils::plot_point_pair(frame1.Image(), m1_retained_eigen, frame1.Seq(),
                                            frame2.Image(), m2_retained_eigen, frame2.Seq(),
                                            dst,
                                            cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),
                                            false, msg);
        return dst;
    }

    void PairMatcher2::basic_nn_matching()
    {
        auto matcher = cv::BFMatcher(cv::NORM_L1);
        matcher.knnMatch(frame1.KeyptsDescriptor(), frame2.KeyptsDescriptor(), nn_matches, 2);
    }

    void PairMatcher2::ratio_test(const float nn_match_ratio)
    {
        matched1.clear();
        matched2.clear();
        const auto &kpts1 = frame1.Keypts();
        const auto &kpts2 = frame2.Keypts();

        const auto &kpts1_idx = frame1.KeyptsIdx();
        const auto &kpts2_idx = frame2.KeyptsIdx();

        for (size_t i = 0; i < nn_matches.size(); i++)
        {
            cv::DMatch first = nn_matches[i][0]; // best match
            float dist1 = nn_matches[i][0].distance;
            float dist2 = nn_matches[i][1].distance;
            if (dist1 < nn_match_ratio * dist2)
            {
                matched1.push_back(kpts1[first.queryIdx]);
                matched2.push_back(kpts2[first.trainIdx]);

                matched1_idx.push_back(kpts1_idx[first.queryIdx]);
                matched2_idx.push_back(kpts2_idx[first.trainIdx]);
            }
        }
    }

    void PairMatcher2::fundamental_matrix_test()
    {
        M_ASSERT(matched1.size() == matched2.size() && matched1.size() > 0u, "expecting matches of same size. now they are: %lu, %lu", matched1.size(), matched2.size());
        M_ASSERT(matched1_idx.size() == matched2_idx.size() && matched1.size() == matched1_idx.size(), "expecting match and matched_idx of same size. now they are: %lu, %lu", matched1.size(), matched1_idx.size());

        helpers::MiscUtils::keypoint_2_point2f(matched1, m1);
        helpers::MiscUtils::keypoint_2_point2f(matched2, m2);

        if constexpr (normalize_for_f_matrix_computation)
        {
            compute_fundamental_matrix_with_normalization();
        }
        else
        {
            compute_fundamental_matrix();
        }

        // filter for inlier
        helpers::MiscUtils::retain_inliers_g(m1, matched_inliers, m1_retained);
        helpers::MiscUtils::retain_inliers_g(m2, matched_inliers, m2_retained);

        // dont forget to do the same for kp_idx
        helpers::MiscUtils::retain_inliers_g(matched1_idx, matched_inliers, m1_retained_idx);
        helpers::MiscUtils::retain_inliers_g(matched2_idx, matched_inliers, m2_retained_idx);

        // convert to eigen for later consumption
        helpers::MiscUtils::point2f_2_eigen(m1_retained, m1_retained_eigen);
        helpers::MiscUtils::point2f_2_eigen(m2_retained, m2_retained_eigen);
    }

    void PairMatcher2::compute_fundamental_matrix()
    {
        matched_inliers = std::vector<uchar>(m1.size(), 0);
        fundamental_matrix = cv::findFundamentalMat(m1, m2, cv::FM_RANSAC, 1.5, 0.9, 4, matched_inliers);
    }

    void PairMatcher2::compute_fundamental_matrix_with_normalization()
    {
        // see: https://sites.cc.gatech.edu/classes/AY2016/cs4476_fall/results/proj3/html/arao83/index.html

        //-- compute mean, stddev of m1 and m2
        M_ASSERT(m1.size() == m2.size(), "expecting equal for m1 and m2 sizes. now=%lu,%lu", m1.size(), m2.size());
        M_ASSERT(m1.size() > 0, "expecting non empty m1, m2");

        cv::Point2f m1_mean = std::accumulate(m1.begin(), m1.end(), cv::Point2f(0, 0)) / static_cast<float>(m1.size());
        cv::Point2f m2_mean = std::accumulate(m1.begin(), m1.end(), cv::Point2f(0, 0)) / static_cast<float>(m2.size());

        cv::Point2f m1_sos(0, 0); // sum of squares
        cv::Point2f m2_sos(0, 0); // sum of squares
        for (const cv::Point2f &point : m1)
        {
            cv::Point2f diff = point - m1_mean;
            m1_sos.x += diff.x * diff.x;
            m1_sos.y += diff.y * diff.y;
        }
        for (const cv::Point2f &point : m2)
        {
            cv::Point2f diff = point - m2_mean;
            m2_sos.x += diff.x * diff.x;
            m2_sos.y += diff.y * diff.y;
        }
        cv::Point2f m1_stddev = cv::Point2f(std::sqrt(m1_sos.x / m1.size()), std::sqrt(m1_sos.y / m1.size()));
        cv::Point2f m2_stddev = cv::Point2f(std::sqrt(m2_sos.x / m2.size()), std::sqrt(m2_sos.y / m2.size()));

        //-- normalize m1, m2
        std::vector<cv::Point2f> m1_normalized, m2_normalized;
        for (const cv::Point2f &point : m1)
        {
            const auto e = (point - m1_mean);
            m1_normalized.push_back(cv::Point2f(e.x / m1_stddev.x, e.y / m1_stddev.y));
        }

        for (const cv::Point2f &point : m2)
        {
            const auto e = (point - m2_mean);
            m2_normalized.push_back(cv::Point2f(e.x / m2_stddev.x, e.y / m2_stddev.y));
        }

        // compute fundamental matrix with normalized m1, m2
        auto fundamental_matrix_x = cv::findFundamentalMat(m1_normalized, m2_normalized,
                                                           cv::FM_RANSAC,
                                                           0.1 /*ransacReprojThreshold*/,
                                                           0.99 /*confidence*/,
                                                           4 /*max itr*/,
                                                           matched_inliers);

        // note: fundamental_matrix_x is of type 64FC1

        //-- build inverse transforms
        cv::Mat T1 = (cv::Mat_<double>(3, 3) << 1.0 / m1_stddev.x, 0., -m1_mean.x / m1_stddev.x,
                      0, 1.0 / m1_stddev.y, -m1_mean.y / m1_stddev.y,
                      0, 0, 1);

        cv::Mat T2 = (cv::Mat_<double>(3, 3) << 1.0 / m2_stddev.x, 0., -m2_mean.x / m2_stddev.x,
                      0, 1.0 / m2_stddev.y, -m2_mean.y / m2_stddev.y,
                      0, 0, 1);

        //-- F_matrix = T2' * F_norm * T1
        fundamental_matrix = T2.t() * fundamental_matrix_x * T1;
    }

    PairMatcher::PairMatcher(const cv::Mat &im1_, const cv::Mat &im2_) : im1(im1_), im2(im2_)
    {
        init_feature_detector();
        detect_features_and_compute_descriptors();
        basic_nn_matching();
        ratio_test(0.8);
        fundamental_matrix_test();
    }

    void PairMatcher::PrintInfo(const uint8_t verbosity) const
    {
        printf("Input Images:> ");
        printf("\t im1:%s | im2:%s\n", helpers::MiscUtils::cvmat_info(im1).c_str(), helpers::MiscUtils::cvmat_info(im2).c_str());

        printf("Detected Features:> %lu|%lu\n", kpts1.size(), kpts2.size());
        printf("After Ratio Test:> %lu|%lu\n", matched1.size(), matched2.size());
        printf("After F-Test:> %lu|%lu\n", m1_retained.size(), m2_retained.size());

        printf("Eigen Matrix Results:> %s | %s\n", helpers::MiscUtils::eigen_info(m1_retained_eigen).c_str(), helpers::MiscUtils::eigen_info(m1_retained_eigen).c_str());
    }

    void PairMatcher::init_feature_detector()
    {
        feat_detector = cv::SIFT::create();
        matcher = cv::BFMatcher(cv::NORM_L1);
    }

    void PairMatcher::detect_features_and_compute_descriptors()
    {
        feat_detector->detectAndCompute(im1, cv::noArray(), kpts1, desc1);
        feat_detector->detectAndCompute(im2, cv::noArray(), kpts2, desc2);
    }

    void PairMatcher::basic_nn_matching()
    {
        matcher.knnMatch(desc1, desc2, nn_matches, 2);
    }

    void PairMatcher::ratio_test(const float nn_match_ratio)
    {
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
    }

    void PairMatcher::fundamental_matrix_test()
    {
        M_ASSERT(matched1.size() == matched2.size() && matched1.size() > 0u, "expecting matches of same size. now they are: %lu, %lu", matched1.size(), matched2.size());
        helpers::MiscUtils::keypoint_2_point2f(matched1, m1);
        helpers::MiscUtils::keypoint_2_point2f(matched2, m2);

        if constexpr (normalize_for_f_matrix_computation)
        {
            compute_fundamental_matrix_with_normalization();
        }
        else
        {
            compute_fundamental_matrix();
        }

        // filter for inlier
        helpers::MiscUtils::retain_inliers(m1, matched_inliers, m1_retained);
        helpers::MiscUtils::retain_inliers(m2, matched_inliers, m2_retained);

        // convert to eigen for later consumption
        helpers::MiscUtils::point2f_2_eigen(m1_retained, m1_retained_eigen);
        helpers::MiscUtils::point2f_2_eigen(m2_retained, m2_retained_eigen);
    }

    void PairMatcher::compute_fundamental_matrix()
    {
        matched_inliers = std::vector<uchar>(m1.size(), 0);
        fundamental_matrix = cv::findFundamentalMat(m1, m2, cv::FM_RANSAC, 1.5, 0.9, 4, matched_inliers);
    }

    void PairMatcher::compute_fundamental_matrix_with_normalization()
    {
        // see: https://sites.cc.gatech.edu/classes/AY2016/cs4476_fall/results/proj3/html/arao83/index.html

        //-- compute mean, stddev of m1 and m2
        M_ASSERT(m1.size() == m2.size(), "expecting equal for m1 and m2 sizes. now=%lu,%lu", m1.size(), m2.size());
        M_ASSERT(m1.size() > 0, "expecting non empty m1, m2");

        cv::Point2f m1_mean = std::accumulate(m1.begin(), m1.end(), cv::Point2f(0, 0)) / static_cast<float>(m1.size());
        cv::Point2f m2_mean = std::accumulate(m1.begin(), m1.end(), cv::Point2f(0, 0)) / static_cast<float>(m2.size());

        cv::Point2f m1_sos(0, 0); // sum of squares
        cv::Point2f m2_sos(0, 0); // sum of squares
        for (const cv::Point2f &point : m1)
        {
            cv::Point2f diff = point - m1_mean;
            m1_sos.x += diff.x * diff.x;
            m1_sos.y += diff.y * diff.y;
        }
        for (const cv::Point2f &point : m2)
        {
            cv::Point2f diff = point - m2_mean;
            m2_sos.x += diff.x * diff.x;
            m2_sos.y += diff.y * diff.y;
        }
        cv::Point2f m1_stddev = cv::Point2f(std::sqrt(m1_sos.x / m1.size()), std::sqrt(m1_sos.y / m1.size()));
        cv::Point2f m2_stddev = cv::Point2f(std::sqrt(m2_sos.x / m2.size()), std::sqrt(m2_sos.y / m2.size()));

        //-- normalize m1, m2
        std::vector<cv::Point2f> m1_normalized, m2_normalized;
        for (const cv::Point2f &point : m1)
        {
            const auto e = (point - m1_mean);
            m1_normalized.push_back(cv::Point2f(e.x / m1_stddev.x, e.y / m1_stddev.y));
        }

        for (const cv::Point2f &point : m2)
        {
            const auto e = (point - m2_mean);
            m2_normalized.push_back(cv::Point2f(e.x / m2_stddev.x, e.y / m2_stddev.y));
        }

        // compute fundamental matrix with normalized m1, m2
        auto fundamental_matrix_x = cv::findFundamentalMat(m1_normalized, m2_normalized,
                                                           cv::FM_RANSAC,
                                                           0.1 /*ransacReprojThreshold*/,
                                                           0.99 /*confidence*/,
                                                           4 /*max itr*/,
                                                           matched_inliers);

        // note: fundamental_matrix_x is of type 64FC1

        //-- build inverse transforms
        cv::Mat T1 = (cv::Mat_<double>(3, 3) << 1.0 / m1_stddev.x, 0., -m1_mean.x / m1_stddev.x,
                      0, 1.0 / m1_stddev.y, -m1_mean.y / m1_stddev.y,
                      0, 0, 1);

        cv::Mat T2 = (cv::Mat_<double>(3, 3) << 1.0 / m2_stddev.x, 0., -m2_mean.x / m2_stddev.x,
                      0, 1.0 / m2_stddev.y, -m2_mean.y / m2_stddev.y,
                      0, 0, 1);

        //-- F_matrix = T2' * F_norm * T1
        fundamental_matrix = T2.t() * fundamental_matrix_x * T1;
    }

    void PairMatcher::get_fundamental_matrix(Eigen::Matrix<double, 3, 3> &F)
    {
        // note: `fundamental_matrix` is of type CV_64F.
        for (int i = 0; i < fundamental_matrix.rows; ++i)
        {
            for (int j = 0; j < fundamental_matrix.cols; ++j)
            {
                F(i, j) = fundamental_matrix.at<double>(i, j);
            }
        }

        if constexpr (false)
        {
            printf("fundamental_matrix: %s\n", helpers::MiscUtils::cvmat_info(fundamental_matrix).c_str());
            std::cout << "opencv f: \n"
                      << fundamental_matrix << std::endl;
            std::cout << "eigen f: \n"
                      << F << std::endl;
        }
    }

};