#pragma once

#include <iostream>
#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <helpers/MiscUtils.h>
#include <helpers/CustomAssert.h>

namespace geom
{

    // given a correspondences between 2 view, to compute the camera relative pose
    // Also things related to triangulation.

    class TwoViewGeometry
    {
    public:
        // M1, M2: Correspondences in image-coordinate. 2xN
        // F: Fundamental matrix 
        // K: Camera intrinsics //TODO-future, should take in K1 and K2 in case of different cams
        TwoViewGeometry(const Eigen::MatrixXd &M1_, const Eigen::MatrixXd &M2_, const Eigen::Matrix3d &F_, const Eigen::Matrix3d &K_);

    private:
        const Eigen::MatrixXd M1, M2; //< correspondences in image cordinates (input)
        const Eigen::Matrix3d K;      //< camera intrinsics (input)
        const Eigen::Matrix3d F;      //< Fundamental matrix (input)

        Eigen::Matrix3d E; //< Essential matrix

        Eigen::Matrix3d R1, R2;
        Eigen::Vector3d tr;

        void compute_essential_matrix();
        void decompose_essential_matrix();

        double compute_x1tFx2(const Eigen::MatrixXd &X1, const Eigen::Matrix3d &F, const Eigen::MatrixXd &X2);
    };

} // namespace geom