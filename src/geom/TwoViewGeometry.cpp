#include <geom/TwoViewGeometry.h>

namespace geom
{
    TwoViewGeometry::TwoViewGeometry(const Eigen::MatrixXd &M1_, const Eigen::MatrixXd &M2_, const Eigen::Matrix3d &F_, const Eigen::Matrix3d &K_) : M1(M1_), M2(M2_), F(F_), K(K_)
    {
        compute_essential_matrix();
        decompose_essential_matrix();

        choose_solution_with_chirality_condition();
    }

    uint32_t TwoViewGeometry::count_positive_z(const MatrixXd &pts3d)
    {
        uint32_t n = 0u;
        for (uint32_t c = 0; c < pts3d.cols(); c++)
        {
            if (pts3d(2, c) > 0)
            {
                n++;
            }
        }
        return n;
    }

    bool TwoViewGeometry::chirality_check(const Matrix4d &cam_T_w, const Vector4d &X_w)
    {
        double d = 0.;
        for (uint32_t i = 0; i < 3; i++)
            d += cam_T_w(2, i) * (X_w(i) - cam_T_w(i, 3));

        return d > 0.;
    }

    uint32_t TwoViewGeometry::count_positive_chirality(const Matrix4d &cam_T_w, const MatrixXd &pts3d)
    {
        uint32_t n = 0;
        for (uint32_t i = 0; i < pts3d.cols(); i++)
        {
            Vector4d X_w = pts3d.col(i);
            n = chirality_check(cam_T_w, X_w) ? n + 1 : n;
        }
        return n;
    }

    double TwoViewGeometry::compute_x1tFx2(const Eigen::MatrixXd &X1, const Eigen::Matrix3d &FF, const Eigen::MatrixXd &X2)
    {
        M_ASSERT(X1.rows() == 2 || X1.rows() == 3, "Expecting x1 as 2xN or 3xN matrix");
        M_ASSERT(X2.rows() == 2 || X2.rows() == 3, "Expecting x1 as 2xN or 3xN matrix");
        M_ASSERT(X1.cols() == X2.cols(), "Expected same N for X1(=%lu) and X2(=%lu)", X1.cols(), X2.cols());
        double residue = 0.;
        auto N = X1.cols();
        for (auto i = 0u; i < N; i++)
        {
            double r = X1.col(i).transpose() * FF * X2.col(i);
            residue += (r * r);
        }
        return residue;
    }

    void TwoViewGeometry::triangulate_linear(const Eigen::Matrix4d &T1, const Eigen::MatrixXd &pts1,
                                             const Eigen::Matrix4d &T2, const Eigen::MatrixXd &pts2,
                                             Eigen::MatrixXd &pts3d)
    {
        constexpr uint32_t n_view = 2;
        const uint32_t n_pts = pts1.cols();
        M_ASSERT(pts2.cols() == n_pts, "Expecting same number of inptut\
                projected pts. pts1.cols=%ld, pts2.cols=%ld",
                 pts1.cols(), pts2.cols());
        M_ASSERT(pts1.rows() == 2 || pts1.rows() == 3, "pts1 has to have 2 or 3 rows");
        M_ASSERT(pts2.rows() == 2 || pts2.rows() == 3, "pts1 has to have 2 or 3 rows");
        M_ASSERT(n_pts > 5, "Insufficient number of correspondences");

        // output:
        pts3d = Eigen::MatrixXd::Constant(4, n_pts, 0.0);

        using MatAType = Eigen::Matrix<double, 2 * n_view, 4>;
        MatAType A;

        uint32_t j = 0; //< j^{th} pts
        for (uint32_t j = 0; j < n_pts; j++)
        {
            const auto u1 = pts1(0, j);
            const auto v1 = pts1(1, j);
            const auto u2 = pts2(0, j);
            const auto v2 = pts2(1, j);

            A.row(0) = T1.row(2) * u1 - T1.row(0);
            A.row(1) = T1.row(2) * v1 - T1.row(1);
            A.row(2) = T2.row(2) * u2 - T2.row(0);
            A.row(3) = T2.row(2) * v2 - T2.row(1);

            // justification for normalization:
            // https://imkaywu.github.io/blog/2017/07/triangulation/
            for (uint32_t r = 0; r < 4; r++)
            {
                A.row(r).normalize();
            }

            // solve AX =0 with svd
            Eigen::JacobiSVD<MatAType> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector4d X = svd.matrixV().col(3);
            X = X / X(3); // to non homogeneous representation

            pts3d.col(j) = X; // set it into the solution
        }
    }

    void TwoViewGeometry::compute_essential_matrix()
    {
        E = K.transpose() * F * K;
    }

    void TwoViewGeometry::decompose_essential_matrix()
    {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Vector3d S = svd.singularValues();
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Matrix3d W;
        W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

        R1 = U * W.transpose() * V.transpose();
        R2 = U * W * V.transpose();
        tr = U.col(2);

        // Avoiding twist-rotation
        if (R1.determinant() < 0)
        {
            R1 = -R1;
        }

        if (R2.determinant() < 0)
        {
            R2 = -R2;
        }
    }

    void TwoViewGeometry::choose_solution_with_chirality_condition()
    {
        // see section3 and section4 of
        // https://cmsc426.github.io/sfm/
        constexpr bool verbose = false;

        //-- correspondences in normalized image coordinates
        M1_normalized_image_coordinates = K.inverse() * M1;
        M2_normalized_image_coordinates = K.inverse() * M2;

        //-- The 4 solutions of camera poses obtained by decomposition of
        //      essential matrix
        Eigen::Matrix4d eye = Eigen::Matrix4d::Identity();
        std::vector<Eigen::Matrix4d> Ti(4);    //< 4 options
        std::vector<Eigen::MatrixXd> pts3d(4); // pointcloud from 4 options
        Ti[0] = to_transform(R1, tr);
        Ti[1] = to_transform(R2, tr);
        Ti[2] = to_transform(R1, -tr);
        Ti[3] = to_transform(R2, -tr);

        //-- triangulate the correspondences for each of the 4 solution.
        //   only one solution (hopefully) will pass the chirality check
        uint32_t op_with_positive_chirality = 100u;
        for (auto op = 0u; op < 4; op++)
        {
            triangulate_linear(eye, M1_normalized_image_coordinates,
                               Ti[op], M2_normalized_image_coordinates, pts3d[op]);
            M_ASSERT(pts3d[op].cols() == M1.cols(), "M1.cols=%lu, pts3d.cols=%lu", M1.cols(), pts3d[op].cols());

            // chirality check
            uint32_t n_pts = pts3d[op].cols();
            uint32_t n_positive_z = count_positive_z(pts3d[op]);
            uint32_t n_chirality_cam1 = count_positive_chirality(eye, pts3d[op]);
            uint32_t n_chirality_cam2 = count_positive_chirality(Ti[op], pts3d[op]);

            float cam1_poschirality_ratio = n_chirality_cam1 / float(n_pts);
            float cam2_pos_chirality_ratio = n_chirality_cam1 / float(n_pts);
            if (cam1_poschirality_ratio > 0.8 && cam2_pos_chirality_ratio > 0.8)
            {
                if constexpr (verbose)
                {
                    printf("***op=%u is the solution\n", op);
                }
                op_with_positive_chirality = op;
            }
        }

        if (op_with_positive_chirality > 3u)
        {
            M_ASSERT(false, "cannnot find a solution for +ve chirality");
        }

        //-- The accepted solution
        cam1_T_w = eye;
        cam2_T_w = Ti[op_with_positive_chirality];
        pts_w = pts3d[op_with_positive_chirality];
    }

    Eigen::Matrix4d TwoViewGeometry::to_transform(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
    {
        Eigen::Matrix4d T;
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;
        T.row(3) << 0, 0, 0, 1;
        return T;
    }

}