#include <geom/TwoViewGeometry.h>

namespace geom
{
    TwoViewGeometry::TwoViewGeometry(const Eigen::MatrixXd &M1_, const Eigen::MatrixXd &M2_, const Eigen::Matrix3d &F_, const Eigen::Matrix3d &K_) : M1(M1_), M2(M2_), F(F_), K(K_)
    {
        printf("-------TwoViewGeometry\n");
        compute_essential_matrix();
        decompose_essential_matrix();

        // TODO: check cheirality  and only retain one of the 4 options
        M1_normalized_image_coordinates = K.inverse() * M1;
        M2_normalized_image_coordinates = K.inverse() * M2;
        Eigen::Matrix4d eye = Eigen::Matrix4d::Identity();
        std::vector<Eigen::Matrix4d> Ti(4);    //< 4 options
        std::vector<Eigen::MatrixXd> pts3d(4); // pointcloud from 4 options
        Ti[0] = to_transform(R1, tr);          //.inverse();
        Ti[1] = to_transform(R2, tr);          //.inverse();
        Ti[2] = to_transform(R1, -tr);         //.inverse();
        Ti[3] = to_transform(R2, -tr);         //.inverse();

        for (auto op = 0u; op < 4; op++)
        {
            triangulate_linear(eye, M1_normalized_image_coordinates,
                               Ti[op], M2_normalized_image_coordinates, pts3d[op]);

            if constexpr (true)
            {
                // TODO: removal
                std::string outfname = "data/pts3d_" + std::to_string(op) + ".txt";
                helpers::RawFileIO::write_eigen_matrix(outfname, pts3d[op].transpose());

                outfname = "data/wTc_" + std::to_string(op) + ".txt";
                helpers::RawFileIO::write_eigen_matrix(outfname, Ti[op].inverse());
            }
        }
        printf("-------END TwoViewGeometry\n");
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
        if constexpr (false)
        {
            // TODO: removal
            helpers::RawFileIO::print_eigen_matrix("T1=", T1);
            helpers::RawFileIO::print_eigen_matrix("T2=", T2);
            std::cout << "pts1.info: " << helpers::MiscUtils::eigen_info(pts1) << std::endl;
            std::cout << "pts2.info: " << helpers::MiscUtils::eigen_info(pts2) << std::endl;
        }

        constexpr uint32_t n_view = 2;
        const uint32_t n_pts = pts1.cols();
        M_ASSERT(pts2.cols() == n_pts, "Expecting same number of inptut\
                projected pts. pts1.cols=%ld, pts2.cols=%ld",
                 pts1.cols(), pts2.cols());
        M_ASSERT(pts1.rows() == 2 || pts1.rows() == 3, "pts1 has to have 2 or 3 rows");
        M_ASSERT(pts2.rows() == 2 || pts2.rows() == 3, "pts1 has to have 2 or 3 rows");

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

        if constexpr (false)
        {
            // TODO: removal
            helpers::RawFileIO::print_eigen_matrix("R1", R1);
            helpers::RawFileIO::print_eigen_matrix("R2", R2);
            helpers::RawFileIO::print_eigen_matrix("tr", tr);
   
            std::cout << "det(R1) = " << R1.determinant() << std::endl;
            std::cout << "det(R2) = " << R2.determinant() << std::endl;
            std::cout << "Singular vals of E: " << S << std::endl; 

            std::cout << "UZUt :\n" << U * W.transpose() * U.transpose() << std::endl;
        }
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