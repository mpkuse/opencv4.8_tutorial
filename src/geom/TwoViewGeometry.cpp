#include <geom/TwoViewGeometry.h>

namespace geom
{
    TwoViewGeometry::TwoViewGeometry(const Eigen::MatrixXd &M1_, const Eigen::MatrixXd &M2_, const Eigen::Matrix3d &F_, const Eigen::Matrix3d &K_) : M1(M1_), M2(M2_), F(F_), K(K_)
    {
        compute_essential_matrix();
        decompose_essential_matrix();

        // TODO: check cheirality  and only retain one of the 4 options
        
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

        if constexpr (true)
        {
            // TODO: removal
            std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);

            std::cout << "R1=\"\"\"\n"
                      << R1 << "\"\"\" " << std::endl;
            std::cout << "R2=\"\"\"\n"
                      << R2 << "\"\"\"" << std::endl;
            std::cout << "tr=\"\"\"\n"
                      << tr << "\"\"\"" << std::endl;
        }
    }

}