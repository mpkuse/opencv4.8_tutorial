#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <iomanip>


// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

using namespace Eigen;
using namespace std;

namespace helpers
{

#define __RawFileIO__write_image_debug_dm(msg) msg;

  class RawFileIO
  {
  public:
    static void write_image(const std::string &fname, const cv::Mat &img);
    static void write_string(const std::string &fname, const string &my_string);

    // templated static function canot only exist in header files.
    template <typename Derived>
    static void write_eigen_matrix(const string &filename, const MatrixBase<Derived> &a)
    {
      // string base = string("/home/mpkuse/Desktop/bundle_adj/dump/datamgr_mateigen_");
      std::ofstream file(filename);
      if (file.is_open())
      {
        // file << a.format(CSVFormat) << endl;
        file << a << endl;
        __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m"
                                                    << "write_EigenMatrix: " << filename << "    size=" << a.rows() << "x" << a.cols() << "\033[0m\n";)
      }
      else
      {
        cout << "\033[1;31m"
             << "FAIL TO OPEN FILE for writing: " << filename << "\033[0m\n";
      }
    }

    template <typename Derived>
    static void print_eigen_matrix(const std::string &msg, const Eigen::MatrixBase<Derived> &matrix)
    {
      std::cout << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);
      std::cout << msg << " " << matrix.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]")) << std::endl;
    }

    static void write_raw_matrix2d(const string &filename, const double *D, int nRows, int nCols);
    static void write_raw_matrix1d(const string &filename, const double *D, int n);

    static bool read_eigen_matrix(const std::string &filename, MatrixXd &result);
    static bool read_eigen_matrix(const std::string &filename, Matrix4d &result);
    static bool read_eigen_matrix(const std::string &filename, Matrix3d &result);
    static bool read_eigen_matrix(const std::string &filename, VectorXi &result);

    ///< read the flat vector ary as a rowmajor matrix.
    /// [ 1, 2, 3, 4,5,6...,16 ] ==> [ [1,2,3,4], [5,6,7,8], [9,10,11,12], [13,14,15,16] ]
    /// TODO: Have a flag to read interpret the 1d array as a colmajor.
    static bool read_eigen_matrix(const std::vector<double> &ary, Matrix4d &result);

    bool if_file_exist(char *fname);
    bool if_file_exist(const std::string &fname);
  };

} // namespace helpers