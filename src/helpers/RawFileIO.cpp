#include <helpers/RawFileIO.h>

namespace helpers
{

    void RawFileIO::write_image(const std::string &fname, const cv::Mat &img)
    {
        __RawFileIO__write_image_debug_dm(std::cout << "write_image: " << fname << endl);
        cv::imwrite((fname).c_str(), img);
    }

    void RawFileIO::write_string(const std::string &fname, const string &my_string)
    {
        __RawFileIO__write_image_debug_dm(std::cout << "write_string: " << fname << endl);
        std::ofstream outfile(fname);
        outfile << my_string << endl;
        outfile.close();
    }

    void RawFileIO::write_raw_matrix2d(const string &filename, const double *D, int nRows, int nCols)
    {
        // string base = string("/home/mpkuse/Desktop/bundle_adj/dump/datamgr_mat2d_");
        std::ofstream file(filename);
        if (file.is_open())
        {
            int c = 0;
            for (int i = 0; i < nRows; i++)
            {
                file << D[c];
                c++;
                for (int j = 1; j < nCols; j++)
                {
                    file << ", " << D[c];
                    c++;
                }
                file << "\n";
            }
            __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m"
                                                        << "write_Matrix2d to file: " << filename << "\033[0m\n");
        }
        else
        {
            std::cout << "\033[1;31m"
                      << "FAIL TO OPEN FILE for writing: " << filename << "\033[0m\n";
        }
    }

    void RawFileIO::write_raw_matrix1d(const string &filename, const double *D, int n)
    {
        std::ofstream file(filename);
        if (file.is_open())
        {
            file << D[0];
            for (int i = 1; i < n; i++)
                file << ", " << D[i];
            file << "\n";
            __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m"
                                                        << "write_Matrix1d: " << filename << "\033[0m\n");
        }
        else
        {
            std::cout << "\033[1;31m"
                      << "FAIL TO OPEN FILE for writing: " << filename << "\033[0m\n";
        }
    }

    bool RawFileIO::read_eigen_matrix(const std::string &filename, MatrixXd &result)
    {
        int cols = 0, rows = 0;
        const int MAXBUFSIZE = ((int)1e6);
        double buff[MAXBUFSIZE];

        // Read numbers from file into buffer.
        __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m"
                                                    << "read_eigen_matrix: " << filename << "\033[0m");
        ifstream infile;
        infile.open(filename);
        if (!infile)
        {
            cout << "\n\033[1;31m"
                 << "failed to open file " << filename << "\033[0m" << endl;
            return false;
        }
        while (!infile.eof())
        {
            string line;
            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while (!stream.eof())
                stream >> buff[cols * rows + temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
        }

        infile.close();

        rows--;

        // Populate matrix with numbers.
        result = MatrixXd::Zero(rows, cols);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result(i, j) = buff[cols * i + j];

        // return result;
        __RawFileIO__write_image_debug_dm(cout << "\tshape=" << result.rows() << "x" << result.cols() << endl;)

            return true;
    };

    bool RawFileIO::read_eigen_matrix(const std::string &filename, Matrix4d &result)
    {
        int cols = 0, rows = 0;
        const int MAXBUFSIZE = ((int)2000);
        double buff[MAXBUFSIZE];

        // Read numbers from file into buffer.
        __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m"
                                                    << "read_eigen_matrix: " << filename << "\033[0m");
        ifstream infile;
        infile.open(filename);
        if (!infile)
        {
            cout << "\n\033[1;31m"
                 << "failed to open file " << filename << "\033[0m" << endl;
            return false;
        }
        while (!infile.eof())
        {
            string line;
            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while (!stream.eof())
                stream >> buff[cols * rows + temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
        }

        infile.close();

        rows--;

        assert(rows == 4 && cols == 4 && "\n[RawFileIO::read_eigen_matrix( string filename, Matrix4d& result )] The input file is not 4x4");

        // Populate matrix with numbers.
        result = Matrix4d::Zero();
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result(i, j) = buff[cols * i + j];

        // return result;
        __RawFileIO__write_image_debug_dm(cout << "\tshape=" << result.rows() << "x" << result.cols() << endl;) return true;
    };

    bool RawFileIO::read_eigen_matrix(const std::string &filename, Matrix3d &result)
    {
        int cols = 0, rows = 0;
        const int MAXBUFSIZE = ((int)2000);
        double buff[MAXBUFSIZE];

        // Read numbers from file into buffer.
        __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m"
                                                    << "read_eigen_matrix: " << filename << "\033[0m");
        ifstream infile;
        infile.open(filename);
        if (!infile)
        {
            cout << "\n\033[1;31m"
                 << "failed to open file " << filename << "\033[0m" << endl;
            return false;
        }
        while (!infile.eof())
        {
            string line;
            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while (!stream.eof())
                stream >> buff[cols * rows + temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
        }

        infile.close();

        rows--;

        assert(rows == 3 && cols == 3 && "\n[RawFileIO::read_eigen_matrix( string filename, Matrix3d& result )] The input file is not 3x3");

        // Populate matrix with numbers.
        result = Matrix3d::Zero();
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result(i, j) = buff[cols * i + j];

        // return result;
        __RawFileIO__write_image_debug_dm(cout << "\tshape=" << result.rows() << "x" << result.cols() << endl;) return true;
    }

    bool RawFileIO::read_eigen_matrix(const std::string &filename, VectorXi &result)
    {
        int cols = 0, rows = 0;
        const int MAXBUFSIZE = ((int)2000);
        double buff[MAXBUFSIZE];

        // Read numbers from file into buffer.
        __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m"
                                                    << "read_eigen_matrix: " << filename << "\033[0m");
        ifstream infile;
        infile.open(filename);
        if (!infile)
        {
            cout << "\n\033[1;31m"
                 << "failed to open file " << filename << "\033[0m" << endl;
            return false;
        }
        while (!infile.eof())
        {
            string line;
            getline(infile, line);

            int temp_cols = 0;
            stringstream stream(line);
            while (!stream.eof())
                stream >> buff[cols * rows + temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
        }

        infile.close();

        rows--;

        assert(cols == 1 && "\n[RawFileIO::read_eigen_matrix( string filename, VectorXi& result )] The input file is not row-vector");

        // Populate matrix with numbers.
        result = VectorXi::Zero(rows);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result(i, j) = (int)buff[cols * i + j];

        // return result;
        __RawFileIO__write_image_debug_dm(cout << "\tshape=" << result.rows() << "x" << result.cols() << endl;) return true;
    };

    bool RawFileIO::read_eigen_matrix(const std::vector<double> &ary, Matrix4d &result)
    {
        assert(ary.size() == result.rows() * result.cols() && "[RawFileIO::read_eigen_matrix] size of vector<double> need to be equal to the size of Eigen::Matrix");

        for (int i = 0; i < ary.size(); i++)
        {
            result(i / 4, i % 4) = ary[i];
        }
        return true;
    }

    bool RawFileIO::if_file_exist(char *fname)
    {
        ifstream f(fname);
        return f.good();
    }

    bool RawFileIO::if_file_exist(const std::string &fname) { return if_file_exist(fname.c_str()); }

} // namespace helpers