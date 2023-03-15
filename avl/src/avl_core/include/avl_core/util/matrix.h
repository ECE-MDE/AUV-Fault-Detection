//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for Eigen matrix manipulation.
//==============================================================================

#ifndef MATRIX_H
#define MATRIX_H

// C++ includes
#include <string>
#include <fstream>
#include <iomanip>
#include <random>
#include <chrono>

// Eigen includes
#include <Eigen/Core>
#include <Eigen/SVD>
using namespace Eigen;

// Util functions
#include "math.h"
#include "byte.h"
#include "vector.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        round_all
// Description: Rounds all terms in a matrix to a given number of decimals.
// Arguments:   - m: Matrix to round.
//              - decimals: Number of decimals to round to.
// Returns:     Matrtix with all terms rounded.
//------------------------------------------------------------------------------
MatrixXd round_all(MatrixXd m, unsigned char decimals);

//------------------------------------------------------------------------------
// Name:        linspace
// Description: Creates a linearly spaced vector of values between min and max
//              with n elements.
// Arguments:   - min: Minimum value.
//              - max: Maximum value.
//              - n: Number of elements.
// Returns :    Linearly spaced vector of n values.
//------------------------------------------------------------------------------
VectorXd linspace(double min, double max, int n);

//--------------------------------------------------------------------------
// Name:        logspace
// Description: Generates a log-spaced set of values between the start and
//              end. Generates up to the specified number of steps, but
//              duplicates are removed.
// Arguments:   - start: Start value.
//              - end: End value.
//              - n_max: Maximum number of values to generate.
// Returns:     Log-spaced values.
//--------------------------------------------------------------------------
VectorXi logspace(int start, int end, int n_max);

//------------------------------------------------------------------------------
// Name:        randn
// Description: Generates a matrix of Gaussian noise with zero mean and a
//              specified covariance.
// Arguments:   - rows: Number of rows in output matrix.
//              - cols: Number of cols in output matrix.
//              - cov: Noise covariance.
// Returns:     Matrix of Gaussian noise.
//------------------------------------------------------------------------------
MatrixXd randn(int rows, int cols, double cov);

//------------------------------------------------------------------------------
// Name:        randn
// Description: Generates a matrix of Gaussian noise with zero mean and a
//              specified covariance.
// Arguments:   - cols: Number of cols in output matrix (n).
//              - cov: Noise covariance matrix (r x r).
// Returns:     Matrix of Gaussian noise (r x n).
//------------------------------------------------------------------------------
MatrixXd randn(int cols, MatrixXd cov);

//------------------------------------------------------------------------------
// Name:        rand
// Description: Generates a matrix of uniformally distributed noise between
//              a specified min and max value.
// Arguments:   - rows: Number of rows in output matrix.
//              - cols: Number of cols in output matrix.
//              - min: Minimum generated number.
//              - max: Maximum generated number.
// Returns:     Matrix of uniformally distributed noise.
//------------------------------------------------------------------------------
MatrixXd rand(int rows, int cols, double min, double max);

//------------------------------------------------------------------------------
// Name:        svd
// Description: Performs singular value decomposition on a matrix.
// Arguments:   - A: Matrix to get singular values of.
// Returns :    Singular values of A.
//------------------------------------------------------------------------------
VectorXd svd(const MatrixXd A);

//------------------------------------------------------------------------------
// Name:        matrix_l2_norm
// Description: Gets the L2 norm of a matrix, which its largest singular value.
// Arguments:   - A: Matrix to get L2 norm of.
// Returns :    Matrix L2 norm of A.
//------------------------------------------------------------------------------
double matrix_l2_norm(MatrixXd A);

//------------------------------------------------------------------------------
// Name:        so2_exp
// Description: Exponential map on SO(2).
// Arguments:   - phi: Scalar to get SO(2) exponential mapping of.
// Returns:     Matrix of exponential mapping of phi to SO(2).
//------------------------------------------------------------------------------
Matrix2d so2_exp(double phi);

//------------------------------------------------------------------------------
// Name:        so2_log
// Description: Log map on SO(2).
// Arguments:   - C: Matrix in SO(2) to get log mapping of.
// Returns:     Scalar of log mapping of C.
//------------------------------------------------------------------------------
double so2_log(Matrix2d C);

//------------------------------------------------------------------------------
// Name:        so3_exp
// Description: Exponential map on SO(3).
// Arguments:   - phi: Vector to get SO(3) exponential mapping of.
// Returns:     Matrix of exponential mapping of phi to SO(3).
//------------------------------------------------------------------------------
Matrix3d so3_exp(Vector3d phi);

//------------------------------------------------------------------------------
// Name:        so3_log
// Description: Log map on SO(3).
// Arguments:   - C: Matrix in SO(3) to get log mapping of.
// Returns:     Vector of log mapping of C.
//------------------------------------------------------------------------------
Vector3d so3_log(Matrix3d C);

//------------------------------------------------------------------------------
// Name:        so3_left_jacobian
// Description: Left Jacobian of SO(3).
// Arguments:   - phi: Vector to get left Jacobian of.
// Returns:     Matrix of left Jacobian.
//------------------------------------------------------------------------------
Matrix3d so3_left_jacobian(Vector3d phi);

//------------------------------------------------------------------------------
// Name:        so3_inv_left_jacobian
// Description: Inverse left Jacobian of SO(3).
// Arguments:   - phi: Vector to get inverse left Jacobian of.
// Returns:     Matrix of inverse left Jacobian.
//------------------------------------------------------------------------------
Matrix3d so3_inv_left_jacobian(Vector3d phi);

//------------------------------------------------------------------------------
// Name:        sek3_inv
// Description: Inverse function for SEk(3).
// Arguments:   - chi: ???.
// Returns:     Inverse of input for SEk(3).
//------------------------------------------------------------------------------
MatrixXd sek3_inv(MatrixXd T);

//------------------------------------------------------------------------------
// Name:        sek3_exp
// Description: Exponential function for SEk(3).
// Arguments:   - xi: ???.
// Returns:     Matrix of exponential mapping of xi to SEk(3).
//------------------------------------------------------------------------------
MatrixXd sek3_exp(VectorXd xi);

//------------------------------------------------------------------------------
// Name:        sek3_inv
// Description: Log function for SEk(3).
// Arguments:   - chi: ???.
// Returns:     Log of input for SEk(3).
//------------------------------------------------------------------------------
VectorXd sek3_log(MatrixXd chi);

//------------------------------------------------------------------------------
// Name:        sek3_left_jacobian
// Description: Left Jacobian of SEk(3).
// Arguments:   - phi: Vector to get left Jacobian of.
// Returns:     Matrix of left Jacobian.
//------------------------------------------------------------------------------
MatrixXd sek3_left_jacobian(VectorXd phi);

//------------------------------------------------------------------------------
// Name:        sek3_inv_left_jacobian
// Description: Inverse left Jacobian of SEk(3).
// Arguments:   - phi: Vector to get inverse left Jacobian of.
// Returns:     Matrix of inverse left Jacobian.
//------------------------------------------------------------------------------
MatrixXd sek3_inv_left_jacobian(VectorXd phi);

//------------------------------------------------------------------------------
// Name:        to_string
// Description: Converts a vector of doubles to a string listing the elements of
//              the vector.
// Arguments:   - v: Vector to be converted to a string.
// Returns:     String listing the elements of the vector.
//------------------------------------------------------------------------------
std::string to_string(VectorXd v, int precision, std::string delim=" ");

//------------------------------------------------------------------------------
// Name:        csv_to_matrix
// Description: Converts CSV file containing matrix elements to an Eigen matrix.
//              The CSV file must have a row of matrix data on each line, where
//              each value is separated by a delimiter.
// Arguments:   - file_path: Path to the CSV file
//              - delim: matrix element delimiter, default comma
// Returns:     CSV file data as an Eigen matrix.
//------------------------------------------------------------------------------
MatrixXd csv_to_matrix(const std::string& file_path, const char& delim = ',',
    const size_t& start_row = 0);

//------------------------------------------------------------------------------
// Name:        matrix_to_csv
// Description: Writes an Eigen matrix to a CSV file containing matrix elements.
// Arguments:   - file_path: Path to save the CSV file to.
//              - m: Matrix to write to CSV file.
//              - mode: in, out, append... Default: std::ifstream::out
//------------------------------------------------------------------------------
void matrix_to_csv(const std::string& file_path, const MatrixXd& m,
    bool truncate = true);

//------------------------------------------------------------------------------
// Name:        allan_variance
// Description: Calculates the overlapping Allan variance down the columns of
//              the input matrix of data. In other words, each column should
//              contain the data for one sensor. Reference:
//    https://telesens.co/wp-content/uploads/2017/05/AllanVariance5087-1.pdf
// Arguments:   - theta: Matrix of integrated sensor measurements.
//              - M: Vector of cluster sizes to calculate Allan variance at.
//              - fs: Data sampling frequency in Hz.
// Returns:     Allan variance
//------------------------------------------------------------------------------
MatrixXd allan_variance(const MatrixXd& theta, const VectorXi& M, double fs);

//------------------------------------------------------------------------------
// Name:        autocorrelation
// Description: Calculates the autocorrelation of the input vector of data.
// Arguments:   - x: Vector of data to calculate autocorrelation of
// Returns:     Vector of single sided autocorrelation values.
//------------------------------------------------------------------------------
VectorXd autocorrelation(const VectorXd& x, const VectorXi& M);

//------------------------------------------------------------------------------
// Name:        euler_cov_to_q_cov
// Description: Converts an Euler angle covariancematrix to a quaternion
//              covariance matrix.
// Arguments:   - euler: Vector containing Euler angles in radians.
//              - euler_cov: Matrix containing Euler angle covariance in
//                radians^2.
// Returns :    Quaternion covariance matrix.
//------------------------------------------------------------------------------
Matrix4d euler_cov_to_q_cov(const Vector3d& euler, const Matrix3d& euler_cov);

//------------------------------------------------------------------------------
// Name:        orthonormalize
// Description: Orthonormalizes a rotation matrix (Equations 5.78 to 5.80).
// Arguments:   - C: Rotation matrix to orthonormalize.
// Returns :    Orthonormalized rotation matrix.
//------------------------------------------------------------------------------
Matrix3d orthonormalize(Matrix3d C);

//------------------------------------------------------------------------------
// Name:        hcat
// Description: Horizontally concatenates two matrices.
// Arguments:   - A: First matrix to concatenate.
//              - B: Second matrix to concatenate.
// Returns :    Concatenated matrix.
//------------------------------------------------------------------------------
MatrixXd hcat(const MatrixXd& A, const MatrixXd& B);

//------------------------------------------------------------------------------
// Name:        vcat
// Description: Vertically concatenates two matrices.
// Arguments:   - A: First matrix to concatenate.
//              - B: Second matrix to concatenate.
// Returns :    Concatenated matrix.
//------------------------------------------------------------------------------
MatrixXd vcat(const MatrixXd& A, const MatrixXd& B);

//==============================================================================
//                          TEMPLATE FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        skew
// Description: Creates a 3x3 skew-symmetric matrix from a 3x1 vector.
// Arguments:   - x: 3x1 vector to make skew-symmetric matrix
// Returns :    Skew-symmetric matrix.
//------------------------------------------------------------------------------
template<typename Derived>
Matrix<typename Derived::Scalar, 3, 3> skew(const MatrixBase<Derived>& v)
{

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3)
    Matrix<typename Derived::Scalar, 3, 3> out;
    out <<   0, -v(2), v(1),
           v(2),   0, -v(0),
          -v(1), v(0),   0;
    return out;

}

//------------------------------------------------------------------------------
// Name:        inv_skew
// Description: Gets the 3x1 vector that was used to form the 3x3 skew-symmetric
//              matrix.
// Arguments:   - x: 3x3 skew-symmetric matrix to get vector from
// Returns :    Vector that was used to form the 3x3 skew-symmetric matrix.
//------------------------------------------------------------------------------
template<typename Derived>
Matrix<typename Derived::Scalar, 3, 1> inv_skew(const MatrixBase<Derived>& x)
{

    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,3)
    Matrix<typename Derived::Scalar, 3, 1> out;
    out << x(2,1), x(0,2), x(1,0);
    return out;

}

//------------------------------------------------------------------------------
// Name:        chol
// Description: Performs a Cholesky decomposition of a symmetric, positive
//              definite matrix A such that A = LL^T, where L is lower
//              triangular.
// Arguments:   - A: Symmetric, positive definite matrix to get Cholesky
//                   decomposition of.
// Returns :    Lower triangular Cholesky decomposition of A.
//------------------------------------------------------------------------------
template<typename Derived>
MatrixBase<Derived> chol(const MatrixBase<Derived>& A)
{
    return A.llt().matrixL();
}


//------------------------------------------------------------------------------
// Name:        euler_to_matrix
// Description: Converts Euler angles in degrees or radians to a rotation matrix
//              using the 3-2-1 rotation scheme. See "Principles of GNSS,
//              Inertial, and Multisensor Integrated Navigation Systems,
//              Second Edition" by Paul D. Groves.
// Arguments:   - theta: vector of Euler angles in degrees or radians
//              - degrees: flag indicating that angles are in degrees instead of
//                radians.
// Returns :    3x3 rotation matrix corresponding to the Euler angles.
//------------------------------------------------------------------------------
template<typename T>
Eigen::Matrix<T, 3, 3> euler_to_matrix(std::vector<T> theta,
    bool degrees = false)
{

    // If the angles are in degrees, convert them to radians
    if (degrees)
    {
        theta.at(0) = deg_to_rad(theta.at(0));
        theta.at(1) = deg_to_rad(theta.at(1));
        theta.at(2) = deg_to_rad(theta.at(2));
    }

    // Precalculate sines and cosines of the Euler angles
    T sin_phi = sin(theta.at(0));
    T cos_phi = cos(theta.at(0));
    T sin_theta = sin(theta.at(1));
    T cos_theta = cos(theta.at(1));
    T sin_psi = sin(theta.at(2));
    T cos_psi = cos(theta.at(2));

    // Calculate coordinate transformation matrix (Equation 2.22)
    Eigen::Matrix<T, 3, 3> C;
    C(0,0) = cos_theta * cos_psi;
    C(0,1) = cos_theta * sin_psi;
    C(0,2) = -sin_theta;
    C(1,0) = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi;
    C(1,1) = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi;
    C(1,2) = sin_phi * cos_theta;
    C(2,0) = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi;
    C(2,1) = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi;
    C(2,2) = cos_phi * cos_theta;

    return C;

}

//------------------------------------------------------------------------------
// Name:        euler_to_matrix
// Description: Converts Euler angles in degrees or radians to a rotation matrix
//              using the 3-2-1 rotation scheme. See "Principles of GNSS,
//              Inertial, and Multisensor Integrated Navigation Systems,
//              Second Edition" by Paul D. Groves.
// Arguments:   - roll: roll angle in degrees or radians
//              - pitch: pitch angle in degrees or radians
//              - yaw: yaw angle in degrees or radians
//              - degrees: flag indicating that angles are in degrees instead of
//                radians.
// Returns :    3x3 rotation matrix corresponding to the Euler angles.
//------------------------------------------------------------------------------
template<typename T>
Eigen::Matrix<T, 3, 3> euler_to_matrix(T roll, T pitch, T yaw,
    bool degrees = false)
{
    std::vector<T> theta;
    theta.push_back(roll);
    theta.push_back(pitch);
    theta.push_back(yaw);
    return euler_to_matrix(theta, degrees);
}

//------------------------------------------------------------------------------
// Name:        euler_to_matrix
// Description: Converts Euler angles in degrees or radians to a rotation matrix
//              using the 3-2-1 rotation scheme. See "Principles of GNSS,
//              Inertial, and Multisensor Integrated Navigation Systems,
//              Second Edition" by Paul D. Groves.
// Arguments:   - theta: vector of Euler angles in degrees or radians
//              - degrees: flag indicating that angles are in degrees instead of
//                radians.
// Returns :    3x3 rotation matrix corresponding to the Euler angles.
//------------------------------------------------------------------------------
template<typename T>
Eigen::Matrix<T, 3, 3> euler_to_matrix(Matrix<T, 3, 1> theta,
    bool degrees = false)
{
    return euler_to_matrix(theta(0), theta(1), theta(2), degrees);
}

//------------------------------------------------------------------------------
// Name:        matrix_to_euler
// Description: Converts a rotation matrix to Euler angles in degrees or radians
//              using the 3-2-1 rotation scheme. See "Principles of GNSS,
//              Inertial, and Multisensor Integrated Navigation Systems,
//              Second Edition" by Paul D. Groves.
// Arguments:   - R: 3x3 rotation matrix
//              - degrees: flag indicating that angles should be output in
//                degrees instead of radians.
// Returns :    Vector of Euler angles in radians or degrees corresponding to
//              the rotation matrix.
//------------------------------------------------------------------------------
template<typename T>
Matrix<T, 3, 1> matrix_to_euler(Matrix<T, 3, 3> R, bool degrees = false)
{

    // Calculate the Euler angles (Equation 2.23)
    double roll = atan2(R(1, 2), R(2, 2));
    double pitch = asin(-R(0, 2));
    double yaw = atan2(R(0, 1), R(0, 0));

    // If the angles should be in degrees, convert them
    if (degrees)
    {
        roll = rad_to_deg(roll);
        pitch = rad_to_deg(pitch);
        yaw = rad_to_deg(yaw);
    }

    Matrix<T, 3, 1> theta;
    theta << roll, pitch, yaw;
    return theta;

}

//------------------------------------------------------------------------------
// Name:        from_bytes
// Description: Converts a vector of bytes to an Eigen matrix with the
//              parameters given by the template arguments. The input bytes
//              should be in big-endian order. Throws a std::runtime_error if
//              the number of bytes in the vector of bytes differs from the
//              required number of bytes for the matrix size.
// Arguments:   - bytes: vector of bytes to be converted is big-endian order
// Returns:     Eigen matrix from the given bytes.
//------------------------------------------------------------------------------
template<typename T, size_t rows, size_t cols>
Eigen::Matrix<T, rows, cols> from_bytes(std::vector<uint8_t> bytes)
{

    // Calculate the total number of bytes required for the vector of bytes
    size_t num_bytes = rows*cols*sizeof(T);

    // Check that there are enough bytes in the vector to form the matrix
    if (bytes.size() != num_bytes)
    {
        throw std::runtime_error("bytes_to_matrix: invalid number of bytes for "
            "conversion from bytes");
    }

    // If the system is little-endian, we need to reverse the order of the
    // bytes to make them little-endian before we convert them
    if (!system_is_big_endian())
        for (size_t i = 0; i < num_bytes; i+=sizeof(T))
            std::reverse(bytes.begin()+i, bytes.begin()+i+sizeof(T));

    // Interpret the vector of bytes as an array of the specified type
    T* array = reinterpret_cast<T*>(&bytes[0]);

    // Construct the Eigen matrix from the array
    return Eigen::Matrix<T, rows, cols>(array).transpose();

}

//------------------------------------------------------------------------------
// Name:        to_bytes
// Description: Converts an Eigen matrix to a vector of bytes. Outputs in
//              big-endian order unless specified.
// Arguments:   - var: variable to be converted to bytes
//              - little_endian: true to set the output to be little endian
// Returns:     Vector of bytes from the given Eigen matrix.
//------------------------------------------------------------------------------
template<typename T, size_t rows, size_t cols>
std::vector<uint8_t> to_bytes(Eigen::Matrix<T, rows, cols> matrix,
    bool little_endian=false)
{

    // Calculate the total number of bytes required for the vector of bytes
    size_t num_bytes = rows*cols*sizeof(T);

    // Reinterpret the matrix's array of values as an array of bytes
    uint8_t* i_start = reinterpret_cast<uint8_t*>(matrix.data());

    // Construct a vector of bytes from the array of bytes using the required
    // number of bytes
    std::vector<uint8_t> bytes(i_start, i_start+num_bytes);

    // Reverse the bytes if the system's endianness is not the endianness that
    // we want.
    if ((system_is_big_endian() && little_endian) ||
        (!system_is_big_endian() && !little_endian))
        for (size_t i = 0; i < num_bytes; i+=sizeof(T))
            std::reverse(bytes.begin()+i, bytes.begin()+i+sizeof(T));

    return bytes;

}

//------------------------------------------------------------------------------
// Name:        to_std_vector
// Description: Converts an Eigen matrix or vector to a std::vector.
// Arguments:   - m: Eigen matrix/vector to be converted to a std::vector.
// Returns:     std::vector containing the elements of the Eigen matrix/vector.
//------------------------------------------------------------------------------
template<typename Derived>
std::vector<typename Derived::Scalar> to_std_vector(const MatrixBase<Derived>& m)
{
    Matrix<typename Derived::Scalar, Dynamic, Dynamic> t = m;
    return std::vector<typename Derived::Scalar>(t.data(), t.data() + t.size());
}

//------------------------------------------------------------------------------
// Name:        from_std_vector
// Description: Converts a std::vector to an Eigen vector. The output vector
//              will be a column vector.
// Arguments:   - v: std::vector to be converted to an Eigen vector.
// Returns:     Column Eigen vector containing the elements of the std::vector.
//------------------------------------------------------------------------------
template<typename T>
Eigen::Matrix<T, Dynamic, 1> from_std_vector(std::vector<T> v)
{
    return Map<Matrix<T, Dynamic, 1>>(v.data(), v.size());
}

//------------------------------------------------------------------------------
// Name:        cumsum
// Description: Calculates the cumulative sum of a matrix either along its rows
//              or its columns. Equivalent to Matlab's cumsum function.
// Arguments:   - mat: Matrix to calculate cumulative sum of.
//              - colwise: True to calculate the cumulative sum along cols (top
//                to bottom), false to calculate along rows (left to right).
// Returns:     Cumulative sum along specified axis.
//------------------------------------------------------------------------------
template<typename Matrix>
Matrix cumsum(Matrix mat, bool colwise=true)
{
    if (colwise && mat.rows() > 1)
        for (int i = 1; i < mat.rows(); i++)
            mat.row(i) = mat.row(i) + mat.row(i-1);
    else if (mat.cols() > 1)
        for (int i = 1; i < mat.cols(); i++)
            mat.col(i) = mat.col(i) + mat.col(i-1);
    return mat;
}

//------------------------------------------------------------------------------
// Name:        cummax
// Description: Calculates the cumulative max of a matrix either along its rows
//              or its columns. Equivalent to Matlab's cummax function.
// Arguments:   - mat: Matrix to calculate cumulative max of.
//              - colwise: True to calculate the cumulative max along cols (top
//                to bottom), false to calculate along rows (left to right).
// Returns:     Cumulative max along specified axis.
//------------------------------------------------------------------------------
template<typename Matrix>
Matrix cummax(Matrix mat, bool colwise=true)
{
    int n = mat.rows();
    int m = mat.cols();
    MatrixXd max = mat;
    if (colwise && n > 1)
        for (int i = 1; i < n; i++)
            max.row(i) = mat.block(0,0,i+1,n).colwise().maxCoeff();
    else if (m > 1)
        for (int i = 1; i < m; i++)
            max.col(i) = mat.block(0,0,m,i+1).rowwise().maxCoeff();
    return max;
}

} // namespace avl

#endif // MATRIX_H
