//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for Eigen matrix manipulation.
//==============================================================================

#include "util/matrix.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        round_all
// Description: Rounds all terms in a matrix to a given number of decimals.
// Arguments:   - m: Matrix to round.
//              - decimals: Number of decimals to round to.
// Returns:     Matrtix with all terms rounded.
//------------------------------------------------------------------------------
MatrixXd round_all(MatrixXd m, unsigned char decimals)
{
    for (int i = 0; i < m.rows(); i++)
        for (int j = 0; j < m.cols(); j++)
            m(i,j) = round_decimals(m(i,j), decimals);
    return m;
}

//------------------------------------------------------------------------------
// Name:        linspace
// Description: Creates a linearly spaced vector of values between min and max
//              with n elements.
// Arguments:   - min: Minimum value.
//              - max: Maximum value.
//              - n: Number of elements.
// Returns :    Linearly spaced vector of n values.
//------------------------------------------------------------------------------
VectorXd linspace(double min, double max, int n)
{

    VectorXd vec(n);
    double step = (max - min) / (n - 1);
    for (int i = 0; i < vec.size(); i++)
        vec(i) = min + step*i;
    return vec;

}

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
VectorXi logspace(int start, int end, int n_max)
{

    // Create a log-spaced vector of powers of 10
    VectorXd pows = linspace(log10(start), log10(end), n_max);

    // Create the vector of log-spaced values, skipping duplicates
    std::vector<int> bins = {1};
    for (int i = 1; i < pows.size(); i++)
    {
        int bin = floor(pow(10, pows(i)));
        if (bin != bins.at(bins.size()-1))
            bins.push_back(bin);
    }

    // Turn the std::vector into an Eigen vector
    return Map<VectorXi>(bins.data(), bins.size());

}

//------------------------------------------------------------------------------
// Name:        randn
// Description: Generates a matrix of Gaussian noise with zero mean and a
//              specified covariance.
// Arguments:   - rows: Number of rows in output matrix.
//              - cols: Number of cols in output matrix.
//              - cov: Noise covariance.
// Returns:     Matrix of Gaussian noise.
//------------------------------------------------------------------------------
MatrixXd randn(int rows, int cols, double cov)
{

    // Create the random number generator and normal distribution
    auto time = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(time);
    std::normal_distribution<double> dist(0.0, sqrt(cov));

    // Create the output matrix
    MatrixXd mat = MatrixXd::Zero(rows, cols);

    // Generate the random number for each element
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
            mat(i,j) = dist(gen);

    return mat;

}

//------------------------------------------------------------------------------
// Name:        randn
// Description: Generates a matrix of Gaussian noise with zero mean and a
//              specified covariance.
// Arguments:   - cols: Number of cols in output matrix (n).
//              - cov: Noise covariance matrix (r x r).
// Returns:     Matrix of Gaussian noise (r x n).
//------------------------------------------------------------------------------
MatrixXd randn(int cols, MatrixXd cov)
{

    // Check for square covariance matrix
    if (cov.rows() != cov.cols())
        throw std::runtime_error("covariance matrix must be square");

    int rows = cov.rows();

    // Create the random number generator and normal distribution
    auto time = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(time);
    std::normal_distribution<double> dist(0.0, 1.0);

    // Create the output matrix
    MatrixXd mat = MatrixXd::Zero(rows, cols);

    // Generate the random number for each element
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            mat(i,j) = dist(gen);

    // Apply covariance
    MatrixXd stddev = cov.cwiseSqrt();
    mat = stddev*mat;

    return mat;

}

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
MatrixXd rand(int rows, int cols, double min, double max)
{

    // Create the random number generator and normal distribution
    auto time = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(time);
    std::uniform_real_distribution<double> dist(min, max);

    // Create the output matrix
    MatrixXd mat = MatrixXd::Zero(rows, cols);

    // Generate the random number for each element
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
            mat(i,j) = dist(gen);

    return mat;

}

//------------------------------------------------------------------------------
// Name:        svd
// Description: Performs singular value decomposition on a matrix.
// Arguments:   - A: Matrix to get singular values of.
// Returns :    Singular values of A.
//------------------------------------------------------------------------------
VectorXd svd(const MatrixXd A)
{
    JacobiSVD<MatrixXd> calc(A, ComputeThinU | ComputeThinV);
    return calc.singularValues();
}

//------------------------------------------------------------------------------
// Name:        matrix_l2_norm
// Description: Gets the L2 norm of a matrix, which its largest singular value.
// Arguments:   - A: Matrix to get L2 norm of.
// Returns :    Matrix L2 norm of A.
//------------------------------------------------------------------------------
double matrix_l2_norm(MatrixXd A)
{
    return svd(A).maxCoeff();
}

//------------------------------------------------------------------------------
// Name:        so2_exp
// Description: Exponential map on SO(2).
// Arguments:   - phi: Scalar to get SO(2) exponential mapping of.
// Returns:     Matrix of exponential mapping of phi to SO(2).
//------------------------------------------------------------------------------
Matrix2d so2_exp(double phi)
{
    Matrix2d R;
    R(0,0) =  cos(phi);
    R(1,0) =  sin(phi);
    R(0,1) = -sin(phi);
    R(1,1) =  cos(phi);
    return R;
}

//------------------------------------------------------------------------------
// Name:        so2_log
// Description: Log map on SO(2).
// Arguments:   - C: Matrix in SO(2) to get log mapping of.
// Returns:     Scalar of log mapping of C.
//------------------------------------------------------------------------------
double so2_log(Matrix2d C)
{
    return atan2(C(1, 0), C(0, 0));
}

//------------------------------------------------------------------------------
// Name:        so3_exp
// Description: Exponential map on SO(3).
// Arguments:   - phi: Vector to get SO(3) exponential mapping of.
// Returns:     Matrix of exponential mapping of phi to SO(3).
//------------------------------------------------------------------------------
Matrix3d so3_exp(Vector3d phi)
{

    double TOL = 1.0e-9;
    Matrix3d I = Matrix3d::Identity();
    double theta = phi.norm();
    Vector3d a = phi / theta;

    Matrix3d R;
    if (theta < TOL)
        R = I + skew(phi);
    else
        R = cos(theta) * I + (1.0-cos(theta))*(a*a.transpose()) +
            sin(theta) * skew(a);

    return R;

}

//------------------------------------------------------------------------------
// Name:        so3_log
// Description: Log map on SO(3).
// Arguments:   - C: Matrix in SO(3) to get log mapping of.
// Returns:     Vector of log mapping of C.
//------------------------------------------------------------------------------
Vector3d so3_log(Matrix3d C)
{

    double TOL = 1.0e-9;
    Matrix3d I = Matrix3d::Identity();

    double cos_phi = 0.5 * (C.trace() - 1.0);
    cos_phi = std::min(std::max(cos_phi, -1.0), 1.0);
    double phi = acos(cos_phi);

    Vector3d J_inv;
    if (phi < TOL)
        J_inv = inv_skew(I - C);
    else
        J_inv = inv_skew(0.5*phi/sin(phi) * (C - C.transpose()));
    return J_inv;

}

//------------------------------------------------------------------------------
// Name:        so3_left_jacobian
// Description: Left Jacobian of SO(3).
// Arguments:   - phi: Vector to get left Jacobian of.
// Returns:     Matrix of left Jacobian.
//------------------------------------------------------------------------------
Matrix3d so3_left_jacobian(Vector3d phi)
{

    double TOL = 1.0e-9;
    Matrix3d I = Matrix3d::Identity();
    double theta = phi.norm();

    Matrix3d J;
    if (theta < TOL)
        J = I + 0.5 * skew(phi);
    else
        J = I + (1.0 - cos(theta)) / (theta*theta) * skew(phi) +
        (theta - sin(theta)) / (theta*theta*theta) * skew(phi)*skew(phi);
    return J;

}

//------------------------------------------------------------------------------
// Name:        so3_inv_left_jacobian
// Description: Inverse left Jacobian of SO(3).
// Arguments:   - phi: Vector to get inverse left Jacobian of.
// Returns:     Matrix of inverse left Jacobian.
//------------------------------------------------------------------------------
Matrix3d so3_inv_left_jacobian(Vector3d phi)
{

    double TOL = 1.0e-9;
    Matrix3d I = Matrix3d::Identity();
    double theta = phi.norm();

    Matrix3d J_inv;
    if (theta < TOL)
        J_inv = I - 0.5 * skew(phi);
    else
        J_inv = I - 0.5*skew(phi) + (1.0/(theta*theta) - (1.0 + cos(theta)) /
            (2.0 * theta * sin(theta))) * skew(phi)*skew(phi);
    return J_inv;

}

//------------------------------------------------------------------------------
// Name:        sek3_inv
// Description: Inverse function for SEk(3).
// Arguments:   - chi: ???.
// Returns:     Inverse of input for SEk(3).
//------------------------------------------------------------------------------
MatrixXd sek3_inv(MatrixXd T)
{

    int k = T.cols() - 3;
    MatrixXd T_inv = MatrixXd::Zero(3+k, 3+k);
    Matrix3d R = T.block(0,0,3,3);
    T_inv.block(0,0,3,3) = R.transpose();
    T_inv.block(0,3,3,k) = -R.transpose() * T.block(0,3,3,k);
    T_inv.block(3,3,k,k) = MatrixXd::Identity(k,k);

    return T_inv;

}

//------------------------------------------------------------------------------
// Name:        sek3_exp
// Description: Exponential function for SEk(3).
// Arguments:   - xi: ???.
// Returns:     Matrix of exponential mapping of xi to SEk(3).
//------------------------------------------------------------------------------
MatrixXd sek3_exp(VectorXd xi)
{

    int k = xi.size() / 3 - 1;
    Vector3d phi = xi.segment(0,3);
    Matrix3d R = so3_exp(phi);
    Matrix3d Jl = so3_left_jacobian(phi);

    MatrixXd T = MatrixXd::Identity(k+3,k+3);
    T.block(0,0,3,3) = R;
    for (int i = 0; i < k; i++)
        T.block(0,3+i,3,1) = Jl*xi.segment(3+i*3,3);
    return T;

}

//------------------------------------------------------------------------------
// Name:        sek3_inv
// Description: Log function for SEk(3).
// Arguments:   - chi: ???.
// Returns:     Log of input for SEk(3).
//------------------------------------------------------------------------------
VectorXd sek3_log(MatrixXd chi)
{

    int k = chi.cols() - 3;
    Matrix3d R = chi.block(0,0,3,3);
    Vector3d phi = so3_log(R);
    Matrix3d Jl_inv = so3_inv_left_jacobian(phi);

    VectorXd xi = VectorXd::Zero(3*k+3);
    xi.segment(0,3) = phi;
    for (int i = 0; i < k; i++)
        xi.segment(3+i*3,3) = Jl_inv*chi.block(0,3+i,3,1);

    return xi;

}

//------------------------------------------------------------------------------
// Name:        sek3_left_jacobian
// Description: Left Jacobian of SEk(3).
// Arguments:   - phi: Vector to get left Jacobian of.
// Returns:     Matrix of left Jacobian.
//------------------------------------------------------------------------------
MatrixXd sek3_left_jacobian(VectorXd phi)
{

    int k = phi.size() / 3 - 1;
    Matrix3d Jl = so3_left_jacobian(phi.segment(0,3));

    MatrixXd J = MatrixXd::Zero(3+3*k, 3+3*k);
    J.block(0,0,3,3) = Jl;
    for (int i = 0; i < k; i++)
        J.block(3+3*i,3+3*i,3,3) = Jl;
    return J;

}

//------------------------------------------------------------------------------
// Name:        sek3_inv_left_jacobian
// Description: Inverse left Jacobian of SEk(3).
// Arguments:   - phi: Vector to get inverse left Jacobian of.
// Returns:     Matrix of inverse left Jacobian.
//------------------------------------------------------------------------------
MatrixXd sek3_inv_left_jacobian(VectorXd phi)
{

    int k = phi.size() / 3 - 1;
    Matrix3d Jl_inv = so3_inv_left_jacobian(phi.segment(0,3));

    MatrixXd J_inv = MatrixXd::Zero(3+3*k, 3+3*k);
    J_inv.block(0,0,3,3) = Jl_inv;
    J_inv.block(0,3,3,3) = Jl_inv*phi.segment(3,3);
    for (int i = 0; i < k; i++)
        J_inv.block(3+3*i,3+3*i,3,3) = Jl_inv;
    return J_inv;

}

//------------------------------------------------------------------------------
// Name:        to_string
// Description: Converts a vector of doubles to a string listing the elements of
//              the vector.
// Arguments:   - v: Vector to be converted to a string.
// Returns:     String listing the elements of the vector.
//------------------------------------------------------------------------------
std::string to_string(VectorXd v, int precision, std::string delim)
{
    std::stringstream ss;
    ss << std::setprecision(precision) << std::scientific;
    for (int i = 0; i < v.size(); i++)
        ss << ((i > 0) ? " " : "") << v(i);
    return ss.str();
}

//------------------------------------------------------------------------------
// Name:        csv_to_matrix
// Description: Converts CSV file containing matrix elements to an Eigen matrix.
//              The CSV file must have a row of matrix data on each line, where
//              each value is separated by a delimiter.
// Arguments:   - file_path: Path to the CSV file
//              - delim: matrix element delimiter, default comma
// Returns:     CSV file data as an Eigen matrix.
//------------------------------------------------------------------------------
MatrixXd csv_to_matrix(const std::string& file_path,
    const char& delim, const size_t& start_row)
{

    // Open the input file
    std::ifstream file_stream;
    file_stream.open(file_path, std::ifstream::in);
    if (!file_stream.is_open())
        throw std::runtime_error("csv_to_matrix: failed to open file ("
            + file_path + ")");

    std::string line;
    std::vector<double> elements;
    size_t rows = 0;
    size_t cols = 0;

    // Call getline on the rows before the start row to move past them
    for (size_t i = 0; i < start_row; i++)
        std::getline(file_stream, line);

    // Loop through all lines (rows) in the file
    bool counted_cols = false;
    while (std::getline(file_stream, line))
    {

        rows++;

        // Read the row values
        std::vector<double> row = avl::vec_from_string(line, delim);

        // Use the first row of data to get the number of columns. For rows
        // after the first, check that they match the number of columns
        if (!counted_cols)
            cols = row.size();
        else if (row.size() != cols)
            throw std::runtime_error("csv_to_matrix: missing data in row " +
                std::to_string(rows + start_row));

        avl::append(elements, row);
        counted_cols = true;

    }

    // Close the file stream
    file_stream.close();

    return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(elements.data(),
        rows, cols);

}

//------------------------------------------------------------------------------
// Name:        matrix_to_csv
// Description: Writes an Eigen matrix to a CSV file containing matrix elements.
// Arguments:   - file_path: Path to save the CSV file to.
//              - m: Matrix to write to CSV file.
//              - mode: in, out, append... Default: std::ifstream::out
//------------------------------------------------------------------------------
void matrix_to_csv(const std::string& file_path, const MatrixXd& m, bool truncate)
{
    // Select truncate or append mode
    std::ios_base::openmode mode = (truncate)? std::ifstream::trunc : std::ifstream::app;

    // Open the output file
    std::ofstream file;
    file.open(file_path, mode);
    if (!file.is_open())
        throw std::runtime_error("matrix_to_csv: failed to open file ("
            + file_path + ")");

    // Change the format to match the matrix read format
    IOFormat csv_format(FullPrecision, DontAlignCols, " ", "\n", "", "", "", "");

    // Write the matrix to the file
    file << m.format(csv_format) << std::endl;

    // Close the file stream
    file.close();

}

//------------------------------------------------------------------------------
// Name:        allan_variance
// Description: Calculates the overlapping Allan variance down the columns of
//              the input matrix of data. In other words, each column should
//              contain the data for one sensor. Reference:
//    https://telesens.co/wp-content/uploads/2017/05/AllanVariance5087-1.pdf
// Arguments:   - theta: Matrix of integrated sensor measurements.
//              - M: Vector of cluster sizes to calculate Allan variance at.
//              - fs: Data sampling frequency in Hz.
// Returns:     Cumulative sum along specified axis.
//------------------------------------------------------------------------------
MatrixXd allan_variance(const MatrixXd& theta, const VectorXi& M,
    double fs)
{

    int N = theta.rows();

    // Calculate the time period for each cluster
    VectorXd tau = M.cast<double>() / fs;

    // Loop through every cluster size
    MatrixXd avar = MatrixXd::Zero(M.size(), 6);
    for (int i = 0; i < M.size(); i++)
    {

        int m = M(i);

        // Calculate the summation in Equation 3
        for (int k = 0; k < (N-2*m); k++)
        {
            VectorXd term = theta.row(k+2*m) - 2*theta.row(k+m) + theta.row(k);
            term = term.array().square();
            avar.row(i) += term;
        }

        // Multiply by the outer term
        avar.row(i) *= 1 / (2*tau(i)*tau(i)*(N-2*m));

    }

    return avar;

}

//------------------------------------------------------------------------------
// Name:        autocorrelation
// Description: Calculates the autocorrelation of the input vector of data.
// Arguments:   - x: Vector of data to calculate autocorrelation of
// Returns:     Vector of single sided autocorrelation values.
//------------------------------------------------------------------------------
VectorXd autocorrelation(const VectorXd& x, const VectorXi& M)
{

        int N = x.size();
        double x_bar = x.mean();
        VectorXd autocorr(M.size());

        for (int t = 0; t < M.size(); t++)
        {

            float num = 0; // Numerator
            float den = 0; // Denominator

            for (int i = 0; i < N; i++)
            {
                double xim = x(i) - x_bar;
                int idx = (i + M(t)) % N;
                num += xim * (x(idx) - x_bar);
                den += xim * xim;
            }

            autocorr(t) = num / den;

        }

        return autocorr;

}

//------------------------------------------------------------------------------
// Name:        euler_cov_to_q_cov
// Description: Converts an Euler angle covariancematrix to a quaternion
//              covariance matrix.
// Arguments:   - euler: Vector containing Euler angles in radians.
//              - euler_cov: Matrix containing Euler angle covariance in
//                radians^2.
// Returns :    Quaternion covariance matrix.
//------------------------------------------------------------------------------
Matrix4d euler_cov_to_q_cov(const Vector3d& euler, const Matrix3d& euler_cov)
{

    MatrixXd J(4, 3);

    double roll = euler(0);
    double pitch = euler(0);
    double yaw = euler(0);

    double cy = cos(yaw*0.5);
    double sy = sin(yaw*0.5);
    double cp = cos(pitch*0.5);
    double sp = sin(pitch*0.5);
    double cr = cos(roll*0.5);
    double sr = sin(roll*0.5);

    double ccc = cr*cp*cy;
    double ccs = cr*cp*sy;
    double csc = cr*sp*cy;
    double css = cr*sp*sy;
    double scc = sr*cp*cy;
    double scs = sr*cp*sy;
    double ssc = sr*sp*cy;
    double sss = sr*sp*sy;

    J << 0.5*( ssc -ccs), 0.5*( scs -csc), 0.5*( css -scc),
         0.5*(-csc -scs), 0.5*(-ssc -ccs), 0.5*( ccc +sss),
         0.5*( scc -css), 0.5*( ccc -sss), 0.5*( ccs -ssc),
         0.5*( ccc +sss), 0.5*(-css -scc), 0.5*(-csc -scs);

    return J * euler_cov * J.transpose();

}

//------------------------------------------------------------------------------
// Name:        orthonormalize
// Description: Orthonormalizes a rotation matrix (Equations 5.78 to 5.80).
// Arguments:   - C: Rotation matrix to orthonormalize.
// Returns :    Orthonormalized rotation matrix.
//------------------------------------------------------------------------------
Matrix3d orthonormalize(Matrix3d C)
{

    // Extract the rows (Equation 5.78)
    Vector3d c1 = C.row(0);
    Vector3d c2 = C.row(1);
    Vector3d c3 = C.row(2);

    // Calculate delta for each row (under Equation 5.78)
    double D12 = c1.transpose() * c2;
    double D23 = c2.transpose() * c3;
    double D13 = c1.transpose() * c3;

    // Apply the correction (Equation 5.79)
    Vector3d c1p = c1 - 0.5*D12*c2 - 0.5*D13*c3;
    Vector3d c2p = c2 - 0.5*D12*c1 - 0.5*D23*c3;;
    Vector3d c3p = c3 - 0.5*D13*c1 - 0.5*D23*c2;;

    // Apply normalization (Equation 5.80)
    c1p = 2.0 / (1.0 + c1p.transpose()*c1p) * c1p;
    c2p = 2.0 / (1.0 + c2p.transpose()*c2p) * c2p;
    c3p = 2.0 / (1.0 + c3p.transpose()*c3p) * c3p;

    // Reconstruct the matrix
    Matrix3d C_orthonormalized;
    C_orthonormalized.row(0) = c1p;
    C_orthonormalized.row(1) = c2p;
    C_orthonormalized.row(2) = c3p;

    return C_orthonormalized;

}

//------------------------------------------------------------------------------
// Name:        hcat
// Description: Horizontally concatenates two matrices.
// Arguments:   - A: First matrix to concatenate.
//              - B: Second matrix to concatenate.
// Returns :    Concatenated matrix.
//------------------------------------------------------------------------------
MatrixXd hcat(const MatrixXd& A, const MatrixXd& B)
{

    if (A.rows() != B.rows())
        throw std::runtime_error("hcat: matrices must have the same "
            "number of rows");

    MatrixXd out(A.rows(), A.cols() + B.cols());
    out << A, B;

    return out;

}

//------------------------------------------------------------------------------
// Name:        vcat
// Description: Vertically concatenates two matrices.
// Arguments:   - A: First matrix to concatenate.
//              - B: Second matrix to concatenate.
// Returns :    Concatenated matrix.
//------------------------------------------------------------------------------
MatrixXd vcat(const MatrixXd& A, const MatrixXd& B)
{

    if (A.cols() != B.cols())
        throw std::runtime_error("vcat: matrices must have the same "
            "number of cols");

    MatrixXd out(A.rows() + B.rows(), A.cols());
    out << A, B;

    return out;

}

}
