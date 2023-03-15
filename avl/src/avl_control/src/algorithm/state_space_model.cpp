//==============================================================================
// Autonomous Vehicle Library
//
// Description: A class representing a state space model consisting of the
//              following state space vectors and matrices:
//
//              x  (n x 1): State vector
//              x0 (n x 1): Initial state vector
//              y  (q x 1): Output vector
//              u  (p x 1): Input vector
//              A  (n x n): State matrix
//              B  (n x p): Input matrix
//              C  (q x n): Output matrix
//              D  (q x p): Feedthrough matrix
//
//          The state space model can be propagated forward in time by calling
//          the iterate function with an input matrix. This function will update
//          state state vector and return the output vector for the iteration.
//==============================================================================

#include <avl_control/algorithm/state_space_model.h>

//------------------------------------------------------------------------------
// Name:        StateSpaceModel constructor
// Description: Creates a state space model with the given state space
//              matrices and initial state vector.
// Arguments:   - A: State space A matrix (n x n).
//              - B: State space B matrix (n x p).
//              - C: State space C matrix (q x n).
//              - D: State space D matrix (q x p).
//              - x0: State space initial state vector (n x 1).
//------------------------------------------------------------------------------
StateSpaceModel::StateSpaceModel(const MatrixXd &A, const MatrixXd &B,
    const MatrixXd &C, const MatrixXd &D, const VectorXd &x0_vect)
{
    initialize(A, B, C, D, x0);
}

//------------------------------------------------------------------------------
// Name:        StateSpaceModel constructor
// Description: Creates an empty state space model.
//--------------------------------------------------------------------------
StateSpaceModel::StateSpaceModel()
{

}

//------------------------------------------------------------------------------
// Name:        StateSpaceModel destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
StateSpaceModel::~StateSpaceModel()
{

}

//------------------------------------------------------------------------------
// Name:        initialize
// Description: Initializes the state space model with the given state space
//              vectors and matrices.
// Arguments:   - A: State space A matrix (n x n).
//              - B: State space B matrix (n x p).
//              - C: State space C matrix (q x n).
//              - D: State space D matrix (q x p).
//              - x0: State space initial state vector (n x 1).
//------------------------------------------------------------------------------
void StateSpaceModel::initialize(const MatrixXd &A, const MatrixXd &B,
    const MatrixXd &C, const MatrixXd &D, const VectorXd &x0)
{

    // Set the local variables from the constructor
    this->A = A;
    this->B = B;
    this->C = C;
    this->D = D;
    this->x0 = x0;

    // Set the state space vector sizes
    n = x0.size();
    q = D.rows();
    p = B.cols();

    // Set the current state to the initial state
    reset();

    // Check that the state space vectors and matrices are the proper sizes
    validate_model();

}

//------------------------------------------------------------------------------
// Name:        add_state_limit
// Description: Adds a limit to an element of the state vector. The element
//              of the state vector will be clamped to the limits after
//              iteration.
// Arguments:   - i_state: Index of state vector element to clamp.
//              - min: State vector element minimum value.
//              - max: State vector element maximum value.
//------------------------------------------------------------------------------
void StateSpaceModel::add_state_limit(int i_state, double min, double max)
{
    if (i_state < 0 || !(i_state < n))
        throw std::runtime_error("add_state_limit: invalid index");
    state_limits[i_state] = {min, max};
}

//------------------------------------------------------------------------------
// Name:        add_output_limit
// Description: Adds a limit to an element of the output vector. The element
//              of the output vector will be clamped to the limits after
//              iteration.
// Arguments:   - i_output: Index of output vector element to clamp.
//              - min: Output vector element minimum value.
//              - max: Output vector element maximum value.
//------------------------------------------------------------------------------
void StateSpaceModel::add_output_limit(int i_output, double min, double max)
{
    if (i_output < 0 || !(i_output < q))
        throw std::runtime_error("add_output_limit: invalid index");
    output_limits[i_output] = {min, max};
}

//------------------------------------------------------------------------------
// Name:        iterate
// Description: Iterates the state space model by stepping it forward one
//              time step using the given input vector u to calculate the
//              output vector y.
// Arguments:   - u: Input vector (p x 1).
// Returns:     The output vector y of the state space model.
//------------------------------------------------------------------------------
VectorXd StateSpaceModel::iterate(const VectorXd &u)
{

    // Check that the input vector has the expected size
    if (u.size() != p)
        throw std::runtime_error("the input vector u must have dimmensions "
            "(p x 1)");

    // Iterate the state space model with the standard state space equations
    // x(k+1) = A*x(k) + B*u(k)
    // y(k)   = C*x(k) + D*u(k)

    // Iterate the state
    x = A*x + B*u;

    // Clamp the state according to the limits
    for (const auto& elem : state_limits)
    {
        int i = elem.first;
        limit_t limit = elem.second;
        x(i) = avl::clamp(x(i), limit.first, limit.second);
    }

    // Calculate the output
    VectorXd y = C*x + D*u;

    // Clamp the output according to the limits
    for (const auto& elem : output_limits)
    {
        int i = elem.first;
        limit_t limit = elem.second;
        y(i) = avl::clamp(y(i), limit.first, limit.second);
    }

    // Return the output vector
    return y;

}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the current state vector of the state space model.
// Returns:     State space model state vector (n x 1).
//------------------------------------------------------------------------------
VectorXd StateSpaceModel::get_state()
{
    return x;
}

//------------------------------------------------------------------------------
// Name:        reset
// Description: Resets the state space model to the initial state x0 that it
//              was originally assigned.
//------------------------------------------------------------------------------
void StateSpaceModel::reset()
{
    x = x0;
}

//------------------------------------------------------------------------------
// Name:        validate_model
// Description: Checks that the state space vectors and matrices are
//              conformable for multiplication. Throws a std::runtime_error
//              exception if they are not.
//------------------------------------------------------------------------------
void StateSpaceModel::validate_model()
{

    if (A.rows() != n || A.cols() != n)
        throw std::runtime_error("the matrix A must have dimmensions (n x n)");

    if (B.rows() != n || B.cols() != p)
        throw std::runtime_error("the matrix B must have dimmensions (n x p)");

    if (C.rows() != q || C.cols() != n)
        throw std::runtime_error("the matrix C must have dimmensions (q x n)");

    if (D.rows() != q || D.cols() != p)
        throw std::runtime_error("the matrix D must have dimmensions (q x p)");

}
