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

#ifndef STATE_SPACE_MODEL_H
#define STATE_SPACE_MODEL_H

// Eigen library
#include <Eigen/Core>
using namespace Eigen;

// Map class
#include <map>

// Util functions
#include <avl_core/util/math.h>

// Typedef for holding min/max limits
typedef std::pair<double, double> limit_t;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class StateSpaceModel
{

public:

    //--------------------------------------------------------------------------
    // Name:        StateSpaceModel constructor
    // Description: Creates a state space model with the given state space
    //              matrices and initial state vector.
    // Arguments:   - A: State space A matrix (n x n).
    //              - B: State space B matrix (n x p).
    //              - C: State space C matrix (q x n).
    //              - D: State space D matrix (q x p).
    //              - x0: State space initial state vector (n x 1).
    //--------------------------------------------------------------------------
    StateSpaceModel(const MatrixXd &A, const MatrixXd &B, const MatrixXd &C,
        const MatrixXd &D, const VectorXd &x0);

    //--------------------------------------------------------------------------
    // Name:        StateSpaceModel constructor
    // Description: Creates an empty state space model.
    //--------------------------------------------------------------------------
    StateSpaceModel();

    //--------------------------------------------------------------------------
    // Name:        StateSpaceModel destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~StateSpaceModel();

    //--------------------------------------------------------------------------
    // Name:        initialize
    // Description: Initializes the state space model with the given state space
    //              vectors and matrices.
    // Arguments:   - A: State space A matrix (n x n).
    //              - B: State space B matrix (n x p).
    //              - C: State space C matrix (q x n).
    //              - D: State space D matrix (q x p).
    //              - x0: State space initial state vector (n x 1).
    //--------------------------------------------------------------------------
    void initialize(const MatrixXd &A, const MatrixXd &B, const MatrixXd &C,
        const MatrixXd &D, const VectorXd &x0);

    //--------------------------------------------------------------------------
    // Name:        add_state_limit
    // Description: Adds a limit to an element of the state vector. The element
    //              of the state vector will be clamped to the limits after
    //              iteration.
    // Arguments:   - i_state: Index of state vector element to clamp.
    //              - min: State vector element minimum value.
    //              - max: State vector element maximum value.
    //--------------------------------------------------------------------------
    void add_state_limit(int i_state, double min, double max);

    //--------------------------------------------------------------------------
    // Name:        add_output_limit
    // Description: Adds a limit to an element of the output vector. The element
    //              of the output vector will be clamped to the limits after
    //              iteration.
    // Arguments:   - i_output: Index of output vector element to clamp.
    //              - min: Output vector element minimum value.
    //              - max: Output vector element maximum value.
    //--------------------------------------------------------------------------
    void add_output_limit(int i_output, double min, double max);

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Iterates the state space model by stepping it forward one
    //              time step using the given input vector u to calculate the
    //              output vector y.
    // Arguments:   - u: Input vector (p x 1).
    // Returns:     The output vector y of the state space model.
    //--------------------------------------------------------------------------
    VectorXd iterate(const VectorXd& u);

    //--------------------------------------------------------------------------
    // Name:        get_state
    // Description: Gets the current state vector of the state space model.
    // Returns:     State space model state vector (n x 1).
    //--------------------------------------------------------------------------
    VectorXd get_state();

    //--------------------------------------------------------------------------
    // Name:        reset
    // Description: Resets the state space model to the initial state x0 that it
    //              was originally assigned.
    //--------------------------------------------------------------------------
    void reset();

private:

    // State space vector sizes
    int n; // Number of states
    int q; // Number of outputs
    int p; // Number of inputs

    // State space matrices
    MatrixXd A; // (n x n)
    MatrixXd B; // (n x p)
    MatrixXd C; // (q x n)
    MatrixXd D; // (q x p)

    // Initial state vector and current state vector
    VectorXd x0; // (n x 1)
    VectorXd x;  // (n x 1)

    // Maps of state and output limits
    std::map<int, limit_t> state_limits;
    std::map<int, limit_t> output_limits;

private:

    //--------------------------------------------------------------------------
    // Name:        validate_model
    // Description: Checks that the state space vectors and matrices are
    //              conformable for multiplication. Throws a std::runtime_error
    //              exception if they are not.
    //--------------------------------------------------------------------------
    void validate_model();

};

#endif // STATE_SPACE_MODEL_H
