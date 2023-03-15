//==============================================================================
// Autonomous Vehicle Library
//
// Description: Inertial navigation algorithm using a manifold UKF to correct
//              inertial navigation estimates with aiding sensor data. Draws
//              heavily from "Principles of GNSS, Inertial and Multisensor
//              Integrated Navigation Systems" by Paul D. Groves.
//==============================================================================

#include <avl_navigation/algorithm/sins_mukf_simple.h>

// MUKF class
#include <avl_navigation/filter/mukf.h>

// Inertial navigation functions
#include <avl_navigation/algorithm/inertial_nav.h>

// Util functions
#include <avl_core/util/geo.h>
#include <avl_core/util/matrix.h>

// Alias for placeholders namespace
namespace ph = std::placeholders;

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        SinsMukfSimple constructor
//------------------------------------------------------------------------------
SinsMukfSimple::SinsMukfSimple()
{

}

//------------------------------------------------------------------------------
// Name:        SinsMukfSimple destructor
//------------------------------------------------------------------------------
SinsMukfSimple::~SinsMukfSimple()
{

}

//------------------------------------------------------------------------------
// Name:        init
// Description: Initializes the filter with an initial state, covariance,
//              and process noise covariance.
// Arguments:   - x0: Initial state vector.
//              - P0: Initial state covariance matrix.
//              - P0: Process noise covariance matrix.
//------------------------------------------------------------------------------
void SinsMukfSimple::init(VectorXd x0, MatrixXd P0, MatrixXd Q)
{

    // Construct the initial state
    NavStateSimple state0;
    state0.C_b_n =  avl::euler_to_matrix<double>(x0.segment(0,3)).transpose();
    state0.v_eb_n = x0.segment(3,3);
    state0.p_b =    x0.segment(6,3);

    // Initialize the MUKF
    mukf.init(state0, P0, Q, phi_func, phi_inv_func);

}

//------------------------------------------------------------------------------
// Name:        iterate
// Description: Iterates the nav algorithm with IMU measurements of
//              angular velocity and specific force with a time step.
// Arguments:   - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame. (3x1)[rad/s]
//              - f_ib_b: Specific force of the body frame w.r.t. the
//                inertial frame. (3x1)[m/s^2]
//              - dt: Time step in seconds.
//------------------------------------------------------------------------------
void SinsMukfSimple::iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt)
{

    // Construct the navigation input
    NavInputSimple input;
    input.dt = dt;
    input.w_ib_b = w_ib_b;
    input.f_ib_b = f_ib_b;

    // Execute the MUKF prediction step
    mukf.predict(f_func, input);

}

//------------------------------------------------------------------------------
// Name:        process_body_velocity
// Description: Processes a body frame velocity measurement.
// Arguments:   - v_eb_b_meas: Body frame velocity measurement. (3x1)[m/s]
//              - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame. (3x1)[rad/s]
//              - R: Measurement noise covariance matrix. (3x3)[(m/s)^2 ]
//              - threshold: Measurement rejection threshold for each
//                measurement element. 0 disables measurement rejection.
//              - l_bS_b: Optional lever arm from the body frame to the
//                sensor frame, expressed in the body frame. (3x1)[m]
// Returns:     Measurement info struct.
//------------------------------------------------------------------------------
MeasInfo SinsMukfSimple::process_body_velocity(Vector3d v_eb_b_meas, Vector3d w_ib_b,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_vel, ph::_1);
    MeasInfo info = mukf.update(v_eb_b_meas, h, R, threshold);
    return info;
}

//------------------------------------------------------------------------------
// Name:        process_depth
// Description: Processes a depth measurement.
// Arguments:   - depth_meas: Depth measurement. (1x1)[m]
//              - alt_surface: Altitude of the surface. [m]
//              - R: Measurement noise covariance matrix. (1x1)[m^2]
//              - threshold: Measurement rejection threshold for each
//                measurement element. 0 disables measurement rejection.
//              - l_bS_b: Optional lever arm from the body frame to the
//                sensor frame, expressed in the body frame. (3x1)[m]
// Returns:     Measurement info struct.
//------------------------------------------------------------------------------
MeasInfo SinsMukfSimple::process_depth(VectorXd depth_meas, double alt_surface,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_depth, ph::_1, alt_surface);
    MeasInfo info = mukf.update(depth_meas, h, R, threshold);
    return info;
}

//------------------------------------------------------------------------------
// Name:        process_range
// Description: Processes a range measurement.
// Arguments:   - range_meas: Range measurement. (1x1)[m]
//              - p_source: Lat, lon, and alt of position that range is
//                measured to. (3x1)[rad,rad,m]
//              - R: Measurement noise covariance matrix. (1x1)[m^2]
//              - threshold: Rejection threshold. Range measurement will be
//                rejected if innovation is greater than this multiple of
//                the position stddev.
//              - threshold: Measurement rejection threshold for each
//                measurement element. 0 disables measurement rejection.
//              - l_bS_b: Optional lever arm from the body frame to the
//                sensor frame, expressed in the body frame. (3x1)[m]
// Returns:     Measurement info struct.
//------------------------------------------------------------------------------
MeasInfo SinsMukfSimple::process_range(VectorXd range_meas, Vector3d p_source,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_range, ph::_1, p_source);
    MeasInfo info = mukf.update(range_meas, h, R, threshold);
    return info;
}

//------------------------------------------------------------------------------
// Name:        process_position
// Description: Processes a position measurement.
// Arguments:   - p_b_meas: Curvilinear position measurement.
//                (3x1)[rad,rad,m]
//              - R: Measurement noise covariance matrix.
//                (3x3)[(rad,rad,m)^2]
//              - threshold: Measurement rejection threshold for each
//                measurement element. 0 disables measurement rejection.
//              - l_bS_b: Optional lever arm from the body frame to the
//                sensor frame, expressed in the body frame. (3x1)[m]
// Returns:     Measurement info struct.
//------------------------------------------------------------------------------
MeasInfo SinsMukfSimple::process_position(Vector3d p_b_meas,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_pos, ph::_1, l_bS_b);
    MeasInfo info = mukf.update(p_b_meas, h, R, threshold);
    return info;
}

//------------------------------------------------------------------------------
// Name:        process_gps
// Description: Processes a GPS measurement with position and velocity.
// Arguments:   - gps_meas: GPS measurement vector with elements:
//                    {lat, lon, alt, vN, vE}
//                and units:
//                    {rad, rad, m, m/s, m/s}.
//              - R: Measurement noise covariance matrix.
//               (5x5)[(rad,rad,m,m/s,m/s)^2]
//              - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame. (3x1)[rad/s]
//              - threshold: Measurement rejection threshold for each
//                measurement element. 0 disables measurement rejection.
//              - l_bS_b: Optional lever arm from the body frame to the
//                sensor frame, expressed in the body frame. (3x1)[m]
// Returns:     Measurement info struct.
//------------------------------------------------------------------------------
MeasInfo SinsMukfSimple::process_gps(VectorXd gps_meas, Vector3d w_ib_b,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_gps, ph::_1);
    MeasInfo info = mukf.update(gps_meas, h, R, threshold);
    return info;
}

//------------------------------------------------------------------------------
// Name:        valid
// Description: Indicates whether the nav state estimate is valid.
// Returns:     True if nav state estimates are valid, false otherwise.
//------------------------------------------------------------------------------
bool SinsMukfSimple::valid()
{
    return true;
}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the most recent state vector (15 x 1).
// Returns:     Most recent state vector (15 x 1).
//------------------------------------------------------------------------------
VectorXd SinsMukfSimple::get_state()
{
    return mukf.get_state().to_vector();
}

//------------------------------------------------------------------------------
// Name:        get_cov
// Description: Gets the diagonal terms of the most recent state covariance
//              matrix.
// Returns:     Diagonal terms of the most recent state covariance
//              matrix.
//------------------------------------------------------------------------------
VectorXd SinsMukfSimple::get_cov()
{
    return mukf.get_cov().diagonal();
}

//------------------------------------------------------------------------------
// Name:        phi_func
// Description: Retraction function for 15 state navigation algorithm.
// Arguments:   - x: ???.
//              - xi: ???.
// Returns:     ???.
//------------------------------------------------------------------------------
NavStateSimple SinsMukfSimple::phi_func(NavStateSimple x, VectorXd xi)
{

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // SE(3) test
    // MatrixXd chi = avl::sek3_exp(xi);
    // x.C_b_n  = x.C_b_n * chi.block(0,0,3,3);
    // x.v_eb_n = x.C_b_n * chi.block(0,3,3,1) + x.v_eb_n;
    // x.p_b    = x.C_b_n * chi.block(0,4,3,1) + x.p_b;
    // std::cout << "====================================" << std::endl;
    // std::cout << "xi" << std::endl;
    // std::cout << xi.transpose() << std::endl;
    // std::cout << "chi" << std::endl;
    // std::cout << chi << std::endl;
    // std::cout << "====================================" << std::endl;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    x.C_b_n  = avl::so3_exp(xi.segment(0,3)) * x.C_b_n;
    x.v_eb_n += xi.segment(3,3);
    x.p_b    += xi.segment(6,3);
    return x;

}

//------------------------------------------------------------------------------
// Name:        phi_inv_func
// Description: Inverse retraction function for 15 state navigation
//              algorithm.
// Arguments:   - x: ???.
//              - x_hat: ???.
// Returns:     ???.
//------------------------------------------------------------------------------
VectorXd SinsMukfSimple::phi_inv_func(NavStateSimple x, NavStateSimple x_hat)
{

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // SE(3) test
    // MatrixXd chi = MatrixXd::Identity(5,5);
    // chi.block(0,0,3,3) = x.C_b_n;
    // chi.block(0,3,3,1) = x.v_eb_n;
    // chi.block(0,4,3,1) = x.p_b;
    //
    // MatrixXd chi_hat = MatrixXd::Identity(5,5);
    // chi.block(0,0,3,3) = x_hat.C_b_n;
    // chi.block(0,3,3,1) = x_hat.v_eb_n;
    // chi.block(0,4,3,1) = x_hat.p_b;
    //
    // VectorXd xi = avl::sek3_log(avl::sek3_inv(chi) * chi_hat);
    // std::cout << "xi" << std::endl;
    // std::cout << xi << std::endl << std::endl;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    VectorXd xi(9);
    xi.segment(0,3)  = avl::so3_log(x_hat.C_b_n * x.C_b_n.transpose());
    xi.segment(3,3)  = x_hat.v_eb_n - x.v_eb_n;
    xi.segment(6,3)  = x_hat.p_b - x.p_b;
    return xi;

}

//------------------------------------------------------------------------------
// Name:        f_func
// Description: Nonlinear state propagation function for 15 state inertial
//              navigation algorithm.
// Arguments:   - state: Current state.
//              - omega: Input.
//              - w: Input noise vector.
// Returns:     New state.
//------------------------------------------------------------------------------
NavStateSimple SinsMukfSimple::f_func(NavStateSimple x, NavInputSimple omega, VectorXd w)
{

    // Noise components
    Vector3d w_g = w.segment(0,3);
    Vector3d w_a = w.segment(3,3);

    // Add noise to IMU measurements
    omega.w_ib_b += w_g;
    omega.f_ib_b += w_a;

    // Propagate nav states
    f_ins_simple(x.C_b_n, x.v_eb_n, x.p_b, omega.w_ib_b, omega.f_ib_b, omega.dt);
    return x;

}

//------------------------------------------------------------------------------
// Name:        h_func_pos
// Description: Nonlinear measurement equation for a measurement of
//              curvilinear position with lever arm correction.
// Arguments:   - x: Current state.
//              - l_bS_b: Lever arm from the body frame to the sensor frame,
//                expressed in the body frame in meters.
// Returns:     Position measurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd SinsMukfSimple::h_func_pos(NavStateSimple x, Vector3d l_bS_b)
{
    return x.p_b;
}

//------------------------------------------------------------------------------
// Name:        h_func_vel
// Description: Nonlinear measurement function for a measurement of linear
//              velocity in the body frame with lever arm correction.
// Arguments:   - x: Current state.
//              - l_bS_b: Lever arm from the body frame to the sensor frame,
//                expressed in the body frame in meters.
//              - w_ib_b: Angular velocity of the body frame relative to the
//                inertial frame, expressed in the body frame as measured by
//                an IMU in rad/s.
// Returns:     Body frame velocity measurement corresponding to current
//              state.
//------------------------------------------------------------------------------
VectorXd SinsMukfSimple::h_func_vel(NavStateSimple x)
{
    return x.C_b_n.transpose()*x.v_eb_n;
}

//------------------------------------------------------------------------------
// Name:        h_func_depth
// Description: Nonlinear measurement function for a measurement of
//              depth.
// Arguments:   - x: Current state.
//              - alt_surface: Altitude of the surface in meters.
//              - l_bS_b: Lever arm from the body frame to the sensor frame,
//                expressed in the body frame in meters.
// Returns:     Depth measurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd SinsMukfSimple::h_func_depth(NavStateSimple x, double alt_surface)
{
    VectorXd y(1);
    y << alt_surface - x.p_b(2);
    return y;
}

//------------------------------------------------------------------------------
// Name:        h_func_range
// Description: Nonlinear measurement function for a measurement of range
//              from the sensor to a range measurement source location.
// Arguments:   - x: Current state.
//              - l_bS_b: Lever arm from the body frame to the sensor frame,
//                expressed in the body frame in meters.
// Returns:     Range measurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd SinsMukfSimple::h_func_range(NavStateSimple x, Vector3d p_source)
{

    // Check for NaN altitude. If one of them is NaN, do a 2D range calculation
    if (std::isnan(x.p_b(2)) || std::isnan(p_source(2)))
    {
        p_source(2) = 0.0;
        x.p_b(2) = 0.0;
    }

    // Construct the measurement vector with the range between the source
    // position and the sensor frame
    VectorXd y(1);
    y << linear_dist(x.p_b, p_source);
    return y;

}

//------------------------------------------------------------------------------
// Name:        h_func_gps
// Description: Nonlinear measurement equation for a GPS measurement with
//              curvilinear position and NED velocity. Includes a lever arm
//              correction.
// Arguments:   - x: Current state.
//              - l_bS_b: Lever arm from the body frame to the sensor frame,
//                expressed in the body frame in meters.
//              - w_ib_b: Angular velocity of the body frame relative to the
//                inertial frame, expressed in the body frame as measured by
//                an IMU in rad/s.
// Returns:     Position measurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd SinsMukfSimple::h_func_gps(NavStateSimple x)
{
    VectorXd y(5);
    y << x.p_b, x.v_eb_n.segment(0,2);
    return y;
}
