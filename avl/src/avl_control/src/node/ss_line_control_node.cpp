//==============================================================================
// Autonomous Vehicle Library
//
// Description: Control node that commands attempts to follow a line defied
//              by a start point and an end point. The defined line acts as an
//              infinite line with direction defined by the direction from the
//              start point to the end point. The vehicle position will
//              converge from its current position to the closest point on line
//              the infinite line by controlling vehicle yaw. It will continue
//              follow the line indefinitely until a new line is speciified or
//              line control is disabled. The yaw setpoint output is
//              controlled by a PID attempting to maintain zero cross-track
//              error between the vehicle's position and the line. The PID is
//              implemented as a state space model in order to allow for
//              nonlinearities in the error input.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/yaw (avl_msgs/Float64SetpointMsg)
//
// Subscribers: nav/inertial_nav (avl_msgs/NavigationMsg)
//              setpoint/line (avl_msgs/LineSetpointMsg)
//==============================================================================

// Control node base class
#include <avl_control/control_node.h>

// State space controller class
#include <avl_control/algorithm/state_space_model.h>

// Util functions
#include <avl_core/util/geo.h>

// ROS messages
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/LineSetpointMsg.h>
using namespace avl_msgs;

// Typedef for a vector of doubles
typedef std::vector<double> doubles_t;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class StateSpaceLineControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        StateSpaceLineControlNode constructor
    //--------------------------------------------------------------------------
    StateSpaceLineControlNode(int argc, char **argv) : ControlNode(argc, argv)
    {

    }

private:

    // State space PID controller instance
    StateSpaceModel ss_model;

    // Publisher for output messages
    ros::Publisher output_pub;

    double lookahead_dist;

private:

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on each controller iteration. Controller iteration
    //              logic and output publishing should be implemented here.
    //--------------------------------------------------------------------------
    void iterate()
    {

        // Get the input and setpoint
        NavigationMsg input = get_input<NavigationMsg>("nav/inertial_nav");
        LineSetpointMsg setpoint = get_setpoint<LineSetpointMsg>("setpoint/line");
        double lat_start = setpoint.lat_start;
        double lon_start = setpoint.lon_start;
        double lat_end = setpoint.lat_end;
        double lon_end = setpoint.lon_end;
        double lat_veh = input.lat;
        double lon_veh = input.lon;

        // Determine the yaw between the start point and the end point
        double line_yaw = avl::wrap_to_2pi(
            avl::initial_bearing(lat_start, lon_start,
                                 lat_end,   lon_end));

        // Find the cross track error in meters
        double cross_track_err = avl::cross_track_error(
            lat_start, lon_start,
            lat_end,   lon_end,
            lat_veh,   lon_veh);

        // We want to drive the cross track error to 0, so we take the error
        // of the cross track error (this is the same as multiplying by -1)
        double xtk_error_error = 0.0 - cross_track_err;

        // Take the atan of the cross track error and feed that in to the
        // proportional term of the controller.
        double atan_xtk_error = std::atan2(xtk_error_error, lookahead_dist);

        // Make the input for the controller
        Eigen::VectorXd u(2);
        u(0) = xtk_error_error;
        u(1) = atan_xtk_error;

        // Get the controller output
        Eigen::VectorXd ss_output = ss_model.iterate(u);

        // Calculate the yaw setpoint to publish. Ideally the vehicle will
        // maintain a yaw parallel to the line. However, if the vehicle has
        // cross track error to the line, it should be pointed more toward
        // the line
        double yaw_setpoint = line_yaw + ss_output(0);

        // Create the yaw setpoint message and publish it
        Float64SetpointMsg output_msg;
        output_msg.enable = true;
        output_msg.data = yaw_setpoint;
        output_pub.publish(output_msg);

        log_data("[line] %.8f %.8f %.8f %.8f %.8f %.8f",
            lat_start, lon_start, lat_end, lon_end, lat_veh, lon_veh);

        log_data("[ss] %.2f %.2f %.4f %.4f",
            cross_track_err, 0.0, line_yaw, yaw_setpoint);

        log_data("[y] %.4f", ss_output(0));

        log_data("[u] %.4f %.4f", u(0), u(1));

        Eigen::VectorXd x = ss_model.get_state();
        log_data("[x] %.4f", x(0));

    }

    //--------------------------------------------------------------------------
    // Name:        disable
    // Description: Called when all setpoints in the control node are disabled.
    //              Logic on how to disable the controller, such as sending
    //              disabling outputs to any cascaded controllers, should be
    //              implemented here.
    //--------------------------------------------------------------------------
    void disable()
    {
        Float64SetpointMsg output_msg;
        output_msg.enable = false;
        output_msg.data = 0.0;
        output_pub.publish(output_msg);
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[line] lat_{start} lon_{start} lat_{end} lon_{end} lat_{vehicle} lon_{vehicle}");
        add_data_header("[line] rad rad rad rad rad rad");
        add_data_header("[ss] cross\\_track\\_err\\_input cross\\_track\\_err\\_setpoint yaw\\_line yaw\\_setpoint");
        add_data_header("[ss] m m rad rad");
        add_data_header("[u] cross\\_track\\_err\\_input atan\\_cross\\_track\\_error\\_input");
        add_data_header("[u] m rad");
        add_data_header("[y] yaw\\_output");
        add_data_header("[y] rad");
        add_data_header("[x] integrator");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<NavigationMsg>("nav/inertial_nav", min_input_rate);
        add_setpoint<LineSetpointMsg>("setpoint/line", min_setpoint_rate);


        // Read in state space model parameters from the config file
        int p = get_param<int>("~num_inputs");
        int n = get_param<int>("~num_states");
        int q = get_param<int>("~num_outputs");
        doubles_t A_vect  = get_param<doubles_t>("~A");
        doubles_t B_vect  = get_param<doubles_t>("~B");
        doubles_t C_vect  = get_param<doubles_t>("~C");
        doubles_t D_vect  = get_param<doubles_t>("~D");
        doubles_t x0_vect = get_param<doubles_t>("~x0");

        // Convert the double vectors to eigen matrices and vectors
        MatrixXd A  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(A_vect.data(), n, n);
        MatrixXd B  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(B_vect.data(), n, p);
        MatrixXd C  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(C_vect.data(), q, n);
        MatrixXd D  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(D_vect.data(), q, p);
        VectorXd x0 = Map<VectorXd>(x0_vect.data(), n);

        // Configure the state space controller
        ss_model.initialize(A, B, C, D, x0);

        // Set output limits of state space controller
        double min_output = avl::deg_to_rad(get_param<double>("~output_min"));
        double max_output = avl::deg_to_rad(get_param<double>("~output_max"));

        ss_model.add_output_limit(0, min_output, max_output);

        // Set state limit for integrator
        double integrator_limit = avl::deg_to_rad(get_param<double>("~max_integrator_yaw"));
        ss_model.add_state_limit(0, -integrator_limit, integrator_limit);

        // Set the lookahead distance for atan2
        lookahead_dist = get_param<double>("~lookahead_distance");

        // Set up the output publisher
        output_pub = node_handle->advertise<Float64SetpointMsg>(
            "setpoint/yaw", 1);

    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function when the node is started. Can
    //              be overriden by a derived node class.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        disable();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{

    StateSpaceLineControlNode node(argc, argv);
    node.start();
    return 0;

}
