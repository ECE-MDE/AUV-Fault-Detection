//==============================================================================
// Autonomous Vehicle Library
//
// Description: Control node for control of vehicle depth (the distance between
//              the water surface and the vehicle) using a simple state space
//              model. This node is meant to be cascaded with an attitude
//              control node.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/pitch (avl_msgs/Float64SetpointMsg)
//
// Subscribers: device/depth (std_msgs/Float64)
//              setpoint/depth (avl_msgs/Float64SetpointMsg)
//==============================================================================

// Control node base class
#include <avl_control/control_node.h>

// State space model
#include <avl_control/algorithm/state_space_model.h>

// ROS messages
#include <avl_msgs/Float64SetpointMsg.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// Deque class for moving average
#include <deque>

// Typedef for a vector of doubles
typedef std::vector<double> doubles_t;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class StateSpaceDepthControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        DepthControlNode constructor
    //--------------------------------------------------------------------------
    StateSpaceDepthControlNode(int argc, char **argv) : ControlNode(argc, argv)
    {

    }

private:

    // Statespace model
    StateSpaceModel ss_model;

    // Publisher for fins messages
    ros::Publisher output_pub;

    double iteration_rate;
    double input_prev = 0.0;
    std::deque<double> rate_buf;
    size_t moving_avg_size;

    double max_input_error;

private:

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on each controller iteration. Controller iteration
    //              logic and output publishing should be implemented here.
    //--------------------------------------------------------------------------
    void iterate()
    {

        // Get the input and setpoint
        double input = get_input<std_msgs::Float64>("device/depth").data;
        double setpoint = get_float64_setpoint("setpoint/depth");

        // Calculate the depth rate
        double rate = (input - input_prev) * iteration_rate;
        input_prev = input;

        // Add the input to the buffer
        rate_buf.push_back(rate);
        if (rate_buf.size() > moving_avg_size)
            rate_buf.pop_front();

        // Calculate the input rate moving average
        double avg_rate = 0.0;
        for (double rate: rate_buf)
            avg_rate+=rate;
        avg_rate /= rate_buf.size();

        log_data("[ss] %.4f %.4f %.4f", setpoint, input, avg_rate);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Calculate error

        double depth_error =  setpoint - input;

        // Clamp the input to the maximum allowed input magnitude
        depth_error = avl::clamp(depth_error, -max_input_error, max_input_error);

        // Create Input Vector ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Input vector is the following 2x1 vector:
        //     u = [depth_error;  depth_rate]
        VectorXd u(1);
        u << depth_error;

        // Log the input
        log_data("[u] %.4f", u(0));

        // Get Output Vector  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Get the controller output by iterating the state space model. The
        // output vector is the following 1x1 vector:
        //     y = [pitch_setpoint]
        VectorXd y = ss_model.iterate(u);

        log_data("[y] %.4f", y(0));

        // Create the pitch setpoint message and publish it
        Float64SetpointMsg output_msg;
        output_msg.enable = true;
        output_msg.data = y(0);
        output_pub.publish(output_msg);

        // Log the state
        VectorXd x = ss_model.get_state();
        log_vector("x", x, 4);
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

        // Publish a pitch setpoint to disable pitch control
        Float64SetpointMsg output_msg;
        output_msg.enable = false;
        output_msg.data = 0.0;
        output_pub.publish(output_msg);

        // Log NAN data entries to split up data plots between subsequent
        // control actions
        log_data("[ss] %.4f %.4f %.4f", NAN, NAN, NAN);
        log_data("[u] %.4f", NAN);
        log_data("[y] %.4f", NAN);
        log_data("[x] %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
                       NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[ss] depth\\_setpoint depth\\_input depth\\_rate");
        add_data_header("[ss] m m m/s");
        add_data_header("[u] depth\\_error");
        add_data_header("[u] m");
        add_data_header("[y] pitch\\_setpoint");
        add_data_header("[y] rad");
        add_data_header("[x] 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17");
        add_data_header("[x] ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ?");

        // Set the control node iteration rate
        iteration_rate = get_param<double>("~iteration_rate");
        set_iteration_rate(iteration_rate);

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<std_msgs::Float64>("device/depth", min_input_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/depth",
            min_setpoint_rate);

        // Set up the output publisher
        output_pub = node_handle->advertise<Float64SetpointMsg>(
            "setpoint/pitch", 1);

        // Get the moving average size from the config file
        moving_avg_size = get_param<int>("~moving_avg_size");

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

        // Initialize the state space model with the parameters
        ss_model.initialize(A, B, C, D, x0);

        // Set output limit
        double max_pitch = avl::deg_to_rad(get_param<double>("~max_pitch"));
        ss_model.add_output_limit(0, -max_pitch, max_pitch);

        // Set input limits
        max_input_error = get_param<double>("~max_input_error");
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
    StateSpaceDepthControlNode node(argc, argv);
    node.start();
    return 0;
}
