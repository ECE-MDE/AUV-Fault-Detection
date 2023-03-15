//==============================================================================
// Autonomous Vehicle Library
//
// Description: Control node for control of vehicle depth (the distance between
//              the water surface and the vehicle) using a simple PID where
//              input is the vehicle's measured depth and output is a pitch
//              setpoint. This node is meant to be cascaded with an attitude
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

// ROS messages
#include <avl_msgs/Float64SetpointMsg.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// Utility Functions
#include <avl_core/util/math.h>

// PID controller class
#include <avl_control/algorithm/pid.h>

// Deque class for moving average
#include <deque>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class PidDepthControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        PidDepthControlNode constructor
    //--------------------------------------------------------------------------
    PidDepthControlNode(int argc, char **argv) : ControlNode(argc, argv)
    {

    }

private:

    // PID controller instance
    Pid pid;

    // Publisher for output messages
    ros::Publisher output_pub;

    double iteration_rate;
    double input_prev = 0.0;
    std::deque<double> rate_buf;
    size_t moving_avg_size;

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

        // Update the PID with the input and setpoint to calculate the output
        IterationInfo info;
        double output = pid.iterate(input, avg_rate, setpoint, info);

        // Create the pitch setpoint message and publish it
        Float64SetpointMsg output_msg;
        output_msg.enable = true;
        output_msg.data = output;
        output_pub.publish(output_msg);

        log_data("[pid] %.2f %.2f %.4f",
            input, setpoint, output);

        log_data("[pid_info] %f %f %f %f %f %f %f %f %d %d",
            info.input_rate,
            info.error,
            info.p_output,
            info.i_output,
            info.d_output,
            info.i_error_sum,
            info.output,
            info.unclamped_output,
            info.output_saturated,
            info.i_saturated);

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

        pid.clear_integral_error();
        Float64SetpointMsg output_msg;
        output_msg.enable = false;
        output_msg.data = 0.0;
        output_pub.publish(output_msg);

        // Log NAN data entries to split up data plots between subsequent
        // control actions
        log_data("[pid] %.4f %.4f %.4f",
            NAN, NAN, NAN);

        log_data("[pid_info] %f %f %f %f %f %f %f %f %d %d",
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 0, 0);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[pid] depth\\_input depth\\_setpoint pitch\\_output");
        add_data_header("[pid] m m rad");
        add_data_header("[pid_info] input\\_rate error p\\_output i\\_output d\\_output "
        "i\\_error\\_sum output unclamped\\_output output\\_saturated "
        "i\\_saturated");
        add_data_header("[pid_info] m/s m rad rad rad rad rad rad bool bool");

        // Input filter moving average size
        moving_avg_size = get_param<int>("~moving_avg_size");

        // Set the control node iteration rate
        iteration_rate = get_param<double>("~iteration_rate");
        set_iteration_rate(iteration_rate);

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<std_msgs::Float64>("device/depth", min_input_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/depth",
            min_setpoint_rate);

        // Set the PID parameters
        pid.set_iteration_rate(iteration_rate);
        pid.set_gains(get_param<double>("~pid/kp"),
                      get_param<double>("~pid/ki"),
                      get_param<double>("~pid/kd"));
        pid.set_output_limits(
            avl::deg_to_rad(get_param<double>("~pid/output_min")),
            avl::deg_to_rad(get_param<double>("~pid/output_max")));
        pid.set_integral_limits(
            avl::deg_to_rad(get_param<double>("~pid/integral_output_min")),
            avl::deg_to_rad(get_param<double>("~pid/integral_output_max")));

        // Set up the output publisher
        output_pub = node_handle->advertise<Float64SetpointMsg>(
            "setpoint/pitch", 1);

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
    PidDepthControlNode node(argc, argv);
    node.start();
    return 0;
}
