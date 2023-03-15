//==============================================================================
// Autonomous Vehicle Library
//
// Description: Control node for control of vehicle height (the distance between
//              the seafloor and the vehicle) using a simple PID where
//              input is the vehicle's measured depth and output is a pitch
//              setpoint. This node is meant to be cascaded with an attitude
//              control node.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/depth (avl_msgs/Float64SetpointMsg)
//
// Subscribers: device/height (std_msgs/Float64)
//              setpoint/height (avl_msgs/Float64SetpointMsg)
//==============================================================================

// Control node base class
#include <avl_control/control_node.h>

// ROS messages
#include <avl_msgs/Float64SetpointMsg.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// PID controller class
#include <avl_control/algorithm/pid.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class HeightControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        HeightControlNode constructor
    //--------------------------------------------------------------------------
    HeightControlNode(int argc, char **argv) : ControlNode(argc, argv)
    {

    }

private:

    // PID controller instance
    Pid pid;

    // Publisher for output messages
    ros::Publisher output_pub;

    // Maximum depth from config file
    double max_depth;

    size_t max_num_nans;
    size_t num_nans;
    double prev_depth_output;

private:

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on each controller iteration. Controller iteration
    //              logic and output publishing should be implemented here.
    //--------------------------------------------------------------------------
    void iterate()
    {

        // Get the input and setpoint
        double height_input = get_input<std_msgs::Float64>("device/height").data;
        double depth_input = get_input<std_msgs::Float64>("device/depth").data;
        double height_setpoint = get_float64_setpoint("setpoint/height");

        // Depth output is only updated if the height input is not NaN,
        // otherwise the previous depth output value will be maintained. This
        // is to handle loss of bottom lock. If more than the maximum allowable
        // consecutive NaNs is received, the node will shut down.
        double depth_output = prev_depth_output;
        IterationInfo info;
        if (!std::isnan(height_input))
        {

            // Calculate the PID output
            depth_output = depth_input -
                pid.iterate(height_input, height_setpoint, info);

            // Ensure the depth output does not exceed the max depth
            depth_output = std::min(depth_output, max_depth);

            // Reset the number of received NaN height measurements
            prev_depth_output = depth_output;
            num_nans = 0;

        }
        else
        {
            num_nans++;
            if (max_num_nans > 0 && num_nans > max_num_nans)
                throw std::runtime_error("received maximum number of NaN "
                    "height measurements");
        }

        // Create the depth setpoint message and publish it
        Float64SetpointMsg output_msg;
        output_msg.enable = true;
        output_msg.data = depth_output;
        output_pub.publish(output_msg);

        log_data("[pid] %.2f %.2f %.2f %.2f",
            height_input, depth_input, height_setpoint, depth_output);

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
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[pid] height\\_input depth\\_input height\\_setpoint depth\\_output");
        add_data_header("[pid] m m m m");
        add_data_header("[pid_info] input\\_rate error p\\_output i\\_output d\\_output "
        "i\\_error\\_sum output unclamped\\_output output\\_saturated "
        "i\\_saturated");
        add_data_header("[pid_info] m/s m m m m m m m bool bool");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<std_msgs::Float64>("device/height", min_input_rate);
        add_input<std_msgs::Float64>("device/depth", min_input_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/height",
            min_setpoint_rate);

        // Set the PID parameters
        pid.set_iteration_rate(get_param<double>("~iteration_rate"));
        pid.set_gains(get_param<double>("~pid/kp"),
                      get_param<double>("~pid/ki"),
                      get_param<double>("~pid/kd"));
        pid.set_output_limits(get_param<double>("~pid/output_min"),
                              get_param<double>("~pid/output_max"));
        pid.set_integral_limits(get_param<double>("~pid/integral_output_min"),
                                get_param<double>("~pid/integral_output_max"));

        // Get the max depth and max NaNs from the config file
        max_depth = get_param<double>("~max_depth");
        max_num_nans = get_param<int>("~max_num_nans");

        // Set up the output publisher
        output_pub = node_handle->advertise<Float64SetpointMsg>(
            "setpoint/depth", 1);

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
    HeightControlNode node(argc, argv);
    node.start();
    return 0;
}
