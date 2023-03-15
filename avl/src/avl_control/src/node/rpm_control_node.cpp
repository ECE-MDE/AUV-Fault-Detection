//==============================================================================
// Autonomous Vehicle Library
//
// Description: Control node for control of propeller RPM using a simple PID
//              with input of the propeller's measured RPM and output of a
//              motor percentage message.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/motor (std_msgs/Float64)
//
// Subscribers: device/rpm (std_msgs/Float64)
//              setpoint/rpm (avl_msgs/Float64SetpointMsg)
//==============================================================================

// Control node base class
#include <avl_control/control_node.h>

// Util functions
#include "avl_core/util/math.h"

// PID controller class
#include <avl_control/algorithm/pid.h>

// ROS messages
#include <std_msgs/Float64.h>
#include <avl_msgs/Float64SetpointMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class RpmControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        RpmControlNode constructor
    //--------------------------------------------------------------------------
    RpmControlNode(int argc, char **argv) : ControlNode(argc, argv)
    {

    }

private:

    // PID controller instance
    Pid pid;

    // Motor percentage deadband value from config file
    double deadband;

    // Publisher for output messages
    ros::Publisher output_pub;

private:

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on each controller iteration. Controller iteration
    //              logic and output publishing should be implemented here.
    //--------------------------------------------------------------------------
    void iterate()
    {

        // Get the input and setpoint
        double rpm_input = get_input<std_msgs::Float64>("device/rpm").data;
        double rpm_setpoint = get_float64_setpoint("setpoint/rpm", 0.0);

        // Update the throttle output value
        IterationInfo info;
        double pid_output = pid.iterate(rpm_input, rpm_setpoint, info);
        double throttle = deadband + pid_output;
        throttle = avl::clamp(throttle, 0.0, 100.0);

        // Create and publish the output message
        std_msgs::Float64 output_msg;
        output_msg.data = throttle;
        output_pub.publish(output_msg);

        log_data("[pid] %.2f %.2f %.2f",
            rpm_input, rpm_setpoint, throttle);

        log_data("[info] %f %f %f %f %f %f %f %f %d %d",
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
        std_msgs::Float64 output_msg;
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
        add_data_header("[pid] rpm\\_input rpm\\_setpoint motor\\_output");
        add_data_header("[pid] rpm rpm %%");

        add_data_header("[info] input_rate error p_output i_output d_output i_error_sum output unclamped_output output_saturated i_saturated");
        add_data_header("[info] rpm/s rpm %% %% %% %% %% %% bool bool");

        // Get config file settings
        deadband = get_param<double>("~deadband");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<std_msgs::Float64>("device/rpm", min_input_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/rpm",
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

        // Set up the output publisher
        output_pub = node_handle->advertise<std_msgs::Float64>(
            "device/motor", 1);

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
    RpmControlNode node(argc, argv);
    node.start();
    return 0;
}
