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
//              error between the vehicle's position and the line.
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

// PID controller class
#include <avl_control/algorithm/pid.h>

// Util functions
#include <avl_core/util/geo.h>

// ROS messages
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/LineSetpointMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class LineControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        LineControlNode constructor
    //--------------------------------------------------------------------------
    LineControlNode(int argc, char **argv) : ControlNode(argc, argv)
    {

    }

private:

    // PID controller instance
    Pid pid;

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

        // Update the PID with the input and setpoint to calculate the output.
        // In this case, the setpoint is zero cross track error. The output
        // is an adjustment to the nominal yaw of the vehicle pointing
        // parallel to the line
        IterationInfo info;
        double pid_output = pid.iterate(cross_track_err, 0.0, info);

        // Calculate the yaw setpoint to publish. Ideally the vehicle will
        // maintain a yaw parallel to the line. However, if the vehicle has
        // cross track error to the line, it should be pointed more toward
        // the line
        double yaw_setpoint = line_yaw + pid_output;

        // Create the yaw setpoint message and publish it
        Float64SetpointMsg output_msg;
        output_msg.enable = true;
        output_msg.data = yaw_setpoint;
        output_pub.publish(output_msg);

        log_data("[line] %.8f %.8f %.8f %.8f %.8f %.8f",
            lat_start, lon_start, lat_end, lon_end, lat_veh, lon_veh);

        log_data("[pid] %.2f %.2f %.4f %.4f",
            cross_track_err, 0.0, line_yaw, yaw_setpoint);

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
        add_data_header("[line] lat_{start} lon_{start} lat_{end} lon_{end} lat_{vehicle} lon_{vehicle}");
        add_data_header("[line] rad rad rad rad rad rad");
        add_data_header("[pid] cross\\_track\\_err\\_input cross\\_track\\_err\\_setpoint yaw\\_line yaw\\_output");
        add_data_header("[pid] m m rad rad");
        add_data_header("[pid_info] input\\_rate error p\\_output i\\_output d\\_output "
        "i\\_error\\_sum output unclamped\\_output output\\_saturated "
        "i\\_saturated");
        add_data_header("[pid_info] m/s m rad rad rad rad rad rad bool bool");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<NavigationMsg>("nav/inertial_nav", min_input_rate);
        add_setpoint<LineSetpointMsg>("setpoint/line", min_setpoint_rate);

        // Configure the waypoint PID from the config file
        pid.set_iteration_rate(get_param<double>("~iteration_rate"));
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

    LineControlNode node(argc, argv);
    node.start();
    return 0;

}
