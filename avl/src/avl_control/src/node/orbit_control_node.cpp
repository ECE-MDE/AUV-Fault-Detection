//==============================================================================
// Autonomous Vehicle Library
//
// Description: Control node that commands attempts to follow an orbit defined
//              by a center point, a radius, and a direction. The vehicle
//              position will converge from its current position to the closest
//              point on orbit by controlling vehicle yaw. It will continue
//              follow the orbit indefinitely until a new orbit is speciified
//              or orbit control is disabled. The yaw setpoint output is
//              controlled by a PID attempting to maintain zero error
//              between the vehicle's distance from the center and the desired
//              orbit radius.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/yaw (avl_msgs/Float64SetpointMsg)
//
// Subscribers: nav/inertial_nav (avl_msgs/NavigationMsg)
//              setpoint/orbit (avl_msgs/LineSetpointMsg)
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
#include <avl_msgs/OrbitSetpointMsg.h>
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
        OrbitSetpointMsg setpoint = get_setpoint<OrbitSetpointMsg>("setpoint/orbit");
        double lat_orbit = setpoint.lat;
        double lon_orbit = setpoint.lon;
        double radius = setpoint.radius;
        double direction = setpoint.clockwise ? -1.0 : 1.0;
        double lat_veh = input.lat;
        double lon_veh = input.lon;

        // Find the yaw from the vehicle to the orbit center
        double yaw_to_center = avl::wrap_to_2pi(
            avl::initial_bearing(lat_veh,   lon_veh,
                                 lat_orbit, lon_orbit));

        // Find the distance from the vehicle to the orbit center
        double distance = avl::distance(lat_orbit, lon_orbit,
                                        lat_veh,   lon_veh);

        // Iterate the PID output, which is an adjustment to the nominal
        // yaw of the vehicle pointing perpendicular to the orbit center
        IterationInfo info;
        double pid_output = pid.iterate(distance, radius, info);

        // Calculate the yaw setpoint to publish. Ideally the vehicle will
        // maintain a +/- 90 radree yaw to the center of the orbit center in
        // order to continue in a circle around it. However, if the vehicle is
        // further than the desired orbit radius, the vehicle should be pointed
        // more toward the center. If the vehicle is closer, it should point
        // further away from the center.
        double yaw_perp = yaw_to_center + direction * (M_PI/2.0);
        double yaw_setpoint = yaw_perp + (direction * pid_output);

        // Create the yaw setpoint message and publish it
        Float64SetpointMsg output_msg;
        output_msg.enable = true;
        output_msg.data = yaw_setpoint;
        output_pub.publish(output_msg);

        log_data("[orbit] %.8f %.8f %.6f %d",
            lat_orbit, lon_orbit, radius, setpoint.clockwise);

        log_data("[pid] %.2f %.2f %.4f %.4f",
            distance, radius, yaw_perp, yaw_setpoint);

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
        add_data_header("[orbit] lat_{center} lon_{center} radius clockwise");
        add_data_header("[orbit] rad rad m bool");
        add_data_header("[pid] distance\\_input distance\\_setpoint yaw\\_perp yaw\\_output");
        add_data_header("[pid] m m rad rad");
        add_data_header("[pid_info] input\\_rate error p\\_output i\\_output d\\_output "
        "i\\_error\\_sum output unclamped\\_output output\\_saturated "
        "i\\_saturated");
        add_data_header("[pid_info] m/s m rad rad rad rad rad bool bool");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<NavigationMsg>("nav/inertial_nav", min_input_rate);
        add_setpoint<OrbitSetpointMsg>("setpoint/orbit", min_setpoint_rate);

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
