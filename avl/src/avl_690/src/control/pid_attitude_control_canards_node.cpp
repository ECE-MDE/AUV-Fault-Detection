//==============================================================================
// Autonomous Vehicle Library
//
// Description: Node for control of attitude using one basic PID controller for
//              each roll, pitch, and yaw. Controller inputs are roll, pitch,
//              and yaw angles from an AHRS. Each angle takes a separate
//              setpoint. The node also accepts elevator and rudder setpoints
//              that will take prescedence over fin angles generated from
//              attitude setpoints.
//                  Fin mixing can be enabled, which rotates fin angles to
//              compensate for roll. For example, if roll is 90 radrees, the
//              rudders are now elevators, and the control node will treat them
//              as such. Without fin mixing, control will not work correctly if
//              roll is significant. If fin mixing is disabled, zero roll is
//              assumed.
//
// Servers:     None
//
// Clients:     /device/reset_fins (std_srvs/Trigger)
//
// Publishers:  /device/fins (avl_devices/FinsMsg)
//
// Subscribers: /nav/inertial_nav (avl_navigation/NavigationMsg)
//              /device/imu (avl_devices/ImuMsg)
//              /setpoint/elevator (avl_control/Float64SetpointMsg)
//              /setpoint/rudder (avl_control/Float64SetpointMsg)
//              /setpoint/roll (avl_control/Float64SetpointMsg)
//              /setpoint/pitch (avl_control/Float64SetpointMsg)
//              /setpoint/yaw (avl_control/Float64SetpointMsg)
//==============================================================================

// Control node base class
#include <avl_control/control_node.h>

// PID controller class
#include <avl_control/algorithm/pid.h>

// Utility functions
#include <avl_core/util/math.h>

// ROS messages
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/ImuMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/FinsMsg.h>
#include <std_srvs/Trigger.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class PidAttitudeControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        PidAttitudeControlNode constructor
    //--------------------------------------------------------------------------
    PidAttitudeControlNode(int argc, char **argv) : ControlNode(argc, argv),
        roll_pid(PID_ANGLE_RAD), pitch_pid(PID_ANGLE_RAD),
        yaw_pid(PID_ANGLE_RAD)
    {

    }

private:

    // PID controller instances for roll, pitch, and yaw control
    Pid roll_pid;
    Pid pitch_pid;
    Pid yaw_pid;

    // Publisher for fins messages as the controller output
    ros::Publisher fins_pub;

    // Publisher for the front fins messages as controller output
    ros::Publisher front_fins_pub;

    // ROS client to call the fins node's reset fins service
    ros::ServiceClient reset_fins_client;

    // Flag indicating that fin mixing is enabled. If fin mixing is enabled,
    // the roll, pitch, and yaw PID outputs are mixed based on the
    // vehicle's roll angle. If it is disabled, zero roll is assumed
    bool enable_fin_mixing = false;

    // Flag indicating if there are canards equipped
    bool canards_equipped = false;

private:

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on each controller iteration. Controller iteration
    //              logic and output publishing should be implemented here.
    //--------------------------------------------------------------------------
    void iterate()
    {

        // Get the input and setpoints
        NavigationMsg nav_input = get_input<NavigationMsg>("/nav/inertial_nav");
        ImuMsg imu_input = get_input<ImuMsg>("/device/imu");
        double roll_input =  nav_input.roll;
        double pitch_input = nav_input.pitch;
        double yaw_input =   nav_input.yaw;
        double roll_rate =   imu_input.angular_velocity.x;
        double pitch_rate =  imu_input.angular_velocity.y;
        double yaw_rate =    imu_input.angular_velocity.z;
        double elevator_setpoint = get_float64_setpoint("/setpoint/elevator");
        double rudder_setpoint =   get_float64_setpoint("/setpoint/rudder");
        double front_elevator_setpoint = get_float64_setpoint("/setpoint/front_elevator");
        double front_rudder_setpoint =   get_float64_setpoint("/setpoint/front_rudder");
        double roll_setpoint =     get_float64_setpoint("/setpoint/roll");
        double pitch_setpoint =    get_float64_setpoint("/setpoint/pitch");
        double yaw_setpoint =      get_float64_setpoint("/setpoint/yaw");

        // Update the roll, pitch, and yaw PID output values if their respective
        // setpoints are not NaN, representing no control
        IterationInfo roll_info;
        double roll_output = 0.0;
        if(!std::isnan(roll_setpoint))
            roll_output = roll_pid.iterate(roll_input, roll_rate, roll_setpoint, roll_info);

        IterationInfo pitch_info;
        double elevator_output = avl::deg_to_rad(get_param<double>("~default_elevator_angle"));
        if(!std::isnan(pitch_setpoint))
            elevator_output = pitch_pid.iterate(pitch_input, pitch_rate, pitch_setpoint, pitch_info);

        IterationInfo yaw_info;
        double rudder_output = avl::deg_to_rad(get_param<double>("~default_rudder_angle"));
        if(!std::isnan(yaw_setpoint))
            rudder_output = yaw_pid.iterate(yaw_input, yaw_rate, yaw_setpoint, yaw_info);

        // Assume zero roll angle is zero unless fin mixing is enabled, in
        // which case the roll angle is the vehicle's measured roll angle
        // in radians
        double roll = 0.0;
        if (enable_fin_mixing)
            roll = roll_input;

        // If there are elevator or rudder setpoints, they take precedence over
        // the fin angles from attitude setpoints
        elevator_output = std::isnan(elevator_setpoint) ? elevator_output : elevator_setpoint;
        rudder_output = std::isnan(rudder_setpoint) ? rudder_output : rudder_setpoint;

        // Calculate the fin angles by mixing the roll, pitch, and yaw PID
        // outputs based on the vehicle's roll
        double sin_roll = sin(roll);
        double cos_roll = cos(roll);
        double port_angle      =  elevator_output * cos_roll - rudder_output   * sin_roll + roll_output;
        double starboard_angle = -elevator_output * cos_roll + rudder_output   * sin_roll + roll_output;
        double top_angle       =  rudder_output   * cos_roll - elevator_output * sin_roll + roll_output;
        double bottom_angle    = -rudder_output   * cos_roll + elevator_output * sin_roll + roll_output;

        // Create the fins message and publish it
        FinsMsg fins_msg;
        fins_msg.port = port_angle;
        fins_msg.starboard = starboard_angle;
        fins_msg.top = top_angle;
        fins_msg.bottom = bottom_angle;
        fins_pub.publish(fins_msg);

        // Do front fin calculations if they are equipped
        if(canards_equipped)
        {
            // Front elevator
            double front_elevator_output = avl::deg_to_rad(get_param<double>("~default_front_elevator_angle"));

            // Front rudder
            double front_rudder_output = avl::deg_to_rad(get_param<double>("~default_front_rudder_angle"));

            // If there are elevator or rudder setpoints, they take precedence over
            // the fin angles from attitude setpoints
            front_elevator_output = std::isnan(front_elevator_setpoint) ? front_elevator_output : front_elevator_setpoint;
            front_rudder_output = std::isnan(front_rudder_setpoint) ? front_rudder_output : front_rudder_setpoint;

            // Calculate front fin angles by mixing the control outputs using the vehicle's roll
            double front_port_angle      = -front_elevator_output * cos_roll + front_rudder_output   * sin_roll + roll_output;
            double front_starboard_angle =  front_elevator_output * cos_roll - front_rudder_output   * sin_roll + roll_output;
            double front_top_angle       = -front_rudder_output   * cos_roll + front_elevator_output * sin_roll + roll_output;
            double front_bottom_angle    =  front_rudder_output   * cos_roll - front_elevator_output * sin_roll + roll_output;

            // Create the front fin message and publish it
            FinsMsg front_fins_msg;
            front_fins_msg.port = front_port_angle;
            front_fins_msg.starboard = front_starboard_angle;
            front_fins_msg.top = front_top_angle;
            front_fins_msg.bottom = front_bottom_angle;
            front_fins_pub.publish(front_fins_msg);

            log_data("[canards] %.4f %.4f %.4f %.4f",
                front_port_angle, front_starboard_angle, front_top_angle, front_bottom_angle);
        }

        // Log data
        log_data("[roll_pid] %.4f %.4f %.4f",
            roll_input, roll_setpoint, roll_output);

        log_data("[pitch_pid] %.4f %.4f %.4f",
            pitch_input, pitch_setpoint, elevator_output);

        log_data("[yaw_pid] %.4f %.4f %.4f",
            yaw_input, yaw_setpoint, rudder_output);

        log_data("[roll_info] %f %f %f %f %f %f %f %f %d %d",
            roll_rate,
            roll_info.error,
            roll_info.p_output,
            roll_info.i_output,
            roll_info.d_output,
            roll_info.i_error_sum,
            roll_info.output,
            roll_info.unclamped_output,
            roll_info.output_saturated,
            roll_info.i_saturated);

        log_data("[pitch_info] %f %f %f %f %f %f %f %f %d %d",
            pitch_rate,
            pitch_info.error,
            pitch_info.p_output,
            pitch_info.i_output,
            pitch_info.d_output,
            pitch_info.i_error_sum,
            pitch_info.output,
            pitch_info.unclamped_output,
            pitch_info.output_saturated,
            pitch_info.i_saturated);

        log_data("[yaw_info] %f %f %f %f %f %f %f %f %d %d",
            yaw_rate,
            yaw_info.error,
            yaw_info.p_output,
            yaw_info.i_output,
            yaw_info.d_output,
            yaw_info.i_error_sum,
            yaw_info.output,
            yaw_info.unclamped_output,
            yaw_info.output_saturated,
            yaw_info.i_saturated);

        log_data("[fins] %.4f %.4f %.4f %.4f",
            port_angle, starboard_angle, top_angle, bottom_angle);

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

        // Call the fin reset service to return fins to their home positions
        std_srvs::Trigger empty_service_msg;
        reset_fins_client.call(empty_service_msg);

        // Reset the controller integral error
        roll_pid.clear_integral_error();
        pitch_pid.clear_integral_error();
        yaw_pid.clear_integral_error();

        // Log NAN data entries to split up data plots between subsequent
        // control actions
        log_data("[roll_pid] %.4f %.4f %.4f",
            NAN, NAN, NAN);
        log_data("[pitch_pid] %.4f %.4f %.4f",
            NAN, NAN, NAN);
        log_data("[yaw_pid] %.4f %.4f %.4f",
            NAN, NAN, NAN);
        log_data("[fins] %.4f %.4f %.4f %.4f",
            NAN, NAN, NAN, NAN);
        log_data("[roll_info] %f %f %f %f %f %f %f %f %d %d",
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
        log_data("[pitch_info] %f %f %f %f %f %f %f %f %d %d",
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
        log_data("[yaw_info] %f %f %f %f %f %f %f %f %d %d",
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
        log_data("[canards] %.4f %.4f %.4f %.4f",
            NAN, NAN, NAN, NAN);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[roll_pid] roll\\_input roll\\_setpoint fin\\_angle\\_output");
        add_data_header("[roll_pid] rad rad rad");
        add_data_header("[pitch_pid] pitch\\_input pitch\\_setpoint elevator\\_angle\\_output");
        add_data_header("[pitch_pid] rad rad rad");
        add_data_header("[yaw_pid] yaw\\_input yaw\\_setpoint rudder\\_angle\\_output");
        add_data_header("[yaw_pid] rad rad rad");

        add_data_header("[roll_info] input\\_rate error p\\_output i\\_output d\\_output "
        "i\\_error\\_sum output unclamped\\_output output\\_saturated "
        "i\\_saturated");
        add_data_header("[roll_info] rad/s rad rad rad rad rad rad rad bool bool");

        add_data_header("[pitch_info] input\\_rate error p\\_output i\\_output d\\_output "
        "i\\_error\\_sum output unclamped\\_output output\\_saturated "
        "i\\_saturated");
        add_data_header("[pitch_info] rad/s rad rad rad rad rad rad rad bool bool");

        add_data_header("[yaw_info] input\\_rate error p\\_output i\\_output d\\_output "
        "i\\_error\\_sum output unclamped\\_output output\\_saturated "
        "i\\_saturated");
        add_data_header("[yaw_info] rad/s rad rad rad rad rad rad rad bool bool");

        add_data_header("[fins] port starboard top bottom");
        add_data_header("[fins] rad rad rad rad");

        add_data_header("[canards] port starboard top bottom");
        add_data_header("[canards] rad rad rad rad");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<NavigationMsg>("/nav/inertial_nav", min_input_rate);
        add_input<ImuMsg>("/device/imu", min_input_rate);
        add_setpoint<Float64SetpointMsg>("/setpoint/elevator", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("/setpoint/rudder", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("/setpoint/front_elevator", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("/setpoint/front_rudder", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("/setpoint/roll", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("/setpoint/pitch", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("/setpoint/yaw", min_setpoint_rate);

        // Configure the roll PID from the config file
        roll_pid.set_iteration_rate(get_param<double>("~iteration_rate"));
        roll_pid.set_gains(get_param<double>("~roll_pid/kp"),
                           get_param<double>("~roll_pid/ki"),
                           get_param<double>("~roll_pid/kd"));
        roll_pid.set_output_limits(
            avl::deg_to_rad(get_param<double>("~roll_pid/output_min")),
            avl::deg_to_rad(get_param<double>("~roll_pid/output_max")));
        roll_pid.set_integral_limits(
            avl::deg_to_rad(get_param<double>("~roll_pid/integral_output_min")),
            avl::deg_to_rad(get_param<double>("~roll_pid/integral_output_max")));

        // Configure the pitch PID from the config file
        pitch_pid.set_iteration_rate(get_param<double>("~iteration_rate"));
        pitch_pid.set_gains(get_param<double>("~pitch_pid/kp"),
                            get_param<double>("~pitch_pid/ki"),
                            get_param<double>("~pitch_pid/kd"));
        pitch_pid.set_output_limits(
            avl::deg_to_rad(get_param<double>("~pitch_pid/output_min")),
            avl::deg_to_rad(get_param<double>("~pitch_pid/output_max")));
        pitch_pid.set_integral_limits(
            avl::deg_to_rad(get_param<double>("~pitch_pid/integral_output_min")),
            avl::deg_to_rad(get_param<double>("~pitch_pid/integral_output_max")));

        // Configure the pitch PID from the config file
        yaw_pid.set_iteration_rate(get_param<double>("~iteration_rate"));
        yaw_pid.set_gains(get_param<double>("~yaw_pid/kp"),
                          get_param<double>("~yaw_pid/ki"),
                          get_param<double>("~yaw_pid/kd"));
        yaw_pid.set_output_limits(
            avl::deg_to_rad(get_param<double>("~yaw_pid/output_min")),
            avl::deg_to_rad(get_param<double>("~yaw_pid/output_max")));
        yaw_pid.set_integral_limits(
            avl::deg_to_rad(get_param<double>("~yaw_pid/integral_output_min")),
            avl::deg_to_rad(get_param<double>("~yaw_pid/integral_output_max")));

        // Get the fin mixing flag from the config file
        enable_fin_mixing = get_param<bool>("~enable_fin_mixing");

        // Determine if the canards are equipped
        canards_equipped = get_param<bool>("~canards");

        // Set up the output publisher
        fins_pub = node_handle->advertise<FinsMsg>(
            "/device/fins", 1);

        // Set up the front fins output publisher
        front_fins_pub = node_handle->advertise<FinsMsg>(
            "/device/canards", 1);

        // Set up the reset fins service client
        reset_fins_client = node_handle->serviceClient<std_srvs::Trigger>(
            "/device/reset_fins");

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
    PidAttitudeControlNode node(argc, argv);
    node.start();
    return 0;
}
