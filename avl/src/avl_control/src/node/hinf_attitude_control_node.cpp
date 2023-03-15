//==============================================================================
// Autonomous Vehicle Library
//
// Description: Control node for control of vehicle pitch and yaw using a state
//              space H infinity controller as described in the following PhD
//              dissertation:
//
//                  Improved Guidance, Navigation, and Control for Autonomous
//                  Underwater Vehicles: Theory and Experiment by Jan Petrich
//
//                  https://vtechworks.lib.vt.edu/handle/10919/27222
//
//              The controller has the following inputs (u) and outputs (y):
//
//  u = [-pitch_rate; pitch_err; pitch_err_int; -yaw_rate; yaw_err; yaw_err_int]
//  y = [elevator_cmd; rudder_cmd]
//
//              Controller inputs are roll, pitch,
//              and yaw angles from an AHRS. Each angle takes a separate
//              setpoint. The node also accepts elevator and rudder setpoints
//              that will take prescedence over fin angles generated from
//              attitude setpoints.
//                  Fin mixing can be enabled, which rotates fin angles to
//              compensate for roll. For example, if roll is 90 degrees, the
//              rudders are now elevators, and the control node will treat them
//              as such. Without fin mixing, control will not work correctly if
//              roll is significant. If fin mixing is disabled, zero roll is
//              assumed.
//
// Servers:     None
//
// Clients:     device/reset_fins (std_srvs/Trigger)
//
// Publishers:  device/fins (avl_msgs/FinsMsg)
//
// Subscribers: device/ahrs (avl_msgs/AhrsMsg)
//              setpoint/elevator (avl_msgs/Float64SetpointMsg)
//              setpoint/rudder (avl_msgs/Float64SetpointMsg)
//              setpoint/roll (avl_msgs/Float64SetpointMsg)
//              setpoint/pitch (avl_msgs/Float64SetpointMsg)
//              setpoint/yaw (avl_msgs/Float64SetpointMsg)
//==============================================================================


// Control node base class
#include <avl_control/control_node.h>

// State space model for control
#include <avl_control/algorithm/state_space_model.h>

// Util functions
#include <avl_core/util/math.h>

// ROS messages
#include <avl_msgs/AhrsMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/FinsMsg.h>
#include <std_srvs/Trigger.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class HinfAttitudeControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        RpmControlNode constructor
    //--------------------------------------------------------------------------
    HinfAttitudeControlNode(int argc, char **argv) : ControlNode(argc, argv)
    {

    }

private:

    // Statespace model
    StateSpaceModel ss_model;

    // Publisher for fins messages
    ros::Publisher fins_pub;

    // Service client to call the fins node's reset fins service
    ros::ServiceClient reset_fins_client;

    // Flag indicating that fin mixing is enabled. If fin mixing is enabled,
    // the fin outputs are mixed based on the vehicle's roll angle. If it is
    // disabled, zero roll is assumed
    bool enable_fin_mixing = false;

    // Integral error term limits
    double pitch_integral_limit;
    double yaw_integral_limit;

    // Maximum fin angle from config file
    double max_fin_angle;

    // Flags for elevator and rudder saturation. Integral does not need to be
    // increased if fins are saturated since maximum effort is already being
    // exerted
    double elevator_saturated = false;
    double rudder_saturated = false;

    // Time step in seconds
    double dt;

    // Angle in degrees that the elevators will be moved to when there is no
    // pitch command. Comes from the config file
    double elevators_up_angle;

    double pitch_integral = 0.0;
    double yaw_integral = 0.0;

private:

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on each controller iteration. Controller iteration
    //              logic and output publishing should be implemented here.
    //--------------------------------------------------------------------------
    void iterate()
    {

        // Get the input and setpoints
        AhrsMsg input = get_input<AhrsMsg>("device/ahrs");
        double pitch_rate_input =  avl::rad_to_deg(input.w.y);
        double yaw_rate_input =    avl::rad_to_deg(input.w.z);
        double roll_input =        avl::rad_to_deg(input.theta.x);
        double pitch_input =       avl::rad_to_deg(input.theta.y);
        double yaw_input =         avl::rad_to_deg(input.theta.z);
        double elevator_setpoint = avl::rad_to_deg(get_float64_setpoint("setpoint/elevator"));
        double rudder_setpoint =   avl::rad_to_deg(get_float64_setpoint("setpoint/rudder"));
        double pitch_setpoint =    avl::rad_to_deg(get_float64_setpoint("setpoint/pitch"));
        double yaw_setpoint =      avl::rad_to_deg(get_float64_setpoint("setpoint/yaw"));

        log_data("[attitude] %.2f %.2f %.2f %.2f",
            pitch_input, yaw_input, pitch_setpoint, yaw_setpoint);

        // Create Input Vector ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        Eigen::VectorXd u(6);
        u.setZero();

        // If there is a pitch setpoint, set the pitch components of the input
        // vector u, and add to the pitch integral error. If there is no pitch
        // setpoint, the input vector will have zeros for the pitch components
        if (!std::isnan(pitch_setpoint))
        {

            u(0) = -pitch_rate_input;
            u(1) = avl::wrap(pitch_setpoint - pitch_input, false);

            // Add the pitch error times the iteration time to the integral term
            // if the fins are not already saturated
            if (!elevator_saturated)
                pitch_integral += u(1)*dt;

            // Clamp the integral to ensure it does not exceed the limit
            pitch_integral = avl::clamp(pitch_integral,
                                       -pitch_integral_limit,
                                        pitch_integral_limit);

            u(2) = pitch_integral;

        }

        // If there is a yaw setpoint, set the yaw components of the input
        // vector u, and add to the yaw integral error. If there is no yaw
        // setpoint, the input vector will have zeros for the yaw components
        if (!std::isnan(yaw_setpoint))
        {

            u(3) = -yaw_rate_input;
            u(4) = avl::wrap(yaw_setpoint - yaw_input, false);

            // Add the yaw error times the iteration time to the integral term
            // if the fins are not already saturated
            if (!rudder_saturated)
                yaw_integral += u(4)*dt;

            // Clamp the integral to ensure it does not exceed the limit
            yaw_integral = avl::clamp(yaw_integral,
                                     -yaw_integral_limit,
                                      yaw_integral_limit);

            u(5) = yaw_integral;

        }

        log_data("[u] %.4f %.4f %.4f %.4f %.4f %.4f",
            u(0), u(1), u(2), u(3), u(4), u(5));

        // Get Output Vector  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Get the controller output values by iterating the state space model
        Eigen::VectorXd y = ss_model.iterate(u);

        log_data("[y] %.4f %.4f",
            y(0), y(1));

        // Choose the elevator output value based on which setpoints are active.
        // The an elevator setpoint is specified, it takes priority. If there
        // is no elevator setpoint but there is a pitch setpoint, use the
        // elevator output from the statespace model. Otherwise, set elevators
        // up using the value from the config file
        double elevator_output;
        if (!std::isnan(elevator_setpoint))
            elevator_output = elevator_setpoint;
        else if (!std::isnan(pitch_setpoint))
            elevator_output = avl::clamp(y(0), -max_fin_angle, max_fin_angle);
        else
            elevator_output = elevators_up_angle;

        // Choose the rudder output value based on which setpoints are active.
        // The an rudder setpoint is specified, it takes priority. If there
        // is no rudder setpoint but there is a yaw setpoint, use the
        // rudder output from the statespace model. Otherwise, set rudders
        // to zero
        double rudder_output;
        if (!std::isnan(rudder_setpoint))
            rudder_output = rudder_setpoint;
        else if (!std::isnan(yaw_setpoint))
            rudder_output = avl::clamp(y(1), -max_fin_angle, max_fin_angle);
        else
            rudder_output = 0.0;

        // Check for elevator and rudder saturation. If there is saturation,
        // the corresponding integral term does not need to be increased
        elevator_saturated = std::abs(elevator_output) >= max_fin_angle;
        rudder_saturated = std::abs(rudder_output) >= max_fin_angle;

        // Log the output data
        log_data("[output] %.2f %.2f",
            elevator_output, rudder_output);

        // Handle Fin Mixing ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Assume zero roll angle is zero unless fin mixing is enabled, in
        // which case the roll angle is the vehicle's measured roll angle
        // in radians
        double roll = 0.0;
        if(enable_fin_mixing)
            roll = roll_input;

        // Calculate the fin angles by mixing the roll, pitch, and yaw PID
        // outputs based on the vehicle's roll
        double sin_roll = sin(avl::deg_to_rad(roll));
        double cos_roll = cos(avl::deg_to_rad(roll));
        double port_angle      =  -elevator_output*cos_roll - rudder_output*sin_roll;
        double starboard_angle =   elevator_output*cos_roll + rudder_output*sin_roll;
        double top_angle       =   elevator_output*sin_roll - rudder_output*cos_roll;
        double bottom_angle    =  -elevator_output*sin_roll + rudder_output*cos_roll;

        // Publish Fin Commands ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Create the fins message and publish it
        FinsMsg fins_msg;
        fins_msg.port =      avl::deg_to_rad(port_angle);
        fins_msg.starboard = avl::deg_to_rad(starboard_angle);
        fins_msg.top =       avl::deg_to_rad(top_angle);
        fins_msg.bottom =    avl::deg_to_rad(bottom_angle);
        fins_pub.publish(fins_msg);

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

        // Reset the integral error terms
        pitch_integral = 0.0;
        yaw_integral = 0.0;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[u] pitch\\_rate pitch\\_error pitch\\_integral yaw\\_rate yaw\\_error yaw\\_integral");
        add_data_header("[u] deg/s deg deg deg/s deg deg");
        add_data_header("[y] elevator rudder");
        add_data_header("[y] deg deg");
        add_data_header("[attitude] pitch yaw pitch\\_setpoint yaw\\_setpoint");
        add_data_header("[attitude] deg deg deg deg");
        add_data_header("[output] elevator rudder");
        add_data_header("[output] deg deg");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<AhrsMsg>("device/ahrs", min_input_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/elevator", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/rudder", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/roll", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/pitch", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/yaw", min_setpoint_rate);

        // Read in state space model parameters from the config file
        int p = get_param<int>("~num_inputs");
        int n = get_param<int>("~num_states");
        int q = get_param<int>("~num_outputs");
        std::vector<double> A_vect  = get_param<std::vector<double>>("~A");
        std::vector<double> B_vect  = get_param<std::vector<double>>("~B");
        std::vector<double> C_vect  = get_param<std::vector<double>>("~C");
        std::vector<double> D_vect  = get_param<std::vector<double>>("~D");
        std::vector<double> x0_vect = get_param<std::vector<double>>("~x0");

        // Get parameters from the config file
        max_fin_angle = get_param<double>("~max_fin_angle");
        pitch_integral_limit = get_param<double>("~pitch_integral_limit");
        yaw_integral_limit = get_param<double>("~yaw_integral_limit");
        enable_fin_mixing = get_param<bool>("~enable_fin_mixing");
        dt = 1.0 / get_param<double>("~iteration_rate");
        elevators_up_angle = get_param<double>("~elevators_up_angle");

        // Convert the double vectors to eigen matrices and vectors
        using namespace Eigen;
        Eigen::MatrixXd A  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(&A_vect[0], n, n);
        Eigen::MatrixXd B  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(&B_vect[0], n, p);
        Eigen::MatrixXd C  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(&C_vect[0], q, n);
        Eigen::MatrixXd D  = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(&D_vect[0], q, p);
        Eigen::VectorXd x0 = Map<VectorXd>(&x0_vect[0], n);

        // Initialize the state space model with the parameters
        ss_model.initialize(A, B, C, D, x0);

        // Get the fin mixing flag from the config file
        enable_fin_mixing = get_param<bool>("~enable_fin_mixing");

        // Set up the output publisher
        fins_pub = node_handle->advertise<FinsMsg>(
            "device/fins", 1);

        // Set up the reset fins service client
        reset_fins_client = node_handle->serviceClient<std_srvs::Trigger>(
            "device/reset_fins");

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
    HinfAttitudeControlNode node(argc, argv);
    node.start();
    return 0;
}
