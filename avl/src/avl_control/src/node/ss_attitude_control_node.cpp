//==============================================================================
// Autonomous Vehicle Library
//
// Description: Control node for control of vehicle pitch and yaw using a state
//              space controller.
//
//              Roll, pitch, and yaw can be commanded separately with a separate
//              setpoint. The node also accepts elevator and rudder setpoints
//              that will take prescedence over fin angles generated from
//              attitude setpoints.
//
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
// Subscribers: nav/inertial_nav (avl_msgs/NavigationMsg)
//              device/imu (avl_msgs/ImuMsg)
//              setpoint/elevator (avl_msgs/Float64SetpointMsg)
//              setpoint/rudder (avl_msgs/Float64SetpointMsg)
//              setpoint/roll (avl_msgs/Float64SetpointMsg)
//              setpoint/pitch (avl_msgs/Float64SetpointMsg)
//              setpoint/yaw (avl_msgs/Float64SetpointMsg)
//==============================================================================

// Control node base class
#include <avl_control/control_node.h>

// State space model
#include <avl_control/algorithm/state_space_model.h>

// Util functions
#include <avl_core/util/math.h>

// ROS messages
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/ImuMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/FinsMsg.h>
#include <std_srvs/Trigger.h>
using namespace avl_msgs;

// Typedef for a vector of doubles
typedef std::vector<double> doubles_t;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class StateSpaceAttitudeControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        RpmControlNode constructor
    //--------------------------------------------------------------------------
    StateSpaceAttitudeControlNode(int argc, char **argv) : ControlNode(argc, argv)
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

    // Clamp the error input to the controller to +/- this value to maintain linearity
    double max_input_error;

private:

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on each controller iteration. Controller iteration
    //              logic and output publishing should be implemented here.
    //--------------------------------------------------------------------------
    void iterate()
    {

        // Get the input and setpoints
        NavigationMsg nav_input = get_input<NavigationMsg>("nav/inertial_nav");
        ImuMsg imu_input = get_input<ImuMsg>("device/imu");
        double roll_input =  nav_input.roll;
        double pitch_input = nav_input.pitch;
        double yaw_input =   nav_input.yaw;
        double pitch_rate =  imu_input.angular_velocity.y;
        double yaw_rate =    imu_input.angular_velocity.z;
        double elevator_setpoint = get_float64_setpoint("setpoint/elevator");
        double rudder_setpoint =   get_float64_setpoint("setpoint/rudder");
        double roll_setpoint =     get_float64_setpoint("setpoint/roll");
        double pitch_setpoint =    get_float64_setpoint("setpoint/pitch");
        double yaw_setpoint =      get_float64_setpoint("setpoint/yaw");

        log_data("[ss] %.4f %.4f %.4f %.4f %.4f %.4f",
            roll_setpoint, roll_input,
            pitch_setpoint, pitch_input,
            yaw_setpoint, yaw_input);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Calculate error

        double roll_error =  avl::wrap(roll_setpoint  - roll_input);
        double pitch_error = avl::wrap(pitch_setpoint - pitch_input);
        double yaw_error =   avl::wrap(yaw_setpoint   - yaw_input);

        // Limit error contribution for linearity
        roll_error = avl::clamp(roll_error, -max_input_error, max_input_error);
        pitch_error = avl::clamp(pitch_error, -max_input_error, max_input_error);
        yaw_error = avl::clamp(yaw_error, -max_input_error, max_input_error);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Create Input Vector

        // Input vector is the following 4x1 vector:
        // u = [pitch_error; pitch_rate;
        //      yaw_error;   yaw_rate]
        VectorXd u = VectorXd::Zero(4);

        // Set only the components of the input vector that have setpoints. If
        // there is no setpoint for an axis, its input elements will be zero

        if (!std::isnan(pitch_setpoint))
        {
            u(0) = pitch_error;
            u(1) = pitch_rate;
        }

        if (!std::isnan(yaw_setpoint))
        {
            u(2) = yaw_error;
            u(3) = yaw_rate;
        }

        // Log the input
        log_data("[u] %.4f %.4f %.4f %.4f",
            u(0), u(1), u(2), u(3));

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Get Output Vector

        // Get the controller output by iterating the state space model
        // y = [elevator_output; rudder_output]
        Eigen::VectorXd y = ss_model.iterate(u);

        log_data("[y] %.4f %.4f",
            y(0), y(1));

        // Choose the elevator output value based on which setpoints are active.
        // The an elevator setpoint is specified, it takes priority. If there
        // is no elevator setpoint but there is a pitch setpoint, use the
        // elevator output from the statespace model. Otherwise, set elevator
        // to the default elevator angle from the config file
        double elevator_output = avl::deg_to_rad(
            get_param<double>("~default_elevator_angle"));
        if (!std::isnan(elevator_setpoint))
            elevator_output = elevator_setpoint;
        else if (!std::isnan(pitch_setpoint))
            elevator_output = y(0);

        // Choose the rudder output value based on which setpoints are active.
        // The an rudder setpoint is specified, it takes priority. If there
        // is no rudder setpoint but there is a yaw setpoint, use the
        // rudder output from the statespace model. Otherwise, set rudder
        // to the default elevator angle from the config file
        double rudder_output = avl::deg_to_rad(
            get_param<double>("~default_rudder_angle"));
        if (!std::isnan(rudder_setpoint))
            rudder_output = rudder_setpoint;
        else if (!std::isnan(yaw_setpoint))
            rudder_output = -1.0 * y(1);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Handle Fin Mixing

        // Assume zero roll angle is zero unless fin mixing is enabled, in
        // which case the roll angle is the vehicle's measured roll angle
        // in radians
        double roll = 0.0;
        if(enable_fin_mixing)
            roll = roll_input;

        // Calculate the fin angles by mixing the roll, pitch, and yaw PID
        // outputs based on the vehicle's roll
        double roll_output = 0.0;
        double sin_roll = sin(roll);
        double cos_roll = cos(roll);
        double port_angle      =  elevator_output * cos_roll + rudder_output   * sin_roll + roll_output;
        double starboard_angle = -elevator_output * cos_roll - rudder_output   * sin_roll + roll_output;
        double top_angle       =  rudder_output   * cos_roll + elevator_output * sin_roll + roll_output;
        double bottom_angle    = -rudder_output   * cos_roll - elevator_output * sin_roll + roll_output;

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Publish Fin Commands

        // Create the fins message and publish it
        FinsMsg fins_msg;
        fins_msg.port =      port_angle;
        fins_msg.starboard = starboard_angle;
        fins_msg.top =       top_angle;
        fins_msg.bottom =    bottom_angle;
        fins_pub.publish(fins_msg);

        log_data("[fins] %.4f %.4f %.4f %.4f",
            port_angle, starboard_angle, top_angle, bottom_angle);

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

        // Call the fin reset service to return fins to their home positions
        std_srvs::Trigger empty_service_msg;
        reset_fins_client.call(empty_service_msg);

        // Log NAN data entries to split up data plots between subsequent
        // control actions
        log_data("[ss] %.4f %.4f %.4f %.4f %.4f %.4f",
            NAN, NAN, NAN, NAN, NAN, NAN);
        log_data("[u] %.4f %.4f %.4f %.4f",
            NAN, NAN, NAN, NAN);
        log_data("[y] %.4f %.4f", NAN, NAN);
        log_data("[x] %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
            NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
        log_data("[fins] %.4f %.4f %.4f %.4f", NAN, NAN, NAN, NAN);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[ss] roll\\_setpoint roll\\_input "
                             "pitch\\_setpoint pitch\\_input "
                             "yaw\\_setpoint yaw\\_input");
        add_data_header("[ss] rad rad rad rad rad rad");
        add_data_header("[u] pitch\\_error pitch\\_rate "
                            "yaw\\_error yaw\\_rate");
        add_data_header("[u] rad rad/s rad rad/s");
        add_data_header("[y] elevator\\_output rudder\\_output");
        add_data_header("[y] rad rad");
        add_data_header("[x] 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20");
        add_data_header("[x] ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ?");
        add_data_header("[fins] port starboard top bottom");
        add_data_header("[fins] rad rad rad rad");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");
        add_input<NavigationMsg>("nav/inertial_nav", min_input_rate);
        add_input<ImuMsg>("device/imu", min_input_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/elevator", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/rudder", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/roll", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/pitch", min_setpoint_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/yaw", min_setpoint_rate);

        // Set up the fins publisher
        fins_pub = node_handle->advertise<FinsMsg>(
            "device/fins", 1);

        // Set up the reset fins service client
        reset_fins_client = node_handle->serviceClient<std_srvs::Trigger>(
            "device/reset_fins");

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

        // Set output limits
        double max_fin_angle = avl::deg_to_rad(get_param<double>("~max_fin_angle"));
        ss_model.add_output_limit(0, -max_fin_angle, max_fin_angle);
        ss_model.add_output_limit(1, -max_fin_angle, max_fin_angle);

        max_input_error = avl::deg_to_rad(get_param<double>("~max_input_error"));
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
    StateSpaceAttitudeControlNode node(argc, argv);
    node.start();
    return 0;
}
