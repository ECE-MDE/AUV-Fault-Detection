//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to publish joystick axis and button states.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/joystick (sensor_msgs/Joy)
//
// Subscribers: None
//==============================================================================

// Core utility
#include <avl_core/node.h>

// ROS message includes
#include <sensor_msgs/Joy.h>

// Joystick driver
#include <avl_devices/driver/joystick.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class JoystickNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        JoystickNode constructor
    //--------------------------------------------------------------------------
    JoystickNode(int argc, char **argv) : Node(argc, argv) {}

private:

    // Joystick instance
    Joystick joy;
    int joystick_num;

    // Publisher for joystick messages
    ros::Publisher joy_pub;
    sensor_msgs::Joy joy_msg;

    // Timer for publishing joystick messages at the minimum rate
    ros::Timer min_rate_timer;
    double min_output_rate;
    double max_output_rate;

private:

    //--------------------------------------------------------------------------
    // Name:        connect_to_joystick
    // Description: Loops until the joystick is successfully connected to.
    //--------------------------------------------------------------------------
    void connect_to_joystick()
    {

        log_info("connecting to joystick... (/dev/input/js%d)",
            joystick_num);

        // Loop until connection success
        do
        {
            try
            {
                joy.connect(joystick_num);
            }
            catch (const std::exception& ex)
            {
                ros::Duration(0.1).sleep();
            }
        } while (ros::ok() && !joy.is_open());

        log_info("connected to joystick (/dev/input/js%d)",
            joystick_num);
        log_info("number of axes:    %d", joy.get_num_axes());
        log_info("number of buttons: %d", joy.get_num_buttons());

    }

    //--------------------------------------------------------------------------
    // Name:        joystick_state_callback
    // Description: Called periodically by the joystick driver with the most
    //              recent joystick axis and button states. This function is
    //              called at the specified joystick state callback rate or
    //              immediately when a button event occurs to prevent missed
    //              button events.
    // Arguments:   - axis_values: most recent joystick axis values
    //              - button_values: most recent joystick button values
    //--------------------------------------------------------------------------
    void joystick_state_callback(std::vector<float> axis_values,
                                 std::vector<int> button_values)
    {

        // Create and publish a joystick message
        joy_msg.axes = axis_values;
        joy_msg.buttons = button_values;
        joy_pub.publish(joy_msg);

        // Format the axis values as a space delimited string
        std::stringstream ss;
        ss.precision(2);
        for (auto i: axis_values)
            ss << std::fixed << i << " ";

        // Format the button values as a space delimited string
        ss.str(std::string());
        for (auto i: button_values)
            ss << i << " ";

        // Log the axis and button values
        log_data("[axis] " + ss.str());
        log_data("[button] " + ss.str());

    }

    //--------------------------------------------------------------------------
    // Name:        timer_callback
    // Description: Called when the timer triggers.
    // Arguments:   - event: ROS timer event.
    //--------------------------------------------------------------------------
    void timer_callback(const ros::TimerEvent& event)
    {
        joy_pub.publish(joy_msg);
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get config file parameters
        joystick_num = get_param<int>("~joystick_num");
        min_output_rate = get_param<double>("~min_output_rate");
        max_output_rate = get_param<double>("~max_output_rate");

        // Set up the publisher for joystick messages
        joy_pub = node_handle->advertise<sensor_msgs::Joy>("device/joystick", 1);

        // Configure the joystick
        joy.set_deadzone(get_param<float>("~axis_deadzone"));
        joy.set_state_callback(&JoystickNode::joystick_state_callback, this);

        // Attempt to connect to the joystick
        connect_to_joystick();

        min_rate_timer = node_handle->createTimer(
            ros::Rate(min_output_rate), &JoystickNode::timer_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::Rate spin_rate(max_output_rate);
        while (ros::ok())
        {

            try
            {
                joy.spin_once();
            }
            catch (const std::exception& ex)
            {

                // If we the joy stick fails to read events, assume it has been
                // disconnected. Try reconnecting
                log_warning("joystick disconnected (/dev/input/js%d)",
                    joystick_num);
                connect_to_joystick();

            }

            ros::spinOnce();
            spin_rate.sleep();

        }
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    JoystickNode node(argc, argv);
    node.start();
    return 0;
}
