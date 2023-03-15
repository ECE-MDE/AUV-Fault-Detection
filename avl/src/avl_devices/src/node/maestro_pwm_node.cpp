//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node for PWM output control using the Polulu Maestro PWM
//              controller. Listens for PWM service requests to the pwm service
//              server and sets the corresponding PWM on the Maestro PWM
//              channel. The Maestro device settings will be reset and set to
//              values that make this node work every time the node is run.
//              There is no need to configure the Maestro device before use.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: device/pwm (avl_msgs/PwmMsg)
//==============================================================================

// Core utility
#include <avl_core/node.h>

// ROS message includes
#include <avl_msgs/PwmMsg.h>
using namespace avl_msgs;

// Maestro USB Device driver
#include <avl_devices/driver/maestro_pwm.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class MaestroPwmNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        MaestroPwmNode constructor
    //--------------------------------------------------------------------------
    MaestroPwmNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Maestro servo controller instance used by node
    MaestroPwm maestro;

    // Subscriber for PWM messages
    ros::Subscriber pwm_sub;

    // Total number of Maestro PWM channels
    const uint8_t num_channels = 6;

private:

    //--------------------------------------------------------------------------
    // Name:        pwm_msg_callback
    //
    // Description: Called when a PWM message is received on the pwm topic.
    //              Sets output pulse width value on the Pololu Maestro PWM
    //              control board.
    //
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void pwm_msg_callback(const PwmMsg& message)
    {

        try
        {
            // Clear any errors and enable PWM output in case a timeout error
            // has disabled them
            maestro.clear_errors();
        }
        catch (const std::exception& ex)
        {
            log_warning(std::string("clear errors failed with error message: (") + ex.what() + ").");
        }

        try
        {
            std::vector<double> pulse_width = message.pulse_width;
            for (size_t i = 0; i < pulse_width.size(); i++)
            {
                if (!std::isnan(pulse_width.at(i)))
                    maestro.set_pulse_width(i, pulse_width.at(i));
            }

        }
        catch (const std::exception& ex)
        {
            log_warning(std::string("set pulse width failed with error message: (") + ex.what() + ").");
        }

        // Log the channel and pulse width to data
        log_data("[pwm] %.4f %.4f %.4f %.4f %.4f %.4f",
            message.pulse_width[0], message.pulse_width[1],
            message.pulse_width[2], message.pulse_width[3],
            message.pulse_width[4], message.pulse_width[5]);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    //
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Connect to the Maestro PWM controller and reset its settings
        maestro.open();

        try
        {
            maestro.reset_settings();
        }
        catch (const std::exception& ex)
        {
            log_warning(std::string("reset settings failed with error message: (") + ex.what() + ").");
        }

        try
        {
            // Change to dual USB port communications mode after the reset since
            // it is not the default
            maestro.enable_dual_usb_mode();
        }
        catch (const std::exception& ex)
        {
            log_warning(std::string("Enable dual usb mode failed with error message: (") + ex.what() + ").");
        }

        // Set the PWM output timeout
        maestro.set_output_timeout(get_param<int>("~pwm_timeout"));

        // Disable the PWM outputs
        maestro.disable_output();

        // Configure the speed and acceleration for each PWM channel
        for (int i = 0; i < num_channels; i++)
        {
            try
            {
                maestro.set_speed(i, get_param<int>(std::string("~channel") + std::to_string(i) + "/speed"));
                maestro.set_acceleration(i, get_param<int>(std::string("~channel") + std::to_string(i) + "/acceleration"));
            }
            catch (const std::exception& ex)
            {
                log_warning(std::string("set speed/acceleration failed with error message: (") + ex.what() + ").");
            }
        }

        // Print the header for the data log
        log_data("[pwm] ch0 ch1 ch2 ch3 ch4 ch5");
        log_data("[pwm] ms ms ms ms ms ms");

        // Set up the PWM message subscriber
        pwm_sub = node_handle->subscribe("device/pwm", 16, &MaestroPwmNode::pwm_msg_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    //
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    //
    // Description: Called when the node finishes running or is stopped.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        maestro.disable_output();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    MaestroPwmNode node(argc, argv);
    node.start();
    return 0;
}
