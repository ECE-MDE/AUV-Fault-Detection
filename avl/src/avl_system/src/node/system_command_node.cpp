//==============================================================================
// Autonomous Vehicle Library
//
// Description: The system command node handles any COMMAND commands that deal
//              with system level actions.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/motor (std_msgs/Float64)
//              device/fins (avl_devices/FinsMsg)
//
// Subscribers: system/mission_status (avl_msgs/MissionStatusMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/math.h>

// Command handler class
#include <avl_comms/command_handler.h>
using namespace avl;

// ROS Messages
#include <std_msgs/Float64.h>
#include <avl_msgs/MissionStatusMsg.h>
#include <avl_msgs/FinsMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class SystemCommandNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        SystemCommandNode constructor
    //--------------------------------------------------------------------------
    SystemCommandNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

    // Publishers for actuator control
    ros::Publisher motor_pub;
    ros::Publisher fins_pub;

    // Subscriber for mode messages
    ros::Subscriber mission_status_sub;
    MissionMode mode;

    // Manual control config
    double max_throttle;
    double max_rudder_angle;

private:

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when a COMMAND packet is received by the
    //              communication architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - command_name: Command name of the received command.
    //              - params: Command parameter list.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool command_callback(CommsChannel channel, CommsInterface interface,
        std::string command_name, ParameterList params,
        bool& result, std::vector<uint8_t>& data)
    {

        // Handle PING commands
        if (command_name == "PING")
        {
            log_debug("handling PING command");
            result = true;
            return true;
        }

        // Handle JOY commands
        if (command_name == "JOY")
        {
            if (mode == MISSION_MODE_MANUAL)
            {

                // Get the command params
                double left_x  = params.get("LEFT X").to_double();
                double left_y  = params.get("LEFT Y").to_double();
                double right_x = params.get("RIGHT X").to_double();
                double right_y = params.get("RIGHT Y").to_double();

                // Calculate throttle and rudder from joystick positions
                double throttle = max_throttle * -left_y;
                double rudder = max_rudder_angle * -right_x;
                double elevator = avl::deg_to_rad(-20.0);

                // Format and publish the motor message
                std_msgs::Float64 motor_msg;
                motor_msg.data = throttle;
                motor_pub.publish(motor_msg);

                // Format and publish the fins message
                FinsMsg fins_msg;
                fins_msg.top =       -rudder;
                fins_msg.bottom =     rudder;
                fins_msg.port =      -elevator;
                fins_msg.starboard =  elevator;
                fins_pub.publish(fins_msg);

                log_data("[joy] %f %f %f %f %f %f",
                    left_x, left_y, right_x, right_y, throttle, rudder);

            }
            return false;
        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        mission_status_msg_callback
    // Description: Called when a mission status message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void mission_status_msg_callback(const MissionStatusMsg& message)
    {
        mode = static_cast<MissionMode>(message.mode);
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data headers
        add_data_header("[joy] left_x left_y right_x right_y throttle rudder elevator");
        add_data_header("[joy] %% %% %% %% %% rad rad");

        // Get config file parameters
        max_throttle =  get_param<double>("~max_throttle");
        max_rudder_angle = avl::deg_to_rad(get_param<double>("~max_rudder_angle"));

        // Set up the actuator publishers
        motor_pub = node_handle->advertise<std_msgs::Float64>(
            "device/motor", 1);
        fins_pub = node_handle->advertise<FinsMsg>(
            "device/fins", 1);

        // Set up the mission status subscriber
        mission_status_sub = node_handle->subscribe(
            "system/mission_status", 1,
            &SystemCommandNode::mission_status_msg_callback, this);

        // Set the command handler's callback
        command_handler.set_callback(
            &SystemCommandNode::command_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================

// Name and initialize the node
int main(int argc, char **argv)
{
    SystemCommandNode node(argc, argv);
    node.start();
    return 0;
}
