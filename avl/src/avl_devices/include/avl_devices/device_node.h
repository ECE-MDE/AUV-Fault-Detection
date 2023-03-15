//==============================================================================
// Autonomous Vehicle Library
//
// Description:
//==============================================================================

#ifndef DEVICE_NODE_H
#define DEVICE_NODE_H

// Node base class
#include <avl_core/node.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>

// Comms functionality
#include <avl_comms/command_handler.h>
#include <avl_comms/comms.h>
using namespace avl;

//==============================================================================
//                              NODE DECLARATION
//==============================================================================

class DeviceNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        DeviceNode constructor
    //--------------------------------------------------------------------------
    DeviceNode(int argc, char **argv);

    //--------------------------------------------------------------------------
    // Name:        set_device_name
    // Description: Sets the device name.
    // Arguments:   - name: Device name.
    //--------------------------------------------------------------------------
    void set_device_name(std::string name);

    //--------------------------------------------------------------------------
    // Name:        set_device_packet_output_rate
    // Description: Sets the DEVICE packet output rate for both the specified
    //              interface. This function cal be called from the device node
    //              child class, but will be automatically overridden by a
    //              COMMAND packets containing a DEVICE_REQUEST field.
    // Arguments:   - rate: DEVICE packet output rate in Hz.
    //--------------------------------------------------------------------------
    void set_device_packet_output_rate(double rate, CommsInterface interface);

    //--------------------------------------------------------------------------
    // Name:        get_device_parameters
    // Description: Called when a DEVICE packet is requested from the node or
    //              when a DEVICE packet is needed to be published to the FSD
    //              or BSD interface. This function should be implemented by the
    //              device child class to create and return a parameter list.
    // Returns:     Parameter list containing parameters to be added to the
    //              device packet that will be transmitted.
    //--------------------------------------------------------------------------
    virtual avl::ParameterList get_device_parameters() = 0;

private:

    // String representing the device name.
    std::string device_name = "unnamed";

    // Timers used to periodically request and transmit DEVICE packets when a
    // DEVICE packet output rate is set. The FSD and BSD interfaces can
    // configure different rates
    ros::Timer fsd_device_packet_timer;
    ros::Timer bsd_device_packet_timer;

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

    // Client for transmitting DEVICE packets through the communication
    // architecture
    ros::ServiceClient comms_tx_client;

private:

    //--------------------------------------------------------------------------
    // Name:        transmit_device_packet
    // Description: Calls the user-defined get_device_packet function to get a
    //              device packet and then transmits it to the specified
    //              interface.
    // Arguments:   - interface: Interface to transmit DEVICE packet to.
    //--------------------------------------------------------------------------
    void transmit_device_packet(CommsInterface interface);

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when an AVL packet is received by the communication
    //              architecture.
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
        bool& result, std::vector<uint8_t>& data);

    //--------------------------------------------------------------------------
    // Name:        device_packet_timer_callback
    // Description: Called at a configured rate to output DEVICE packets from
    //              the user-defined get_device_packet function to the FSD
    //              or BSD interface. Calls the user-defined get_device_packet
    //              function.
    // Arguments:   - event: ROS timer event structure.
    //              - interface: Interface to transmit DEVICE packet to.
    //--------------------------------------------------------------------------
    void device_packet_timer_callback(const ros::TimerEvent& event,
        CommsInterface interface);

};

#endif //DEVICE_NODE_H
