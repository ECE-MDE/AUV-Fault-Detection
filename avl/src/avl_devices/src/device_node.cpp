//==============================================================================
// Autonomous Vehicle Library
//
// Description: Device node base class. Handles broadcasting of DEVICE packets
//              when requested or at a specified rate by calling the
//              get_device_packet function that must be implemented by
//              a device node child class.
//==============================================================================

#include <avl_devices/device_node.h>

// Node base class
#include <avl_core/node.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>

// Comms functionality
#include <avl_comms/command_handler.h>
#include <avl_comms/comms.h>
using namespace avl;

// ROS message includes
#include <avl_msgs/CommsTxSrv.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        DeviceNode constructor
//------------------------------------------------------------------------------
DeviceNode::DeviceNode(int argc, char **argv) : Node(argc, argv)
{

    // Set up the comms TX client
    comms_tx_client = node_handle->serviceClient<CommsTxSrv>(
        "comms/comms_tx");

    // Bind the timer callback to specify FSD or BSD interface
    auto fsd_timer_cb = std::bind(&DeviceNode::device_packet_timer_callback,
        this, std::placeholders::_1, INTERFACE_FSD);
    auto bsd_timer_cb = std::bind(&DeviceNode::device_packet_timer_callback,
        this, std::placeholders::_1, INTERFACE_BSD);

    // Create the DEVICE packet timers (which also starts them), and stop
    // them in order to wait for an output rate to be set
    fsd_device_packet_timer = node_handle->createTimer(ros::Duration(1.0),
        fsd_timer_cb);
    fsd_device_packet_timer.stop();
    bsd_device_packet_timer = node_handle->createTimer(ros::Duration(1.0),
        bsd_timer_cb);
    bsd_device_packet_timer.stop();

    // Configure the command handler callback
    command_handler.set_callback(&DeviceNode::command_callback, this);

}

//------------------------------------------------------------------------------
// Name:        set_device_name
// Description: Sets the device name.
// Arguments:   - name: Device name.
//------------------------------------------------------------------------------
void DeviceNode::set_device_name(std::string name)
{
    device_name = name;
}

//------------------------------------------------------------------------------
// Name:        set_device_packet_output_rate
// Description: Sets the DEVICE packet output rate for both the specified
//              interface. This function cal be called from the device node
//              child class, but will be automatically overridden by a
//              COMMAND packets containing a DEVICE_REQUEST field.
// Arguments:   - rate: DEVICE packet output rate in Hz.
//------------------------------------------------------------------------------
void DeviceNode::set_device_packet_output_rate(double rate,
    CommsInterface interface)
{

    // Pick the correct timer based on the interface
    ros::Timer* timer = (interface == INTERFACE_FSD) ?
        &fsd_device_packet_timer : &bsd_device_packet_timer;

    // A rate of NAN is a request for a single device packet to be
    // transmitted
    if (std::isnan(rate))
    {
        transmit_device_packet(interface);
    }

    // A rate of zero disables the periodic device packet transmission
    else if (rate == 0.0)
    {
        timer->stop();
    }

    // Otherwise, set the specified rate and start the timer
    else
    {
        timer->setPeriod(ros::Duration(1.0/rate));
        timer->start();
    }

}

//------------------------------------------------------------------------------
// Name:        transmit_device_packet
// Description: Calls the user-defined get_device_packet function to get a
//              device packet and then transmits it to the specified
//              interface.
// Arguments:   - interface: Interface to transmit DEVICE packet to.
//------------------------------------------------------------------------------
void DeviceNode::transmit_device_packet(CommsInterface interface)
{

    // Create a DEVICE packet from the device name and the parameter list
    // returned by the child class

    avl::PacketHeader header;
    header.timestamp = avl::get_epoch_time_nanoseconds();
    header.timeout = 0;
    header.source_id = avl::get_vehicle_id();
    header.destination_id = 0;

    avl::DeviceInfo device_info;
    device_info.name = device_name;
    device_info.parameters = get_device_parameters();

    avl::DevicePacket device_packet(header, device_info);

    // Transmit the DEVICE packet to the FSD interface
    CommsTxSrv srv;
    srv.request.channel = CHANNEL_ETHERNET;
    srv.request.interface = interface;
    srv.request.data = device_packet.get_bytes();
    comms_tx_client.call(srv);

}

//------------------------------------------------------------------------------
// Name:        command_callback
// Description: Called when a COMMAND packet is received by the communication
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
//------------------------------------------------------------------------------
bool DeviceNode::command_callback(CommsChannel channel,
    CommsInterface interface, std::string command_name, ParameterList params,
    bool& result, std::vector<uint8_t>& data)
{

    // Handle DEVICE commands
    if (command_name == "DEVICE")
    {

        // Get the requested device name and output rate
        std::string name = params.get("NAME").to_string();
        double rate = params.get("RATE").to_double();

        // If every device is being requested or if the requested device
        // name matches this device's name, configure the timer
        // corresponding to the interface from which the command was
        // received
        if (name == "all" || name == "ALL" || name == device_name)
        {

            // Extract the DEVICE packet output rate and configure the node
            // to output at that rate
            set_device_packet_output_rate(rate, interface);

            result = true;
            return true;

        }

    }

    return false;

}

//------------------------------------------------------------------------------
// Name:        device_packet_timer_callback
// Description: Called at a configured rate to output DEVICE packets from
//              the user-defined get_device_packet function to the FSD
//              or BSD interface. Calls the user-defined get_device_packet
//              function.
// Arguments:   - event: ROS timer event structure.
//              - interface: Interface to transmit DEVICE packet to.
//------------------------------------------------------------------------------
void DeviceNode::device_packet_timer_callback(const ros::TimerEvent& event,
    CommsInterface interface)
{
    transmit_device_packet(interface);
}
