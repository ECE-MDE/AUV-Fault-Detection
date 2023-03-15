//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides a simple class for nodes to use to handle incoming
//              packets conforming to the AVL binary command protocol in a
//              decentralized manner. The class sets up a subscriber for
//              incoming packets and a publisher for response packets.
//              Packets received on the topic are passed to a user defined
//              packet callback with the following signature:
//
//              bool packet_callback(AvlPacket command, bool result,
//                  std::vector<uint8_t> data);
//
//              The command callback should handle the command and return true
//              if the command was handled (even if it failed) and false if it
//              was not. If the command was handled (even if it failed in some
//              way), the function should set the result boolean to reflect the
//              result of the packet handling and, in the case of a failure,
//              set a failure message as the response data.
//==============================================================================

#include <avl_comms/packet_handler.h>

// ROS functions and messages
#include "ros/ros.h"
#include <avl_msgs/CommsTxSrv.h>
#include <avl_msgs/CommsRxMsg.h>
using namespace avl_msgs;

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

// Util functions
#include <avl_core/util/ros.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        PacketHandler constructor
// Description: Default constructor.
//------------------------------------------------------------------------------
PacketHandler::PacketHandler()
{

}

//------------------------------------------------------------------------------
// Name:        PacketHandler destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
PacketHandler::~PacketHandler()
{

}

//------------------------------------------------------------------------------
// Name:        set_callback
// Description: Sets a packet callback function to be called when a packet
//              is received through the communications architecture.
// Arguments:   - callback: Packet callback function.
//------------------------------------------------------------------------------
void PacketHandler::set_callback(callback_t callback)
{

    // Set up the command and command response subscriber and publisher
    comms_rx_sub = node_handle.subscribe("comms/comms_rx", 32,
        &PacketHandler::comms_rx_msg_callback, this);
    comms_tx_client =
        node_handle.serviceClient<CommsTxSrv>("comms/comms_tx");

    // Set the packet handler function
    packet_callback = callback;

}

//------------------------------------------------------------------------------
// Name:        comms_rx_msg_callback
// Description: Called when a packet is received on the comms_rx topic.
//              Passes the packet to the user defined command callback.
// Arguments:   - message: Message received on the topic.
//------------------------------------------------------------------------------
void PacketHandler::comms_rx_msg_callback(const CommsRxMsg& message)
{

    // If a packet callback was specified, pass the packet to it and get its
    // response. If it was responded to, publish a corresponding RESPONSE
    // packet
    if (packet_callback != nullptr)
    {

        // Turn the received bytes into a packet
        Packet received_packet(message.data);
        PacketHeader received_header = received_packet.get_header();

        // Pass the received packet to the user-defined packet callback
        bool response_result;
        std::vector<uint8_t> response_data;
        CommsChannel channel = static_cast<CommsChannel>(message.channel);
        CommsInterface interface =
            static_cast<CommsInterface>(message.interface);

        bool packet_handled = packet_callback(
            channel,
            interface,
            received_packet,
            response_result,
            response_data);

        // If the packet was handled in the callback, format and transmit the
        // response packet
        if (packet_handled)
        {

            // Format a RESPONSE packet
            avl::PacketHeader header;
            header.timestamp = avl::get_epoch_time_nanoseconds();
            header.timeout = 0;
            header.source_id = avl::get_vehicle_id();
            header.destination_id = received_packet.get_header().source_id;

            avl::Response response;
            response.source = received_header.timestamp;
            response.result = response_result;
            response.data = response_data;
            avl::ResponsePacket response_packet(header, response);

            // Call the comms transmit service to transmit the response
            CommsTxSrv srv;
            srv.request.channel = message.channel;
            srv.request.interface = message.interface;
            srv.request.data = response_packet.get_bytes();
            comms_tx_client.call(srv);

        }

    }

}
