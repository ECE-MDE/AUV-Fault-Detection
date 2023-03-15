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
//              bool packet_callback(Packet packet, bool& result,
//                  std::vector<uint8_t>& data);
//
//              The command callback should handle the command and return true
//              if the command was handled (even if it failed) and false if it
//              was not. If the command was handled (even if it failed in some
//              way), the function should set the result boolean to reflect the
//              result of the packet handling and, in the case of a failure,
//              set a failure message as the response data.
//==============================================================================

#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

// ROS functions and messages
#include "ros/ros.h"
#include <avl_msgs/CommsTxSrv.h>
#include <avl_msgs/CommsRxMsg.h>
using namespace avl_msgs;

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>

// AVL util functions
#include <avl_core/util/misc.h>

// Comms enums
#include <avl_comms/comms.h>
using namespace avl;

// Typedef for a vector of bytes
typedef std::vector<uint8_t> bytes_t;

// Typedef for a packet calllback function
typedef std::function<bool(avl::CommsChannel, avl::CommsInterface,
    avl::Packet, bool&, bytes_t&)> callback_t;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class PacketHandler
{

public:

    //--------------------------------------------------------------------------
    // Name:        PacketHandler constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    PacketHandler();

    //--------------------------------------------------------------------------
    // Name:        PacketHandler destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~PacketHandler();

    //--------------------------------------------------------------------------
    // Name:        set_callback
    // Description: Sets a packet callback function to be called when a packet
    //              is received through the communications architecture.
    // Arguments:   - callback: Packet callback function.
    //--------------------------------------------------------------------------
    void set_callback(callback_t callback);

    //--------------------------------------------------------------------------
    // Name:        set_callback
    // Description: Sets a packet callback function to be called when a packet
    //              is received through the communications architecture. Used
    //              for class member functions
    // Arguments:   - callback: Packet callback function.
    //--------------------------------------------------------------------------
    template<class T>
    void set_callback(bool (T::*callback)(CommsChannel, CommsInterface, Packet,
        bool&, bytes_t&), T* obj)
    {
        auto bound_callback = std::bind(callback, obj,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            std::placeholders::_4,
            std::placeholders::_5);
        set_callback(bound_callback);
    }

private:

    // Node handle for publishing and subscribing
    ros::NodeHandle node_handle;

    // Client and subscriber for receiving packets and sending responses
    // through the communication architecture
    ros::Subscriber comms_rx_sub;
    ros::ServiceClient comms_tx_client;

    // User defined packet callback function
    callback_t packet_callback = nullptr;

private:

    //--------------------------------------------------------------------------
    // Name:        comms_rx_msg_callback
    // Description: Called when a packet is received on the comms_rx topic.
    //              Passes the packet to the user defined command callback.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void comms_rx_msg_callback(const CommsRxMsg& message);

};

#endif // PACKET_HANDLER_H
