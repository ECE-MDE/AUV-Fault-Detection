//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides a simple class for nodes to use to handle incoming
//              commands packets conforming to the AVL binary command protocol
//              in a decentralized manner. Command packets received are passed
//              to a user defined callback with the following signature:
//
//              bool packet_callback(avl::CommsChannel, avl::CommsInterface,
//                  std::string name, ParameterList params, bool&, bytes_t&);
//
//              The command callback should handle the command and return true
//              if the command was handled (even if it failed) and false if it
//              was not. If the command was handled (even if it failed in some
//              way), the function should set the result boolean to reflect the
//              result of the packet handling and, in the case of a failure,
//              set a failure message as the response data.
//==============================================================================

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

// PacketHandler base class
#include <avl_comms/packet_handler.h>

// Typedef for a packet calllback function
typedef std::function<bool(avl::CommsChannel, avl::CommsInterface,
    std::string name, ParameterList params, bool&, bytes_t&)>
    command_callback_t;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class CommandHandler : public PacketHandler
{

public:

    //--------------------------------------------------------------------------
    // Name:        CommandHandler constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    CommandHandler();

    //--------------------------------------------------------------------------
    // Name:        CommandHandler destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~CommandHandler();

    //--------------------------------------------------------------------------
    // Name:        set_callback
    // Description: Sets a packet callback function to be called when a packet
    //              is received through the communications architecture.
    // Arguments:   - callback: Packet callback function.
    //--------------------------------------------------------------------------
    void set_callback(command_callback_t callback);

    //--------------------------------------------------------------------------
    // Name:        set_callback
    // Description: Sets the command packet callback function to be called when
    //              a command packet is received through the communications
    //              architecture. Used for class member functions
    // Arguments:   - callback: Command packet callback function.
    //--------------------------------------------------------------------------
    template<class T>
    void set_callback(bool (T::*callback)(CommsChannel, CommsInterface,
        std::string, ParameterList, bool&, bytes_t&), T* obj)
    {
        auto bound_callback = std::bind(callback, obj,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            std::placeholders::_4,
            std::placeholders::_5,
            std::placeholders::_6);
        set_callback(bound_callback);
    }

private:

    // User defined command packet callback function
    command_callback_t command_callback = nullptr;

private:

    //--------------------------------------------------------------------------
    // Name:        packet_callback
    // Description: Called when an AVL packet is received by the communication
    //              architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - packet: Received packet.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool packet_callback(CommsChannel channel, CommsInterface interface,
        Packet packet, bool& result, std::vector<uint8_t>& data);

};

#endif // COMMAND_HANDLER_H
