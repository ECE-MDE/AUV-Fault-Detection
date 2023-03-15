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

// PacketHandler base class
#include <avl_comms/command_handler.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        CommandHandler constructor
// Description: Default constructor.
//------------------------------------------------------------------------------
CommandHandler::CommandHandler()
{
    PacketHandler::set_callback(&CommandHandler::packet_callback, this);
}

//------------------------------------------------------------------------------
// Name:        CommandHandler destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
CommandHandler::~CommandHandler()
{

}

//------------------------------------------------------------------------------
// Name:        set_callback
// Description: Sets a packet callback function to be called when a packet
//              is received through the communications architecture.
// Arguments:   - callback: Packet callback function.
//------------------------------------------------------------------------------
void CommandHandler::set_callback(command_callback_t callback)
{
    command_callback = callback;
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
bool CommandHandler::packet_callback(CommsChannel channel,
    CommsInterface interface, Packet packet, bool& result,
    std::vector<uint8_t>& data)
{

    // Do nothing if a command callback has not been set
    if (command_callback != nullptr)
    {

        // Check if the packet is a COMMAND packet
        if (packet.get_descriptor() == COMMAND_PACKET)
        {

            // Create the command packet and call the command callback with it
            CommandPacket command_packet(packet);
            Command command = command_packet.get_command();
            return command_callback(
                channel,
                interface,
                command.name,
                command.parameters,
                result,
                data);

        }

    }

    return false;

}
