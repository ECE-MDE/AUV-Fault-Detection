//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a COMMAND packet conforming to the AVL binary packet
//              protocol, providing functions to construct them and get
//              the information described by their fields.
//==============================================================================

#include "protocol/command_packet.h"
#include <iostream>

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        CommandPacket constructor
// Description: Constructs a COMMAND packet from a base packet class.
// Arguments:   - packet: Packet to construct the COMMAND packet from.
//------------------------------------------------------------------------------
CommandPacket::CommandPacket(Packet packet) : Packet(packet.get_bytes())
{
    if (packet.get_descriptor() != COMMAND_PACKET)
        throw std::runtime_error("packet is not a COMMAND packet");
}

//------------------------------------------------------------------------------
// Name:        CommandPacket constructor
// Description: Constructs a COMMAND packet with a field with data.
// Arguments:   - header: Packet header data.
//              - field_desc: Command field descriptor.
//------------------------------------------------------------------------------
CommandPacket::CommandPacket(PacketHeader header, Command command) :
    Packet(header, COMMAND_PACKET)
{
    add_field(COMMAND_NAME, avl::to_bytes(command.name));
    add_fields(command.parameters.to_fields(COMMAND_PARAMS_START));
}

//------------------------------------------------------------------------------
// Name:        CommandPacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: vector of packet bytes
//------------------------------------------------------------------------------
CommandPacket::CommandPacket(std::vector<uint8_t> bytes) : Packet(bytes)
{

    if (!has_field(COMMAND_NAME))
        throw std::runtime_error("packet is missing required NAME field");

}

//------------------------------------------------------------------------------
// Name:        get_command
// Description: Gets the command information from the COMMAND packet's
//              fields.
// Returns:     Command struct described by the packet contents.
//------------------------------------------------------------------------------
Command CommandPacket::get_command()
{

    Command command;

    // Extract the command name string
    std::vector<uint8_t> name_data = get_field(COMMAND_NAME).get_data();
    command.name = std::string(name_data.begin(), name_data.end());

    // Create the prameter list from the parameter list fields if there are any
    ParameterList param_list;
    if (payload.size() > COMMAND_PARAMS_START)
        param_list.from_fields(avl::subvector(payload,
            COMMAND_PARAMS_START, payload.size()-1));
    command.parameters = param_list;

    return command;

}

}
