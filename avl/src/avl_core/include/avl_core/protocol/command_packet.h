//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a COMMAND packet conforming to the AVL binary packet
//              protocol, providing functions to construct them and get
//              the information described by their fields.
//==============================================================================

#ifndef COMMAND_PACKET_H
#define COMMAND_PACKET_H

// Packet base class
#include "packet.h"

// ParameterList class
#include "parameter_list.h"

namespace avl
{

//==============================================================================
//                            PACKET FIELD MAPPING
//==============================================================================

// Enum listing a COMMAND packet's fields in order starting at 0x00
enum CommandField
{
    COMMAND_NAME,
    COMMAND_PARAMS_START
};

//==============================================================================
//                             STRUCT DEFINITION
//==============================================================================

// Command struct
struct Command
{
    std::string name;
    ParameterList parameters;
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class CommandPacket : public Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        CommandPacket constructor
    // Description: Constructs a COMMAND packet from a base packet class.
    // Arguments:   - packet: Packet to construct the COMMAND packet from.
    //--------------------------------------------------------------------------
    CommandPacket(Packet packet);

    //--------------------------------------------------------------------------
    // Name:        CommandPacket constructor
    // Description: Constructs a COMMAND packet with a field with data.
    // Arguments:   - header: Packet header data.
    //              - field_desc: Command field descriptor.
    //--------------------------------------------------------------------------
    CommandPacket(PacketHeader header, Command command);

    //--------------------------------------------------------------------------
    // Name:        CommandPacket constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: vector of packet bytes
    //--------------------------------------------------------------------------
    CommandPacket(std::vector<uint8_t> bytes);
    //--------------------------------------------------------------------------
    // Name:        get_command
    // Description: Gets the command information from the COMMAND packet's
    //              fields.
    // Returns:     Command struct described by the packet contents.
    //--------------------------------------------------------------------------
    Command get_command();

};

}

#endif // COMMAND_PACKET_H
