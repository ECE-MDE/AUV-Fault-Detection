//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements an ACTION packet conforming to the AVL binary packet
//              protocol, providing functions to construct them and get the
//              information described by their fields.
//==============================================================================

#ifndef ACTION_PACKET_H
#define ACTION_PACKET_H

// Packet base class
#include "packet.h"

// ParameterList class
#include "parameter_list.h"

namespace avl
{

//==============================================================================
//                            PACKET FIELD MAPPING
//==============================================================================

// Enum listing an ACTION packet's fields in order starting at 0x00
enum ActionField
{
    ACTION_NAME,
    ACTION_MODE,
    ACTION_PARAMS_START
};

//==============================================================================
//                             ENUM DEFINITIONS
//==============================================================================

// Enum listing modes for action packets
enum ActionMode
{
    ACTION_MODE_SET,
    ACTION_MODE_APPEND,
    ACTION_MODE_EXECUTE
};

//==============================================================================
//                             STRUCT DEFINITION
//==============================================================================

// Action struct
struct Action
{
    std::string name;
    ActionMode mode;
    ParameterList parameters;
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class ActionPacket : public Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        ActionPacket constructor
    // Description: Constructs an ACTION packet from a base packet class.
    // Arguments:   - packet: Packet to construct the ACTION packet from.
    //--------------------------------------------------------------------------
    ActionPacket(Packet packet);

    //--------------------------------------------------------------------------
    // Name:        ActionPacket constructor
    // Description: Constructs an ACTION packet.
    // Arguments:   - action: Action struct to populate packet fields with.
    //--------------------------------------------------------------------------
    ActionPacket(PacketHeader header, Action action);

    //--------------------------------------------------------------------------
    // Name:        ActionPacket constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: vector of packet bytes
    //--------------------------------------------------------------------------
    ActionPacket(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        get_action
    // Description: Gets the action information from the ACTION packet's fields.
    // Returns:     Action struct described by the packet contents.
    //--------------------------------------------------------------------------
    Action get_action();

};

}

#endif // ACTION_PACKET_H
