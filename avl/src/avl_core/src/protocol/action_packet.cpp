//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements an ACTION packet conforming to the AVL binary packet
//              protocol, providing functions to construct them and get the
//              information described by their fields.
//==============================================================================

#include "protocol/action_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        ActionPacket constructor
// Description: Constructs an ACTION packet from a base packet class.
// Arguments:   - packet: Packet to construct the ACTION packet from.
//------------------------------------------------------------------------------
ActionPacket::ActionPacket(Packet packet) : Packet(packet.get_bytes())
{
    if (packet.get_descriptor() != ACTION_PACKET)
        throw std::runtime_error("packet is not an ACTION packet");
}

//------------------------------------------------------------------------------
// Name:        ActionPacket constructor
// Description: Constructs an ACTION packet.
// Arguments:   - action: Action struct to populate packet fields with.
//------------------------------------------------------------------------------
ActionPacket::ActionPacket(PacketHeader header, Action action) :
    Packet(header, ACTION_PACKET)
{
    add_field(ACTION_NAME, avl::to_bytes(action.name));
    add_field(ACTION_MODE, avl::to_bytes(static_cast<uint8_t>(action.mode)));
    add_fields(action.parameters.to_fields(ACTION_PARAMS_START));
}

//------------------------------------------------------------------------------
// Name:        ActionPacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: vector of packet bytes
//------------------------------------------------------------------------------
ActionPacket::ActionPacket(std::vector<uint8_t> bytes) : Packet(bytes)
{

    if (!has_field(ACTION_NAME))
        throw std::runtime_error("packet is missing required TYPE field");

    if (!has_field(ACTION_MODE))
        throw std::runtime_error("packet is missing required MODE field");

}

//------------------------------------------------------------------------------
// Name:        get_action
// Description: Gets the action information from the ACTION packet's fields.
// Returns:     Action struct described by the packet contents.
//------------------------------------------------------------------------------
Action ActionPacket::get_action()
{

    Action action;

    // Extract the action type string and action mode
    std::vector<uint8_t> type_data = get_field(ACTION_NAME).get_data();
    action.name = std::string(type_data.begin(), type_data.end());
    action.mode = static_cast<ActionMode>(
        get_field(ACTION_MODE).get_data().at(0));

    // Create the prameter list from the remaining fields
    ParameterList param_list;
    param_list.from_fields(avl::subvector(payload,
        ACTION_PARAMS_START, payload.size()-2));
    action.parameters = param_list;

    return action;

}

}
