//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a PASSTHROUGH packet conforming to the AVL binary
//              packet protocol, providing functions to construct them and get
//              the information described by their fields.
//==============================================================================

#include "protocol/passthrough_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        PassthroughPacket constructor
// Description: Constructs a PASSTHROUGH packet from a base packet class.
// Arguments:   - packet: Packet to construct the PASSTHROUGH packet from.
//------------------------------------------------------------------------------
PassthroughPacket::PassthroughPacket(Packet packet) : Packet(packet.get_bytes())
{
    if (packet.get_descriptor() != PASSTHROUGH_PACKET)
        throw std::runtime_error("packet is not a PASSTHROUGH packet");
}

//------------------------------------------------------------------------------
// Name:        PassthroughPacket constructor
// Description: Constructs a PASSTHROUGH packet with the given header data
//              and passthrough message data.
// Arguments:   - header: Packet header data.
//              - message: Passthrough message information.
//------------------------------------------------------------------------------
PassthroughPacket::PassthroughPacket(PacketHeader header, PassthroughMessage message) :
    Packet(header, PASSTHROUGH_PACKET)
{
    add_field(PASSTHROUGH_TARGET_ID, {message.target_id});
    add_field(PASSTHROUGH_CHANNEL, avl::to_bytes(message.channel));
    add_field(PASSTHROUGH_INTERFACE, avl::to_bytes(message.interface));
    add_field(PASSTHROUGH_DATA, message.data);
}

//------------------------------------------------------------------------------
// Name:        PassthroughPacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: Vector of packet bytes.
//------------------------------------------------------------------------------
PassthroughPacket::PassthroughPacket(std::vector<uint8_t> bytes) : Packet(bytes)
{

    // Ensure that the packet descriptor is correct and that the packet has
    // all of the required fields

    if (descriptor != PASSTHROUGH_PACKET)
        throw std::runtime_error("incorrect packet descriptor for a "
            "PASSTHROUGH packet");

    if (!has_field(PASSTHROUGH_TARGET_ID))
        throw std::runtime_error("packet is missing required DESTINATION_ID"
            "field");
    if (!has_field(PASSTHROUGH_CHANNEL))
        throw std::runtime_error("packet is missing required CHANNEL "
            "field");
    if (!has_field(PASSTHROUGH_INTERFACE))
        throw std::runtime_error("packet is missing required INTERFACE "
            "field");
    if (!has_field(PASSTHROUGH_DATA))
        throw std::runtime_error("packet is missing required DATA field");

}

//------------------------------------------------------------------------------
// Name:        has_origin_id
// Description: Checks whether the PASSTHROUGH packet has an origin ID
//              field.
// Returns:     True if the PASSTHROUGH packet has an origin ID field, false
//              if it does not.
//------------------------------------------------------------------------------
bool PassthroughPacket::has_origin_id()
{
    return has_field(PASSTHROUGH_ORIGIN_ID);
}

//------------------------------------------------------------------------------
// Name:        get_origin_id
// Description: Gets the ID specified in the PASSTHROUGH packet's ORIGIN_ID
//              field.
// Returns:     PASSTHROUGH packet's origin ID.
//------------------------------------------------------------------------------
uint8_t PassthroughPacket::get_origin_id()
{
    return get_field(PASSTHROUGH_ORIGIN_ID).get_data().at(0);
}

//------------------------------------------------------------------------------
// Name:        get_passthrough_message
// Description: Gets the PASSTHROUGH packet's field data as a struct.
// Returns:     Passthrough message struct containing the packet's field
//              data.
//------------------------------------------------------------------------------
PassthroughMessage PassthroughPacket::get_passthrough_message()
{

    PassthroughMessage message;
    message.target_id = get_field(PASSTHROUGH_TARGET_ID).get_data().at(0);
    message.channel = get_field(PASSTHROUGH_CHANNEL).get_data().at(0);
    message.interface = get_field(PASSTHROUGH_INTERFACE).get_data().at(0);
    message.data = get_field(PASSTHROUGH_DATA).get_data();
    return message;

}

}
