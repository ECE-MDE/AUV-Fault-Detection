//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a EventPacket packet conforming to the AVL
//              binary packet protocol, providing functions to construct them
//              and get the information described by their fields.
//==============================================================================

#include "protocol/event_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        EventPacket constructor
// Description: Constructs a EVENT packet with the given header data
//              and field data.
// Arguments:   - header: Packet header data.
//              - event: Event object
//------------------------------------------------------------------------------
EventPacket::EventPacket(PacketHeader header, Event event) :
    Packet(header, EVENT_PACKET)
{
    add_field(EVENT_TYPE, {static_cast<uint8_t>(event.type)});
    add_field(EVENT_RESULT, avl::to_bytes(event.result));
    add_field(EVENT_DATA, event.data);
}

//------------------------------------------------------------------------------
// Name:        EventPacket constructor
// Description: Constructs an EVENT packet from a base packet class.
// Arguments:   - packet: Packet to construct the EVENT packet from.
//------------------------------------------------------------------------------
EventPacket::EventPacket(Packet packet) : Packet(packet.get_bytes())
{
    if (packet.get_descriptor() != EVENT_PACKET)
        throw std::runtime_error("packet is not a EVENT packet");
}

//------------------------------------------------------------------------------
// Name:        EventPacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: vector of packet bytes
//------------------------------------------------------------------------------
EventPacket::EventPacket(std::vector<uint8_t> bytes) : Packet(bytes)
{

    // Ensure that the packet descriptor is correct and that the packet has
    // all of the required fields

    if (descriptor != EVENT_PACKET)
        throw std::runtime_error("incorrect packet descriptor for an "
            "EVENT packet");

    if (!has_field(EVENT_TYPE))
        throw std::runtime_error("packet is missing required TYPE field");
    if (!has_field(EVENT_RESULT))
        throw std::runtime_error("packet is missing required RESULT field");

}

//------------------------------------------------------------------------------
// Name:        get_event
// Description: Gets the event information from the EVENT packet's
//              fields.
// Returns:     Event info struct described by the packet contents.
//------------------------------------------------------------------------------
Event EventPacket::get_event()
{

    Event event;

    if(has_field(EVENT_TYPE))
        event.type = static_cast<EventType>(
            avl::from_bytes<uint8_t>(
                get_field(EVENT_TYPE).get_data()));

    if(has_field(EVENT_RESULT))
        event.result = avl::from_bytes<bool>(
            get_field(EVENT_RESULT).get_data());

    if(has_field(EVENT_DATA))
        event.data = get_field(EVENT_DATA).get_data();

    return event;

}

}
