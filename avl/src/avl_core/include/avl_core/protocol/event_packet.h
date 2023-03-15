//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a EventPacket packet conforming to the AVL
//              binary packet protocol, providing functions to construct them
//              and get the information described by their fields.
//==============================================================================

#ifndef EVENT_PACKET_H
#define EVENT_PACKET_H

// Packet base class
#include "packet.h"

// Util functions
#include "util/time.h"

namespace avl
{

//==============================================================================
//                            PACKET FIELD MAPPING
//==============================================================================

// Enum listing a EVENT packet's fields in order starting at 0x00
enum EventField
{
    EVENT_TYPE,
    EVENT_RESULT,
    EVENT_DATA
};

//==============================================================================
//                             STRUCT DEFINITION
//==============================================================================

// Enum listing possible event types
typedef enum EventType
{
    EVENT_TYPE_UNKNOWN,
    EVENT_TYPE_ACTION_COMPLETE,
    EVENT_TYPE_MISSION_STARTED,
    EVENT_TYPE_MISSION_STOPPED,
    EVENT_TYPE_MISSION_PAUSED,
    EVENT_TYPE_MISSION_ADVANCED,
    EVENT_TYPE_MISSION_CLEARED,
    EVENT_TYPE_MISSION_MODE_CHANGED,
    EVENT_TYPE_ACOMMS_BROADCAST_WINDOW_OPEN,
    EVENT_TYPE_ACOMMS_BROADCAST_WINDOW_CLOSED


} EventType;

static const char * type_string[] = { "UNKNOWN",
                                      "ACTION_COMPLETE",
                                      "MISSION_STARTED",
                                      "MISSION_STOPPED",
                                      "MISSION_PAUSED",
                                      "MISSION_ADVANCED",
                                      "MISSION_CLEARED",
                                      "MISSION_MODE_CHANGED",
                                      "ACOMMS_BROADCAST_WINDOW_OPEN",
                                      "ACOMMS_BROADCAST_WINDOW_CLOSED"};
inline const char* type_to_string(EventType type)
{
    return type_string[type];
}

// Event struct
struct Event
{
    EventType type = EVENT_TYPE_UNKNOWN;
    bool result = false;
    std::vector<uint8_t> data = {};
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class EventPacket : public Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        EventPacket constructor
    // Description: Constructs a EVENT packet with the given header data
    //              and field data.
    // Arguments:   - header: Packet header data.
    //              - event: Event object
    //--------------------------------------------------------------------------
    EventPacket(PacketHeader header, Event event);

    //--------------------------------------------------------------------------
    // Name:        EventPacket constructor
    // Description: Constructs an EVENT packet from a base packet class.
    // Arguments:   - packet: Packet to construct the EVENT packet from.
    //--------------------------------------------------------------------------
    EventPacket(Packet packet);

    //--------------------------------------------------------------------------
    // Name:        EventPacket constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: vector of packet bytes
    //--------------------------------------------------------------------------
    EventPacket(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        get_event
    // Description: Gets the event information from the EVENT packet's
    //              fields.
    // Returns:     Event info struct described by the packet contents.
    //--------------------------------------------------------------------------
    Event get_event();

};

}

#endif // EVENT_PACKET_H
