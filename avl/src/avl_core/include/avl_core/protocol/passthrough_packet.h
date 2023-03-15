//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a PASSTHROUGH packet conforming to the AVL binary
//              packet protocol, providing functions to construct them and get
//              the information described by their fields.
//==============================================================================

#ifndef PASSTHROUGH_PACKET_H
#define PASSTHROUGH_PACKET_H

// Packet base class
#include "packet.h"

namespace avl
{

//==============================================================================
//                            PACKET FIELD MAPPING
//==============================================================================

// Enum listing a PASSTHROUGH packet's fields in order starting at 0x00
enum PassthroughField
{
    PASSTHROUGH_ORIGIN_ID,
    PASSTHROUGH_TARGET_ID,
    PASSTHROUGH_CHANNEL,
    PASSTHROUGH_INTERFACE,
    PASSTHROUGH_DATA
};

//==============================================================================
//                             STRUCT DEFINITION
//==============================================================================

struct PassthroughMessage
{
    uint8_t target_id = 0x00;
    uint8_t channel = 0x00;
    uint8_t interface = 0x00;
    std::vector<uint8_t> data;
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class PassthroughPacket : public Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        PassthroughPacket constructor
    // Description: Constructs a PASSTHROUGH packet from a base packet class.
    // Arguments:   - packet: Packet to construct the PASSTHROUGH packet from.
    //--------------------------------------------------------------------------
    PassthroughPacket(Packet packet);

    //--------------------------------------------------------------------------
    // Name:        PassthroughPacket constructor
    // Description: Constructs a PASSTHROUGH packet with the given header data
    //              and passthrough message data.
    // Arguments:   - header: Packet header data.
    //              - message: Passthrough message information.
    //--------------------------------------------------------------------------
    PassthroughPacket(PacketHeader header, PassthroughMessage message);

    //--------------------------------------------------------------------------
    // Name:        PassthroughPacket constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: Vector of packet bytes.
    //--------------------------------------------------------------------------
    PassthroughPacket(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        has_origin_id
    // Description: Checks whether the PASSTHROUGH packet has an origin ID
    //              field.
    // Returns:     True if the PASSTHROUGH packet has an origin ID field, false
    //              if it does not.
    //--------------------------------------------------------------------------
    bool has_origin_id();

    //--------------------------------------------------------------------------
    // Name:        get_origin_id
    // Description: Gets the ID specified in the PASSTHROUGH packet's ORIGIN_ID
    //              field.
    // Returns:     PASSTHROUGH packet's origin ID.
    //--------------------------------------------------------------------------
    uint8_t get_origin_id();

    //--------------------------------------------------------------------------
    // Name:        get_passthrough_message
    // Description: Gets the PASSTHROUGH packet's field data as a struct.
    // Returns:     Passthrough message struct containing the packet's field
    //              data.
    //--------------------------------------------------------------------------
    PassthroughMessage get_passthrough_message();

};

}

#endif // PASSTHROUGH_PACKET_H
