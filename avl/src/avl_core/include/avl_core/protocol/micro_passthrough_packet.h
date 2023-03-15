//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements conversions from a full passthrough packets to micro
//              passthrough packet that contains a minimal number of bytes and
//              vice versa. A micro passthrough consists of the following bytes:
//
//              Byte    Type    Description
//              ----------------------------------------------------------
//              0       uint8   Packet descriptor (0x05)
//              1       uint8   Source vehicle ID (Broadcasting vehicle)
//              2       uint8   Destination vehicle ID (Intermediate Destination)
//              3       uint8   Origin vehicle ID (Original Sender)
//              4       uint8   Target vehicle ID (Final Destination)
//              5       float   Interface (FSD/BSD)
//              6-end   uint8   Data
//==============================================================================

#ifndef MICRO_PASSTHROUGH_PACKET_H
#define MICRO_PASSTHROUGH_PACKET_H

// passthrough packet class for conversion
#include "passthrough_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class MicroPassthroughPacket
{

public:

    //--------------------------------------------------------------------------
    // Name:        from_passthrough_packet
    // Description: Constructs micro passthrough packet bytes from a passthrough
    //              packet.
    // Arguments:   - passthrough_packet: Passthrough packet to construct the micro
    //                passthrough packet from.
    // Returns:     Micro passthrough packet bytes containing data from the
    //              passthrough packet.
    //--------------------------------------------------------------------------
    static std::vector<uint8_t> from_passthrough_packet(PassthroughPacket packet);

    //--------------------------------------------------------------------------
    // Name:        to_passthrough_packet
    // Description: Constructs a passthrough packet from micro passthrough packet
    //              bytes.
    // Arguments:   - micro_passthrough_packet: Micro passthrough packet to
    //                construct the passthrough packet from.
    // Returns:     Passthrough packet containing data from the micro passthrough
    //              packet.
    //--------------------------------------------------------------------------
    static PassthroughPacket to_passthrough_packet(std::vector<uint8_t> bytes);

};

}

#endif // MICRO_PASSTHROUGH_PACKET_H
