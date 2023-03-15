//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements conversions from a full heartbeat packets to micro
//              heartbeat packet that contains a minimal number of bytes and
//              vice versa. A micro heartbeat consists of the following 32
//              bytes:
//
//              Byte    Type    Description
//              ----------------------------------------------------------
//              0       uint8   Packet descriptor (0x01)
//              1       uint8   Source vehicle ID
//              2-9     uint64  Packet timestamp in epoch nanoseconds
//              10      uint8   FSD current action number
//              11      uint8   FSD total actions
//              12-15   float   Yaw in degrees.
//              16-19   float   Nav lat in degrees.
//              20-23   float   Nav lon in degrees.
//              24-27   float   Depth in meters.
//              28-31   float   Height in meters.
//==============================================================================

#ifndef MICRO_HEARTBEAT_PACKET_H
#define MICRO_HEARTBEAT_PACKET_H

// Heartbeat packet class for conversion
#include "heartbeat_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class MicroHeartbeatPacket
{

public:

    //--------------------------------------------------------------------------
    // Name:        from_heartbeat_packet
    // Description: Constructs micro heartbeat packet bytes from a heartbeat
    //              packet.
    // Arguments:   - heartbeat_packet: Heartbeat packet to construct the micro
    //                heartbeat packet from.
    // Returns:     Micro heartbeat packet bytes containing data from the
    //              heartbeat packet.
    //--------------------------------------------------------------------------
    static std::vector<uint8_t> from_heartbeat_packet(HeartbeatPacket packet);

    //--------------------------------------------------------------------------
    // Name:        to_heartbeat_packet
    // Description: Constructs a heartbeat packet from micro heartbeat packet
    //              bytes.
    // Arguments:   - micro_heartbeat_packet: Micro heartbeat packet to
    //                construct the heartbeat packet from.
    // Returns:     Heartbeat packet containing data from the micro heartbeat
    //              packet.
    //--------------------------------------------------------------------------
    static HeartbeatPacket to_heartbeat_packet(std::vector<uint8_t> bytes);

};

}

#endif // MICRO_HEARTBEAT_PACKET_H
