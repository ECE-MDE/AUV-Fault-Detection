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

#include "protocol/micro_heartbeat_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        from_heartbeat_packet
// Description: Constructs micro heartbeat packet bytes from a heartbeat
//              packet.
// Arguments:   - heartbeat_packet: Heartbeat packet to construct the micro
//                heartbeat packet from.
// Returns:     Micro heartbeat packet bytes containing data from the
//              heartbeat packet.
//------------------------------------------------------------------------------
std::vector<uint8_t> MicroHeartbeatPacket::from_heartbeat_packet(HeartbeatPacket packet)
{

    PacketHeader header = packet.get_header();
    Heartbeat heartbeat = packet.get_heartbeat();

    // Prepare the vector of bytes to return
    std::vector<uint8_t> bytes;

    // Push the data to the vector
    bytes.push_back(static_cast<uint8_t>(MICRO_HEARTBEAT_PACKET));
    bytes.push_back(header.source_id);
    avl::append(bytes, avl::to_bytes(header.timestamp));
    bytes.push_back(heartbeat.fsd_current_action);
    bytes.push_back(heartbeat.fsd_total_actions);
    avl::append(bytes, avl::to_bytes(static_cast<float>(heartbeat.yaw)));
    avl::append(bytes, avl::to_bytes(static_cast<float>(heartbeat.nav_lat)));
    avl::append(bytes, avl::to_bytes(static_cast<float>(heartbeat.nav_lon)));
    avl::append(bytes, avl::to_bytes(static_cast<float>(heartbeat.nav_alt)));
    avl::append(bytes, avl::to_bytes(static_cast<float>(heartbeat.height)));

    return bytes;

}

//------------------------------------------------------------------------------
// Name:        to_heartbeat_packet
// Description: Constructs a heartbeat packet from micro heartbeat packet
//              bytes.
// Arguments:   - micro_heartbeat_packet: Micro heartbeat packet to
//                construct the heartbeat packet from.
// Returns:     Heartbeat packet containing data from the micro heartbeat
//              packet.
//------------------------------------------------------------------------------
HeartbeatPacket MicroHeartbeatPacket::to_heartbeat_packet(std::vector<uint8_t> bytes)
{

    // Ensure that the packet descriptor is correct and that the vector has
    // the correct number of bytes

    if (bytes.at(0) != MICRO_HEARTBEAT_PACKET)
        throw std::runtime_error("incorrect packet descriptor for a "
            "micro heartbeat packet");

    if (bytes.size() != 32)
        throw std::runtime_error("incorrect number of bytes for a "
            "micro heartbeat packet");

    PacketHeader header;
    header.timestamp = avl::from_bytes<uint64_t>(avl::subvector(bytes,2,8));
    header.timeout = 0;
    header.source_id =  bytes.at(1);
    header.destination_id = 0;

    Heartbeat heartbeat;
    heartbeat.fsd_current_action =  bytes.at(10);
    heartbeat.fsd_total_actions =   bytes.at(11);
    heartbeat.yaw =     avl::from_bytes<float>(avl::subvector(bytes,12,4));
    heartbeat.nav_lat = avl::from_bytes<float>(avl::subvector(bytes,16,4));
    heartbeat.nav_lon = avl::from_bytes<float>(avl::subvector(bytes,20,4));
    heartbeat.nav_alt = avl::from_bytes<float>(avl::subvector(bytes,24,4));
    heartbeat.height =  avl::from_bytes<float>(avl::subvector(bytes,28,4));

    return HeartbeatPacket(header, heartbeat);

}

}
