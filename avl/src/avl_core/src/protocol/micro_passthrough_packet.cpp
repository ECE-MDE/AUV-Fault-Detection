//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements conversions from a full passthrough packets to micro
//              passthrough packet that contains a minimal number of bytes and
//              vice versa. A micro passthrough consists of the following 32
//              bytes:
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

#include "protocol/micro_passthrough_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        from_passthrough_packet
// Description: Constructs micro passthrough packet bytes from a passthrough
//              packet.
// Arguments:   - passthrough_packet: Passthrough packet to construct the micro
//                passthrough packet from.
// Returns:     Micro passthrough packet bytes containing data from the
//              passthrough packet.
//------------------------------------------------------------------------------
std::vector<uint8_t> MicroPassthroughPacket::from_passthrough_packet(
    PassthroughPacket packet)
{

    PacketHeader header = packet.get_header();
    PassthroughMessage message = packet.get_passthrough_message();

    // Prepare the vector of bytes to return
    std::vector<uint8_t> bytes;

    // Push the data to the vector
    bytes.push_back(static_cast<uint8_t>(MICRO_PASSTHROUGH_PACKET));
    bytes.push_back(header.source_id);
    bytes.push_back(header.destination_id);
    uint8_t origin_id = (packet.has_origin_id())? packet.get_origin_id() : 0x00;
    bytes.push_back(origin_id);
    bytes.push_back(message.target_id);
    bytes.push_back(message.interface);
    avl::append(bytes, message.data);

    return bytes;

}

//------------------------------------------------------------------------------
// Name:        to_passthrough_packet
// Description: Constructs a passthrough packet from micro passthrough packet
//              bytes.
// Arguments:   - micro_passthrough_packet: Micro passthrough packet to
//                construct the passthrough packet from.
// Returns:     Passthrough packet containing data from the micro passthrough
//              packet.
//------------------------------------------------------------------------------
PassthroughPacket MicroPassthroughPacket::to_passthrough_packet(
    std::vector<uint8_t> bytes)
{

    // Ensure that the packet descriptor is correct and that the vector has
    // the correct number of bytes

    if (bytes.at(0) != MICRO_PASSTHROUGH_PACKET)
        throw std::runtime_error("incorrect packet descriptor for a "
            "micro passthrough packet");

    PacketHeader header;
    // header.timestamp = avl::from_bytes<uint64_t>(avl::subvector(bytes,2,8));
    header.timeout = 0;
    header.source_id = bytes.at(1);
    header.destination_id = bytes.at(2);

    PassthroughMessage message;
    message.target_id = bytes.at(4);
    message.interface = bytes.at(5);
    message.data = avl::subvector(bytes,6);

    // Construct the full packet
    PassthroughPacket packet(header, message);

    // If the origin id is not zero, add it to the packet.
    if(bytes.at(3))
        packet.add_field(PASSTHROUGH_ORIGIN_ID, {header.source_id});

    return packet;

}

}
