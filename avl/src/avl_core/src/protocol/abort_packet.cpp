//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements an ABORT packet conforming to the AVL binary packet
//              protocol, providing functions to construct them and get the
//              information described by their fields.
//==============================================================================

#include "protocol/abort_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFENITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        AbortPacket constructor
// Description: Constructs an ABORT packet from a base packet class.
// Arguments:   - packet: Packet to construct the ABORT packet from.
//------------------------------------------------------------------------------
AbortPacket::AbortPacket(Packet packet) : Packet(packet.get_bytes())
{
    if (packet.get_descriptor() != ABORT_PACKET)
        throw std::runtime_error("packet is not an ABORT packet");
}

//------------------------------------------------------------------------------
// Name:        AbortPacket constructor
// Description: Constructs an ABORT packet with a header.
// Arguments:   - header: Packet header.
//------------------------------------------------------------------------------
AbortPacket::AbortPacket(PacketHeader header) : Packet(header, ABORT_PACKET)
{

}

//------------------------------------------------------------------------------
// Name:        AbortPacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: vector of packet bytes
//------------------------------------------------------------------------------
AbortPacket::AbortPacket(std::vector<uint8_t> bytes) : Packet(bytes)
{

}

}
