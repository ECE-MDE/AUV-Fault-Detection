//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements an ABORT packet conforming to the AVL binary packet
//              protocol, providing functions to construct them and get the
//              information described by their fields.
//==============================================================================

#ifndef ABORT_PACKET_H
#define ABORT_PACKET_H

// Packet base class
#include "packet.h"

// ParameterList class
#include "parameter_list.h"

namespace avl
{

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class AbortPacket : public Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        AbortPacket constructor
    // Description: Constructs an ABORT packet from a base packet class.
    // Arguments:   - packet: Packet to construct the ABORT packet from.
    //--------------------------------------------------------------------------
    AbortPacket(Packet packet);

    //--------------------------------------------------------------------------
    // Name:        AbortPacket constructor
    // Description: Constructs an ABORT packet with a header.
    // Arguments:   - header: Packet header.
    //--------------------------------------------------------------------------
    AbortPacket(PacketHeader header);

    //--------------------------------------------------------------------------
    // Name:        AbortPacket constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: vector of packet bytes
    //--------------------------------------------------------------------------
    AbortPacket(std::vector<uint8_t> bytes);

};

}

#endif // ABORT_PACKET_H
