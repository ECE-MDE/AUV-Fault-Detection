//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a RESPONSE packet conforming to the AVL binary packet
//              protocol, providing functions to construct them and get
//              the information described by their fields.
//==============================================================================

#ifndef RESPONSE_PACKET_H
#define RESPONSE_PACKET_H

// Packet base class
#include "packet.h"

// Util functions
#include "util/time.h"

namespace avl
{

//==============================================================================
//                            PACKET FIELD MAPPING
//==============================================================================

// Enum listing a RESPONSE packet's fields in order starting at 0x00
enum ResponseField
{
    RESPONSE_SOURCE,
    RESPONSE_RESULT,
    RESPONSE_DATA
};

//==============================================================================
//                             STRUCT DEFINITION
//==============================================================================

// Response struct
struct Response
{
    uint64_t source = 0;
    bool result = false;
    std::vector<uint8_t> data = {};
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class ResponsePacket : public Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        ActionPacket constructor
    // Description: Creates a response packet in response to the specified
    //              packet. The response packet header will be populated with
    //              a timestamp when this function is called, a timeout of zero
    //              and a destination ID corresponding to the source ID of the
    //              specified packet. The source ID must be specified.
    //              The SOURCE field will also be populated from the specified
    //              packet's timestamp. RESULT and DATA must be specified.
    // Arguments:   - source_packet: Packet to construct the RESPONSE packet
    //                from.
    //              - source_id: Source ID to put in response packet header.
    //              - result: Response result.
    //              - data: Response data as vector of bytes.
    // Returns:     Response packet corresponding to source packet.
    //--------------------------------------------------------------------------
    static ResponsePacket create_from(Packet source_packet, uint8_t source_id,
        bool result, std::vector<uint8_t> data = {});

    //--------------------------------------------------------------------------
    // Name:        ActionPacket constructor
    // Description: Creates a response packet in response to the specified
    //              packet. The response packet header will be populated with
    //              a timestamp when this function is called, a timeout of zero
    //              and a destination ID corresponding to the source ID of the
    //              specified packet. The source ID must be specified.
    //              The SOURCE field will also be populated from the specified
    //              packet's timestamp. RESULT and DATA must be specified.
    // Arguments:   - source_packet: Packet to construct the RESPONSE packet
    //                from.
    //              - source_id: Source ID to put in response packet header.
    //              - result: Response result.
    //              - data: Response data as string.
    // Returns:     Response packet corresponding to source packet.
    //--------------------------------------------------------------------------
    static ResponsePacket create_from(Packet source_packet, uint8_t source_id,
        bool result, std::string data);

public:

    //--------------------------------------------------------------------------
    // Name:        ResponsePacket constructor
    // Description: Constructs an RESPONSE packet from a base packet class.
    // Arguments:   - packet: Packet to construct the RESPONSE packet from.
    //--------------------------------------------------------------------------
    ResponsePacket(Packet packet);

    //--------------------------------------------------------------------------
    // Name:        ResponsePacket constructor
    // Description: Constructs a RESPONSE packet with the given header data
    //              and field data.
    // Arguments:   - header: Packet header data.
    //              - source: Timestamp of the packet being responded to.
    //              - result: Indicates a success or failure of the packet being
    //                responded to.
    //--------------------------------------------------------------------------
    ResponsePacket(PacketHeader header, Response response);

    //--------------------------------------------------------------------------
    // Name:        ResponsePacket constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: vector of packet bytes
    //--------------------------------------------------------------------------
    ResponsePacket(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        get_response
    // Description: Gets the response information from the RESPONSE packet's
    //              fields.
    // Returns:     Response info struct described by the packet contents.
    //--------------------------------------------------------------------------
    Response get_response();

};

}

#endif // RESPONSE_PACKET_H
