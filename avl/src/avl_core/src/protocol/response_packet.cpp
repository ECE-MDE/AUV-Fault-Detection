//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a RESPONSE packet conforming to the AVL binary packet
//              protocol, providing functions to construct them and get
//              the information described by their fields.
//==============================================================================

#include "protocol/response_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
ResponsePacket ResponsePacket::create_from(Packet source_packet,
    uint8_t source_id, bool result, std::vector<uint8_t> data)
{

    PacketHeader source_header = source_packet.get_header();

    PacketHeader header;
    header.timestamp = avl::get_epoch_time_nanoseconds();
    header.timeout = 0;
    header.source_id = source_id;
    header.destination_id = source_header.source_id;

    Response response;
    response.source = source_header.timestamp;
    response.result = result;
    response.data = data;
    return ResponsePacket(header, response);

}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
ResponsePacket ResponsePacket::create_from(Packet source_packet,
    uint8_t source_id, bool result, std::string data)
{
    return create_from(source_packet, source_id, result,
        std::vector<uint8_t>(data.begin(), data.end()));
}

//------------------------------------------------------------------------------
// Name:        ResponsePacket constructor
// Description: Constructs an RESPONSE packet from a base packet class.
// Arguments:   - packet: Packet to construct the RESPONSE packet from.
//------------------------------------------------------------------------------
ResponsePacket::ResponsePacket(Packet packet) : Packet(packet.get_bytes())
{
    if (packet.get_descriptor() != RESPONSE_PACKET)
        throw std::runtime_error("packet is not a RESPONSE packet");
}

//------------------------------------------------------------------------------
// Name:        ResponsePacket constructor
// Description: Constructs a RESPONSE packet with the given header data
//              and field data.
// Arguments:   - header: Packet header data.
//              - source: Timestamp of the packet being responded to.
//              - result: Indicates a success or failure of the packet being
//                responded to.
//------------------------------------------------------------------------------
ResponsePacket::ResponsePacket(PacketHeader header, Response response) :
    Packet(header, RESPONSE_PACKET)
{
    add_field(RESPONSE_SOURCE, avl::to_bytes(response.source));
    add_field(RESPONSE_RESULT, avl::to_bytes(response.result));
    add_field(RESPONSE_DATA, response.data);
}

//------------------------------------------------------------------------------
// Name:        ResponsePacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: vector of packet bytes
//------------------------------------------------------------------------------
ResponsePacket::ResponsePacket(std::vector<uint8_t> bytes) : Packet(bytes)
{

    // Ensure that the packet descriptor is correct and that the packet has
    // all of the required fields

    if (descriptor != RESPONSE_PACKET)
        throw std::runtime_error("incorrect packet descriptor for a "
            "RESPONSE packet");

    if (!has_field(RESPONSE_SOURCE))
        throw std::runtime_error("packet is missing required SOURCE field");
    if (!has_field(RESPONSE_RESULT))
        throw std::runtime_error("packet is missing required RESULT field");
    if (!has_field(RESPONSE_DATA))
        throw std::runtime_error("packet is missing required DATA field");

}

//------------------------------------------------------------------------------
// Name:        get_response
// Description: Gets the response information from the RESPONSE packet's
//              fields.
// Returns:     Response info struct described by the packet contents.
//------------------------------------------------------------------------------
Response ResponsePacket::get_response()
{

    auto source_data = get_field(RESPONSE_SOURCE).get_data();
    auto result_data = get_field(RESPONSE_RESULT).get_data();
    auto data = get_field(RESPONSE_DATA).get_data();

    Response response;
    response.source = avl::from_bytes<uint64_t>(source_data);
    response.result = avl::from_bytes<bool>(result_data);
    response.data = data;
    return response;

}

}
