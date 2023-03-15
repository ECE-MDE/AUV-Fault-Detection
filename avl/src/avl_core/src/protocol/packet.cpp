//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a AVL communication packet conforming to the AVL
//              binary packet protocol.
//==============================================================================

#include "protocol/packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        contains_packet
// Description: Checks whether a vector of bytes contains a packet.
// Arguments:   - bytes: Vector of bytes to check for a packet.
//              - i_start: Start index of the packet.
//              - i_end: End index of the packet.
// Returns:     True if the vector of bytes contains a packet, false if it
//              does not.
//------------------------------------------------------------------------------
bool Packet::contains_packet(const std::vector<uint8_t>& bytes,
    size_t& i_start, size_t& i_end)
{

    // Find all start indicies of a packet start-of-frame in the vector of
    // bytes
    std::vector<size_t> i_starts = avl::find_subvector(bytes, PACKET_SOF);
    if (i_starts.empty()) return false;

    // Check each start-of-frame to see if it forms a valid packet
    for (size_t i = 0; i < i_starts.size(); i++)
    {

        // Check that the vector of bytes is long enough to contain at least
        // a packet with no payload after the start-of-frame
        if (bytes.size() - i_starts.at(i) < MIN_PACKET_LENGTH)
            return false;

        // Extract the total payload length
        uint16_t payload_length = avl::from_bytes<uint16_t>(
            avl::subvector(bytes, i_starts.at(i) + 17, 2));

        // Check that the vector contains enough bytes to be a packet with
        // the payload length after the start-of-frame
        size_t packet_length = MIN_PACKET_LENGTH + payload_length;
        if (bytes.size() - i_starts.at(i) < packet_length) return false;

        // Extract the bytes that are supposedly a packet
        std::vector<uint8_t> packet_bytes =
            avl::subvector(bytes, i_starts.at(i), packet_length);

        // Attempt to parse a packet from the packet bytes. If it fails,
        // it is not a packet
        try
        {
            Packet packet(packet_bytes);
            i_start = i_starts.at(i);
            i_end = i_start + packet_length;
            return true;
        }
        catch(const std::exception&)
        {
            // If bytes failed to form a valid packet, move on to the next
            // start-of-frame
        }

    }

    return false;

}

//------------------------------------------------------------------------------
// Name:        parse_multiple
// Description: Parses a vector of bytes into a vector of packets.
// Arguments:   - bytes: Vector of bytes containing a number of packets.
// Returns:     Vector of packets parsed from the bytes.
//------------------------------------------------------------------------------
std::vector<Packet> Packet::parse_multiple(std::vector<uint8_t> bytes)
{

    // Vector of packets to return
    std::vector<Packet> packets;

    try
    {

        while(bytes.size() > 0)
        {

            // The total length of a packet is the header bytes plus the
            // number of payload bytes plus the two CRC bytes
            uint16_t payload_length =
                avl::from_bytes<uint16_t>(avl::subvector(bytes, 17, 2));
            size_t packet_length = MIN_PACKET_LENGTH + payload_length;

            // Extract the packet bytes by extracting the number of bytes
            // calculated above
            std::vector<uint8_t> packet_bytes =
                avl::subvector(bytes, 0, packet_length);

            // Create a packet from the packet bytes and put it in the
            // packet vector
            Packet packet(packet_bytes);
            packets.push_back(packet);

            // Remove the used bytes from the byte array
            avl::remove(bytes, 0, packet_length);

        }

    }
    catch (const std::exception& ex)
    {
        throw std::runtime_error(std::string("parse_multiple: failed to "
            "parse improperly formatted packet bytes (") + ex.what() + ")");
    }

    return packets;

}

//------------------------------------------------------------------------------
// Name:        from_hex_string
// Description: Parses a hex string of bytes into a packet.
// Arguments:   - hex: String of hex formatted bytes to parse into a packet.
//              - formatted: True if bytes are formatted as
//                    "0x01 0x02 0x03"
//                or false if formatted as
//                    "010203"
// Returns:     Packet parsed from the hex string of bytes.
//------------------------------------------------------------------------------
Packet Packet::from_hex_string(std::string hex, bool formatted)
{
    std::vector<uint8_t> bytes = avl::hex_to_byte(hex, formatted);
    return Packet(bytes);
}

//------------------------------------------------------------------------------
// Name:        Packet constructor
// Description: Constructs a packet with the given header data and an empty
//              payload.
// Arguments:   - timestamp: Packet epoch timestamp in nanoseconds.
//              - timeout: Packet timeout in seconds.
//              - src_id: Packet source ID.
//              - src_id: Packet destination ID.
//              - descriptor: Packet descriptor.
//------------------------------------------------------------------------------
Packet::Packet(PacketHeader header, PacketDescriptor descriptor) :
    header(header), descriptor(descriptor)
{

}

//------------------------------------------------------------------------------
// Name:        Packet constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: Vector of packet bytes.
//------------------------------------------------------------------------------
Packet::Packet(std::vector<uint8_t> bytes)
{
    set_bytes(bytes);
}

//------------------------------------------------------------------------------
// Name:        Packet destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
Packet::~Packet()
{

}

//------------------------------------------------------------------------------
// Name:        get_bytes
// Description: Gets the packet as a vector of bytes including the header
//              and checksum.
// Returns:     Vector of packet bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> Packet::get_bytes()
{

    // Create a vector of bytes containing the packet header information
    std::vector<uint8_t> bytes = PACKET_SOF;
    avl::append(bytes, avl::to_bytes(header.timestamp));
    avl::append(bytes, avl::to_bytes(header.timeout));
    avl::append(bytes, avl::to_bytes(header.source_id));
    avl::append(bytes, avl::to_bytes(header.destination_id));
    bytes.push_back(static_cast<uint8_t>(descriptor));
    avl::append(bytes, avl::to_bytes(get_payload_length()));

    // Loop through the packet's fields and add their bytes
    for (Field field : payload)
        avl::append(bytes, field.get_bytes());

    // Calculate the chechsum bytes and append them
    avl::append(bytes, get_checksum(bytes));

    return bytes;

}

//------------------------------------------------------------------------------
// Name:        set_bytes
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              header and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
//              This function will overwrite all current packet
//              values with the new ones.
// Arguments:   - bytes: Vector of packet bytes including the header
//                and the checksum bytes.
//------------------------------------------------------------------------------
void Packet::set_bytes(std::vector<uint8_t> bytes)
{

    // Check that the bytes form a valid packet
    validate_bytes(bytes);

    // Extract the packet header data
    header.timestamp = avl::from_bytes<uint64_t>(
        avl::subvector(bytes, 2, 8));
    header.timeout = avl::from_bytes<uint32_t>(
        avl::subvector(bytes, 10, 4));
    header.source_id = bytes.at(14);
    header.destination_id = bytes.at(15);
    descriptor = static_cast<PacketDescriptor>(bytes.at(16));
    uint32_t payload_length = avl::from_bytes<uint16_t>(
        avl::subvector(bytes, 17, 2));

    // Extract the packet payload
    std::vector<uint8_t> payload_bytes =
        avl::subvector(bytes, 19, payload_length);
    payload = Field::parse_multiple(payload_bytes);

}

//------------------------------------------------------------------------------
// Name:        set_header
// Description: sets the packet header.
// Arguments:   - new_header: New packet header.
//------------------------------------------------------------------------------
void Packet::set_header(PacketHeader new_header)
{
    header = new_header;
}

//------------------------------------------------------------------------------
// Name:        get_header
// Description: Gets the packet header.
// Returns:     Packet header.
//------------------------------------------------------------------------------
PacketHeader Packet::get_header()
{
    return header;
}

//------------------------------------------------------------------------------
// Name:        get_descriptor
// Description: Gets the packet descriptor.
// Returns:     Packet descriptor byte.
//------------------------------------------------------------------------------
PacketDescriptor Packet::get_descriptor()
{
    return descriptor;
}

//------------------------------------------------------------------------------
// Name:        has_field
// Description: Checks whether or not a packet has a field with a given
//              field descriptor.
// Arguments:   - field_descriptor: Field descriptor to check for.
// Returns:     True if the packet has a field with the given field
//              descriptor, false otherwise.
//------------------------------------------------------------------------------
bool Packet::has_field(uint8_t field_descriptor)
{
    return get_field_index(field_descriptor) != -1;
}

//------------------------------------------------------------------------------
// Name:        get_field_index
// Description: Determines the index of a field with a given field
//              descriptor in the packet's vector of fields.
// Arguments:   - field_descriptor: Field descriptor to find.
// Returns:     field index if the field with the given field
//              descriptor exists in the packet, -1 otherwise.
//------------------------------------------------------------------------------
int Packet::get_field_index(uint8_t field_descriptor)
{
    for (size_t i = 0; i < payload.size(); i++)
        if (payload.at(i).get_descriptor() == field_descriptor)
            return i;
    return -1;
}

//------------------------------------------------------------------------------
// Name:        get_num_fields
// Description: Gets the number of fields in the packet.
// Returns:     Number of fields in the packet.
//------------------------------------------------------------------------------
size_t Packet::get_num_fields()
{
    return payload.size();
}

//------------------------------------------------------------------------------
// Name:        get_field
// Description: Gets a field with a given descriptor from the packet. Throws
//              a std::runtime_exception if the packet does not contain a
//              field with the given descriptor.
// Arguments:   - field_descriptor: Field descriptor byte.
// Returns:     Field with the given descriptor.
//------------------------------------------------------------------------------
Field Packet::get_field(uint8_t field_descriptor)
{
    int idx = get_field_index(field_descriptor);
    if (idx == -1)
        throw std::runtime_error("get_field: packet does not have field with "
            "descriptor " + avl::byte_to_hex(field_descriptor));
    return payload.at(idx);
}

//------------------------------------------------------------------------------
// Name:        get_fields
// Description: Gets all fields in the packet.
// Returns:     Vector of fields in the packet.
//------------------------------------------------------------------------------
std::vector<Field> Packet::get_fields()
{
    return payload;
}

//------------------------------------------------------------------------------
// Name:        add_field
// Description: Adds a field with the given parameters to the packet.
// Arguments:   - field_descriptor: Field descriptor byte.
//------------------------------------------------------------------------------
void Packet::add_field(uint8_t field_descriptor)
{
    Field field(field_descriptor);
    payload.push_back(field);
}

//------------------------------------------------------------------------------
// Name:        add_field
// Description: Adds a field with the given parameters to the packet.
// Arguments:   - field_descriptor: Field descriptor byte.
//              - data: field data bytes
//------------------------------------------------------------------------------
void Packet::add_field(uint8_t field_descriptor, std::vector<uint8_t> data)
{
    Field field(field_descriptor, data);
    payload.push_back(field);
}

//------------------------------------------------------------------------------
// Name:        add_field
// Description: Adds a field to the packet.
// Arguments:   - field: Field to add.
//------------------------------------------------------------------------------
void Packet::add_field(Field field)
{
    payload.push_back(field);
}

//------------------------------------------------------------------------------
// Name:        add_fields
// Description: Adds a vector of fields to the packet.
// Arguments:   - fields: Fields to add.
//------------------------------------------------------------------------------
void Packet::add_fields(std::vector<Field> fields)
{
    avl::append(payload, fields);
}

//------------------------------------------------------------------------------
// Name:        clear_fields
// Description: Removes all fields from the packet.
//------------------------------------------------------------------------------
void Packet::clear_fields()
{
    payload.clear();
}

//------------------------------------------------------------------------------
// Name:        get_string
// Description: Gets a hex formatted string representing the packet.
// Arguments:   - formatted: (Optional) If false, returns string formatted as
//                  000111AAFF
// Returns:     Hex formatted string representation of the packet.
//------------------------------------------------------------------------------
std::string Packet::get_string(bool formatted)
{
    return avl::byte_to_hex(get_bytes(), formatted);
}

//------------------------------------------------------------------------------
// Name:        get_info
// Description: Gets a string containing packet information for debugging.
// Returns:     String containing packet information for debugging.
//------------------------------------------------------------------------------
std::string Packet::get_info()
{

    std::stringstream ss;
    ss << "================================================================================" << std::endl;
    ss << "Packet Info" << std::endl;
    ss << "timestamp:      " << header.timestamp << std::endl;
    ss << "timeout:        " << header.timeout << std::endl;
    ss << "source_id:      " << static_cast<int>(header.source_id) << std::endl;
    ss << "destination_id: " << static_cast<int>(header.destination_id) << std::endl;
    ss << "descriptor:     " << descriptor_to_string(descriptor) << std::endl;
    ss << "payload size:   " << get_payload_length() << std::endl;
    ss << "--------------------------------------------------------------------------------" << std::endl;
    for (auto field : get_fields())
    {

        int desc = static_cast<int>(field.get_descriptor());
        auto data = field.get_data();
        std::string data_hex = avl::byte_to_hex(data);
        std::string data_str(data.begin(), data.end());

        ss << "field " << desc << " data: " << data_hex;
        if (avl::is_ascii(data))
            ss << " (" << data_str << ")";
        ss << std::endl;

    }
    ss << "================================================================================" << std::endl;
    return ss.str();

}

//------------------------------------------------------------------------------
// Name:        get_checksum
// Description: Calculates a two byte fletcher checksum from a vector of
//              bytes.
// Arguments:   - bytes: Vector of bytes to calculate checksum from
// Returns:      Two byte fletcher checksum.
//------------------------------------------------------------------------------
std::vector<uint8_t> Packet::get_checksum(std::vector<uint8_t> bytes)
{
    uint8_t msb = 0x00;
    uint8_t lsb = 0x00;
    for (const uint8_t& byte : bytes)
    {
        msb += byte;
        lsb += msb;
    }
    return {msb, lsb};
}

//------------------------------------------------------------------------------
// Name:        get_payload_length
// Description: Calculates the number of bytes in the payload consisting
//              of packet fields.
// Returns:     Number of bytes in the packet payload.
//------------------------------------------------------------------------------
uint16_t Packet::get_payload_length()
{
    uint16_t payload_length = 0;
    for (Field field : payload)
        payload_length += field.get_bytes().size();
    return payload_length;
}

//------------------------------------------------------------------------------
// Name:        validate_bytes
// Description: Checks whether a vector of bytes is a properly formatted
//              packet, including a header and correct checksum bytes.
//              Throws a std::runtime_error if the bytes are not a properly
//              formatted packet.
// Arguments:   - bytes: Vector of bytes to validate.
//------------------------------------------------------------------------------
void Packet::validate_bytes(std::vector<uint8_t> bytes)
{

    // Check that the vector contains the minimum number of bytes for a
    // packet
    if (bytes.size() < MIN_PACKET_LENGTH)
        throw std::runtime_error("validate_bytes: invalid packet "
            "(insufficient number of bytes)");

    // Check that the first two bytes match the expected header
    if (avl::subvector(bytes, 0, 2) != PACKET_SOF)
        throw std::runtime_error("validate_bytes: invalid packet "
            "(header does not match)");

    // Extract the payload length bytes
    uint16_t payload_length =
        avl::from_bytes<uint16_t>(avl::subvector(bytes, 17, 2));

    // Check that the extracted payload length matches the number of bytes
    // in the vector
    if ((MIN_PACKET_LENGTH + payload_length) != bytes.size())
        throw std::runtime_error("validate_bytes: invalid packet "
            "(calculated payload length does not agree with vector size)");

    // Get the checksum from the bytes (last two bytes)
    std::vector<uint8_t> given_checksum =
        avl::subvector(bytes, bytes.size()-2, 2);

    // Calculate the checksum for the bytes before the last two checksum
    // bytes
    std::vector<uint8_t> calculated_checksum =
        get_checksum(avl::subvector(bytes, 0, bytes.size()-2));

    // Check that the given checksum matches the calculated checksum
    if (calculated_checksum != given_checksum)
        throw std::runtime_error("validate_bytes: invalid packet "
            "(checksum does not match)");

}

}
