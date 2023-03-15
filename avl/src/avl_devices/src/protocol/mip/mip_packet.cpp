//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a MIP communication packet conforming to the MIP
//              binary packet protocol. A packet consists of two header bytes,
//              a packet descriptor byte, a number of packet fields, and a two
//              byte checksum. The header bytes are the same for every packet
//              and denote the start of a packet. The packet descriptor byte
//              indicates the contents of the packet. The packet can contain
//              any number of fields, which form the packet payload.
//==============================================================================

// Core includes
#include <avl_devices/protocol/mip/mip_packet.h>
#include <avl_devices/protocol/mip/mip_field.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        parse_multiple
// Description: Parses a vector of bytes into a vector of packets.
// Arguments:   - byes: vector of bytes containing a number of packets
// Returns:     Vector of packets parsed from the bytes.
//------------------------------------------------------------------------------
std::vector<MipPacket> MipPacket::parse_multiple(std::vector<uint8_t> bytes)
{

    // Vector of packets to return
    std::vector<MipPacket> packets;

    // Continue while the helper buffer is not empty
    while(bytes.size() > 0)
    {

        // The total length of a packet is the two header bytes, the packet
        // descriptor and payload length bytes, the payload size, and the two
        // checksum bytes. The fourth byte is the payload length byte
        size_t packet_length = 2 + 2 + bytes.at(3) + 2;

        // Get the packet bytes by getting the number of bytes calculated above
        std::vector<uint8_t> packet_bytes = avl::subvector(bytes, 0, packet_length);

        // Create a packet from the packet bytes and put it in the packet vector
        MipPacket packet(packet_bytes);
        packets.push_back(packet);

        // Remove the used bytes from the byte array
        avl::remove(bytes, 0, packet_length);

    }

    return packets;

}

//------------------------------------------------------------------------------
// Name:        Packet constructor
// Description: Default constructor.
//------------------------------------------------------------------------------
MipPacket::MipPacket()
{

}

//------------------------------------------------------------------------------
// Name:        Packet constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              header and the checksum bytes. Throws a std::runtime_error
//              if the bytes do not form a valid packet.
// Arguments:   - packet_bytes: vector of packet bytes including the header
//                and the checksum bytes.
//------------------------------------------------------------------------------
MipPacket::MipPacket(std::vector<uint8_t> packet_bytes)
{
    set_bytes(packet_bytes);
}

//------------------------------------------------------------------------------
// Name:        Packet destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
 MipPacket::~MipPacket()
{

}

//------------------------------------------------------------------------------
// Name:        get_bytes
// Description: Gets the packet as a vector of bytes including the header
//              and checksum.
// Returns:     Vector of packet bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> MipPacket::get_bytes()
{

    std::vector<uint8_t> bytes;

    // Add the header, descriptor, and payload length bytes
    avl::append(bytes, header);
    bytes.push_back(descriptor);
    bytes.push_back(payload_length);

    // Loop through the packet's fields and add their bytes
    for (size_t i = 0; i < fields.size(); i++)
        avl::append(bytes, fields.at(i).get_bytes());

    // Calculat the chechsum bytes and add them
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
// Arguments:   - packet_bytes: vector of packet bytes including the header
//                and the checksum bytes.
//------------------------------------------------------------------------------
void MipPacket::set_bytes(std::vector<uint8_t> packet_bytes)
{

    // Check that the bytes form a valid packet
    validate_bytes(packet_bytes);

    // Get the packet descriptor byte. This is the third byte, after the
    // first two header bytes
    descriptor = packet_bytes.at(2);

    // Get the packet payload length. This is the fourth byte. The payload
    // length is the total number of bytes in the payload, and does not include
    // the two checksum bytes
    payload_length = packet_bytes.at(3);

    // Get the payload bytes. The number of payload bytes is given by the
    // payload length byte that we just read. The payload bytes start on byte
    // five.
    std::vector<uint8_t> payload_bytes = avl::subvector(packet_bytes, 4, payload_length);

    // Don't parse a payload if there is none
    if(payload_length > 0)
    {

        // The payload may contain multiple fields. The first field will be
        // parsed and then avl::removed from the vector of payload bytes. The next
        // field will then be at the start of the vector

        try
        {

            // Loop while the payload bytes vector still contains data
            while (payload_bytes.size() > 0)
            {

                // Read the field length, which is the first byte of the field.
                uint8_t field_length = payload_bytes.at(0);

                // Create a field from the set of field bytes and add it to the
                // packet payload
                fields.push_back(MipField(avl::subvector(payload_bytes, 0, field_length)));

                // avl::remove the field's bytes from the payload bytes. The next field
                // is now at the beginning of the vector
                avl::remove(payload_bytes, 0, field_length);

            }

        }
        catch (const std::exception&)
        {
            throw std::runtime_error("set_bytes: failed to parse improperly formatted field bytes");
        }

    }

}

//------------------------------------------------------------------------------
// Name:        get_descriptor
// Description: Gets the packet descriptor.
// Returns:     packet descriptor byte.
//------------------------------------------------------------------------------
uint8_t MipPacket::get_descriptor()
{
    return descriptor;
}

//------------------------------------------------------------------------------
// Name:        set_descriptor
// Description: Sets the packet descriptor.
// Arguments:   - packet_descriptor: packet descriptor byte
//------------------------------------------------------------------------------
void MipPacket::set_descriptor(uint8_t packet_descriptor)
{
    descriptor = packet_descriptor;
}

//------------------------------------------------------------------------------
// Name:        has_field
// Description: Checks whether or not a packet has a field with a given
//              field descriptor.
// Arguments:   - field_descriptor: field descriptor to check for
// Returns:     True if the packet has a field with the given field
//              descriptor, false otherwise.
//------------------------------------------------------------------------------
bool MipPacket::has_field(uint8_t field_descriptor)
{

    int idx = get_field_index(field_descriptor);
    if (idx == -1)
    {
        return false;
    }

    return true;

}

//------------------------------------------------------------------------------
// Name:        get_field_index
// Description: Determines the index of a field with a given field
//              descriptor in the packet's vector of fields.
// Arguments:   - field_descriptor: field descriptor to find
// Returns:     field index if the field with the given field
//              descriptor exists in the packet, -1 otherwise.
//------------------------------------------------------------------------------
int MipPacket::get_field_index(uint8_t field_descriptor)
{

    // Loop through the packet fields and check if any of the fields have the
    // given descriptor
    for (size_t i = 0; i < fields.size(); i++)
        if (fields.at(i).get_descriptor() == field_descriptor)
            return i;

    return -1;

}

//------------------------------------------------------------------------------
// Name:        get_num_fields
// Description: Gets the number of fields in the packet.
// Returns:     Number of fields in the packet.
//------------------------------------------------------------------------------
size_t MipPacket::get_num_fields()
{
    return fields.size();
}

//------------------------------------------------------------------------------
// Name:        get_field
// Description: Gets a field with a given descriptor from the packet. Throws
//              a std::runtime_exception if the packet does not contain a
//              field with the given descriptor.
// Arguments:   - field_descriptor: field descriptor byte
// Returns:     field.
//------------------------------------------------------------------------------
MipField MipPacket::get_field(uint8_t field_descriptor)
{

    int idx = get_field_index(field_descriptor);
    if (idx == -1)
    {
        throw std::runtime_error("get_field: packet does not have field with descriptor " + avl::byte_to_hex(field_descriptor));
    }

    return fields.at(idx);

}

//------------------------------------------------------------------------------
// Name:        add_field
// Description: Adds a field with the given parameters to the packet.
// Arguments:   - field_descriptor: field descriptor byte
//------------------------------------------------------------------------------
void MipPacket::add_field(uint8_t field_descriptor)
{
    MipField field(field_descriptor);
    fields.push_back(field);
    payload_length += field.get_length();
}

//------------------------------------------------------------------------------
// Name:        add_field
// Description: Adds a field with the given parameters to the packet.
// Arguments:   - field_descriptor: field descriptor byte
//              - data: field data bytes
//------------------------------------------------------------------------------
void MipPacket::add_field(uint8_t field_descriptor, std::vector<uint8_t> data)
{
    MipField field(field_descriptor, data);
    fields.push_back(field);
    payload_length += field.get_length();
}

//------------------------------------------------------------------------------
// Name:        add_field
// Description: Adds a field to the packet.
// Arguments:   - field: field to add
//------------------------------------------------------------------------------
void MipPacket::add_field(MipField field)
{
    fields.push_back(field);
    payload_length += field.get_length();
}

//--------------------------------------------------------------------------
// Name:        clear_fields
// Description: Removes all fields from the packet.
//--------------------------------------------------------------------------
void MipPacket::clear_fields()
{
    fields.clear();
    payload_length = 0;
}

//------------------------------------------------------------------------------
// Name:        get_string
// Description: Gets a hex formatted string representing the packet. The
//              string is formatted as per the following example:
//                  0x75 0x65 0x00 0x00 0xDA 0x03
// Returns:     Hex formatted string representation of the packet
//------------------------------------------------------------------------------
std::string MipPacket::get_string()
{
    return avl::byte_to_hex(get_bytes());
}

//------------------------------------------------------------------------------
// Name:        get_checksum
// Description: Calculates a two byte fletcher checksum from a vector of
//              bytes.
// Arguments:   - bytes: vector of bytes to calculate checksum from
// Returns:      Two byte fletcher checksum.
//------------------------------------------------------------------------------
std::vector<uint8_t> MipPacket::get_checksum(std::vector<uint8_t> bytes)
{

    uint8_t checksum_msb = 0x00;
    uint8_t checksum_lsb = 0x00;

    // Calculate Fletcher Checksum according to Microstrain documentation
    for (std::vector<uint8_t>::iterator it = bytes.begin(); it != bytes.end(); ++it)
    {
        checksum_msb += *it;
        checksum_lsb += checksum_msb;
    }

    // Vector to contain the checksum
    std::vector<uint8_t> checksum;

    // avl::append checksum to the command and return
    checksum.push_back(checksum_msb);
    checksum.push_back(checksum_lsb);

    return checksum;

}

//------------------------------------------------------------------------------
// Name:        validate_bytes
// Description: Checks whether a vector of bytes is a properly formatted
//              packet, including a header and correct checksum bytes.
//              Throws a std::runtime_error if the bytes are not a properly
//              formatted packet.
// Arguments:   - bytes: vector of bytes to validate
//------------------------------------------------------------------------------
void MipPacket::validate_bytes(std::vector<uint8_t> bytes)
{

    // Check that the first two bytes match the expected header
    if (avl::subvector(bytes, 0, 2) != header)
    {
        throw std::runtime_error("validate_bytes: invalid packet (header does not match)");
    }

    // Check that the payload length byte matches the number of payload bytes.
    // The total payload length is the total number of bytes minus the
    // 6 bytes for header, descriptor, payload length, and checksum
    if (bytes.at(3) != bytes.size()-6)
    {
        throw std::runtime_error("validate_bytes: invalid packet (payload length does not match)");
    }

    // Get the checksum from the bytes (last two bytes)
    std::vector<uint8_t> given_checksum = avl::subvector(bytes, bytes.size()-2, 2);

    // Calculate the checksum for the bytes before the last two checksum bytes
    std::vector<uint8_t> calculated_checksum = get_checksum(avl::subvector(bytes,0,bytes.size()-2));

    // Check that the given checksum matches the calculated checksum
    if (calculated_checksum != given_checksum)
    {
        throw std::runtime_error("validate_bytes: invalid packet (checksum does not match)");
    }

}
