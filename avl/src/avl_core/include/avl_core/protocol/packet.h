//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a AVL communication packet conforming to the AVL
//              binary packet protocol.
//==============================================================================

#ifndef PACKET_H
#define PACKET_H

// Core includes
#include "field.h"

// Util functions
#include "util/byte.h"
#include "util/misc.h"

namespace avl
{

//==============================================================================
//                           PACKET DESCRIPTOR MAPPING
//==============================================================================

// Enum listing packet descriptors in order starting at 0x00
enum PacketDescriptor
{
    RESPONSE_PACKET,
    HEARTBEAT_PACKET,
    MICRO_HEARTBEAT_PACKET,
    DEVICE_PACKET,
    COMMAND_PACKET,
    ACTION_PACKET,
    PASSTHROUGH_PACKET,
    MICRO_PASSTHROUGH_PACKET,
    EVENT_PACKET,
    ABORT_PACKET
};

static const char * descriptor_string[] = {
    "RESPONSE", "HEARBTEAT", "MICRO_HEARTBEAT", "DEVICE", "COMMAND", "ACTION",
    "PASSTHROUGH", "MICRO_PASSTHROUGH", "EVENT", "ABORT" };
inline const char* descriptor_to_string(PacketDescriptor desc)
{
    return descriptor_string[desc];
}

//==============================================================================
//                          PACKET HEADER DEFINITION
//==============================================================================

// Packet start-of-frame bytes
const std::vector<uint8_t> PACKET_SOF = {0x75, 0x65};

// Struct defining the contents of a packet header. Does not include the payload
//length bytes
typedef struct PacketHeader
{

    uint64_t timestamp;      // Packet timestamp in epoch nanoseconds
    uint32_t timeout;        // Packet timeout duration in seconds
    uint8_t source_id;       // Source ID number
    uint8_t destination_id;  // Destination ID number

} PacketHeader;

// Total number of bytes in the header. The total number is:
//    - 2 start-of-frame bytes
//    - The number of bytes in the header struct above,
//    - 1 packet descriptor byte
//    - 2 payload length bytes
const size_t NUM_HEADER_BYTES = 2 + 14 + 1 + 2;

// Minimum number of bytes for a complete packet. The minimum number of required
// bytes is:
//    - The number of header bytes,
//    - An empty payload (0 bytes)
//    - Two CRC bytes
const size_t MIN_PACKET_LENGTH = NUM_HEADER_BYTES + 0 + 2;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        contains_packet
    // Description: Checks whether a vector of bytes contains a packet.
    // Arguments:   - bytes: Vector of bytes to check for a packet.
    //              - i_start: Start index of the packet.
    //              - i_end: End index of the packet.
    // Returns:     True if the vector of bytes contains a packet, false if it
    //              does not.
    //--------------------------------------------------------------------------
    static bool contains_packet(const std::vector<uint8_t>& bytes,
        size_t& i_start, size_t& i_end);

    //--------------------------------------------------------------------------
    // Name:        parse_multiple
    // Description: Parses a vector of bytes into a vector of packets.
    // Arguments:   - bytes: Vector of bytes containing a number of packets.
    // Returns:     Vector of packets parsed from the bytes.
    //--------------------------------------------------------------------------
    static std::vector<Packet> parse_multiple(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        from_hex_string
    // Description: Parses a hex string of bytes into a packet.
    // Arguments:   - hex: String of hex formatted bytes to parse into a packet.
    //              - formatted: True if bytes are formatted as
    //                    "0x01 0x02 0x03"
    //                or false if formatted as
    //                    "010203"
    // Returns:     Packet parsed from the hex string of bytes.
    //--------------------------------------------------------------------------
    static Packet from_hex_string(std::string hex, bool formatted);

public:

    //--------------------------------------------------------------------------
    // Name:        Packet constructor
    // Description: Constructs a packet with the given header data and an empty
    //              payload.
    // Arguments:   - timestamp: Packet epoch timestamp in nanoseconds.
    //              - timeout: Packet timeout in seconds.
    //              - src_id: Packet source ID.
    //              - src_id: Packet destination ID.
    //              - descriptor: Packet descriptor.
    //--------------------------------------------------------------------------
    Packet(PacketHeader header, PacketDescriptor descriptor);

    //--------------------------------------------------------------------------
    // Name:        Packet constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: Vector of packet bytes.
    //--------------------------------------------------------------------------
    Packet(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        Packet destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~Packet();

    //--------------------------------------------------------------------------
    // Name:        get_bytes
    // Description: Gets the packet as a vector of bytes including the header
    //              and checksum.
    // Returns:     Vector of packet bytes.
    //--------------------------------------------------------------------------
    std::vector<uint8_t> get_bytes();

    //--------------------------------------------------------------------------
    // Name:        set_bytes
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              header and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    //              This function will overwrite all current packet
    //              values with the new ones.
    // Arguments:   - bytes: Vector of packet bytes including the header
    //                and the checksum bytes.
    //--------------------------------------------------------------------------
    void set_bytes(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        set_header
    // Description: sets the packet header.
    // Arguments:   - new_header: New packet header.
    //--------------------------------------------------------------------------
    void set_header(PacketHeader new_header);

    //--------------------------------------------------------------------------
    // Name:        get_header
    // Description: Gets the packet header.
    // Returns:     Packet header.
    //--------------------------------------------------------------------------
    PacketHeader get_header();

    //--------------------------------------------------------------------------
    // Name:        get_descriptor
    // Description: Gets the packet descriptor.
    // Returns:     Packet descriptor byte.
    //--------------------------------------------------------------------------
    PacketDescriptor get_descriptor();

    //--------------------------------------------------------------------------
    // Name:        has_field
    // Description: Checks whether or not a packet has a field with a given
    //              field descriptor.
    // Arguments:   - field_descriptor: Field descriptor to check for.
    // Returns:     True if the packet has a field with the given field
    //              descriptor, false otherwise.
    //--------------------------------------------------------------------------
    bool has_field(uint8_t field_descriptor);

    //--------------------------------------------------------------------------
    // Name:        get_field_index
    // Description: Determines the index of a field with a given field
    //              descriptor in the packet's vector of fields.
    // Arguments:   - field_descriptor: Field descriptor to find.
    // Returns:     field index if the field with the given field
    //              descriptor exists in the packet, -1 otherwise.
    //--------------------------------------------------------------------------
    int get_field_index(uint8_t field_descriptor);

    //--------------------------------------------------------------------------
    // Name:        get_num_fields
    // Description: Gets the number of fields in the packet.
    // Returns:     Number of fields in the packet.
    //--------------------------------------------------------------------------
    size_t get_num_fields();

    //--------------------------------------------------------------------------
    // Name:        get_field
    // Description: Gets a field with a given descriptor from the packet. Throws
    //              a std::runtime_exception if the packet does not contain a
    //              field with the given descriptor.
    // Arguments:   - field_descriptor: Field descriptor byte.
    // Returns:     Field with the given descriptor.
    //--------------------------------------------------------------------------
    Field get_field(uint8_t field_descriptor);

    //--------------------------------------------------------------------------
    // Name:        get_fields
    // Description: Gets all fields in the packet.
    // Returns:     Vector of fields in the packet.
    //--------------------------------------------------------------------------
    std::vector<Field> get_fields();

    //--------------------------------------------------------------------------
    // Name:        add_field
    // Description: Adds a field with the given parameters to the packet.
    // Arguments:   - field_descriptor: Field descriptor byte.
    //--------------------------------------------------------------------------
    void add_field(uint8_t field_descriptor);

    //--------------------------------------------------------------------------
    // Name:        add_field
    // Description: Adds a field with the given parameters to the packet.
    // Arguments:   - field_descriptor: Field descriptor byte.
    //              - data: field data bytes
    //--------------------------------------------------------------------------
    void add_field(uint8_t field_descriptor, std::vector<uint8_t> data);

    //--------------------------------------------------------------------------
    // Name:        add_field
    // Description: Adds a field to the packet.
    // Arguments:   - field: Field to add.
    //--------------------------------------------------------------------------
    void add_field(Field field);

    //--------------------------------------------------------------------------
    // Name:        add_fields
    // Description: Adds a vector of fields to the packet.
    // Arguments:   - fields: Fields to add.
    //--------------------------------------------------------------------------
    void add_fields(std::vector<Field> fields);

    //--------------------------------------------------------------------------
    // Name:        clear_fields
    // Description: Removes all fields from the packet.
    //--------------------------------------------------------------------------
    void clear_fields();

    //--------------------------------------------------------------------------
    // Name:        get_string
    // Description: Gets a hex formatted string representing the packet.
    // Arguments:   - formatted: (Optional) If false, returns string formatted as
    //                  000111AAFF
    // Returns:     Hex formatted string representation of the packet.
    //--------------------------------------------------------------------------
    std::string get_string(bool formatted=true);

    //--------------------------------------------------------------------------
    // Name:        get_info
    // Description: Gets a string containing packet information for debugging.
    // Returns:     String containing packet information for debugging.
    //--------------------------------------------------------------------------
    std::string get_info();

protected:

    // Packet header information
    PacketHeader header;

    // Packet descriptor byte
    PacketDescriptor descriptor;

    // Packet payload consisting of a vector of fields
    std::vector<Field> payload;

private:

    //--------------------------------------------------------------------------
    // Name:        get_checksum
    // Description: Calculates a two byte fletcher checksum from a vector of
    //              bytes.
    // Arguments:   - bytes: Vector of bytes to calculate checksum from
    // Returns:      Two byte fletcher checksum.
    //--------------------------------------------------------------------------
    std::vector<uint8_t> get_checksum(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        get_payload_length
    // Description: Calculates the number of bytes in the payload consisting
    //              of packet fields.
    // Returns:     Number of bytes in the packet payload.
    //--------------------------------------------------------------------------
    uint16_t get_payload_length();
    //--------------------------------------------------------------------------
    // Name:        validate_bytes
    // Description: Checks whether a vector of bytes is a properly formatted
    //              packet, including a header and correct checksum bytes.
    //              Throws a std::runtime_error if the bytes are not a properly
    //              formatted packet.
    // Arguments:   - bytes: Vector of bytes to validate.
    //--------------------------------------------------------------------------
    void validate_bytes(std::vector<uint8_t> bytes);

};

}

#endif // PACKET_H
