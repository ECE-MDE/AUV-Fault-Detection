//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a field class conforming to the AVL binary packet
//              protocol. A field consists of a field length byte, a field
//              descriptor byte, and a number of data bytes. The field length
//              byte is the total number of bytes in the field, including the
//              length byte. The descriptor byte indicates the contents of the
//              field data. The field data is the data bytes themselves. A field
//              may have no data, in which case the field has only the length
//              and descriptor bytes.
//==============================================================================

// Core includes
#include <avl_devices/protocol/aaf/aaf_field.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>

// C++ includes
#include <string>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Field constructor
// Description: Default constructor. Constructs a field with a descriptor of
//              0x00 and no data bytes.
//------------------------------------------------------------------------------
AafField::AafField() : AafField(0x00, {})
{

}

//------------------------------------------------------------------------------
// Name:        Field constructor
// Description: Constructs the field with the given descriptor and no
//              data bytes.
// Arguments:   - field_descriptor: field descriptor byte
//------------------------------------------------------------------------------
AafField::AafField(uint8_t field_descriptor)
{

    descriptor = field_descriptor;
    data = {};

    // Field length is two bytes for the field length byte and the descriptor
    // byte, plus the number of data bytes
    length = 2;

}

//------------------------------------------------------------------------------
// Name:        Field constructor
// Description: Constructs the field with the given descriptor and data
//              bytes.
// Arguments:   - field_descriptor: field descriptor byte
//              - field_data: field data bytes
//------------------------------------------------------------------------------
AafField::AafField(uint8_t field_descriptor, std::vector<uint8_t> field_data)
{

    descriptor = field_descriptor;
    data = field_data;

    // Field length is two bytes for the field length byte and the descriptor
    // byte, plus the number of data bytes
    length = 2 + data.size();

}

//------------------------------------------------------------------------------
// Name:        Field constructor
// Description: Constructs the field from a vector of bytes. The vector
//              should contain all field bytes (length byte, descriptor
//              byte, and data bytes). Throws a std::runtime_error if the
//              bytes do not form a valid field.
//              - field_data: field data bytes
//------------------------------------------------------------------------------
AafField::AafField(std::vector<uint8_t> field_bytes)
{
    set_bytes(field_bytes);
}

//------------------------------------------------------------------------------
// Name:        Field destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
AafField::~AafField()
{

}

//------------------------------------------------------------------------------
// Name:        get_bytes
// Description: Gets the field as a vector of bytes.
// Returns:     Vector of field bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AafField::get_bytes()
{

    std::vector<uint8_t> bytes;

    // Put the field length, descriptor, and data bytes together
    bytes.push_back(length);
    bytes.push_back(descriptor);
    avl::append(bytes, data);

    return bytes;

}

//------------------------------------------------------------------------------
// Name:        set_bytes
// Description: Constructs the field from a vector of bytes. The vector
//              should contain all field bytes (length byte, descriptor
//              byte, and data bytes). Throws a std::runtime_error if the
//              bytes do not form a valid field. This function will
//              overwrite all current field values with the new ones.
// Arguments:   - packet_bytes: vector of field bytes
//------------------------------------------------------------------------------
void AafField::set_bytes(std::vector<uint8_t> field_bytes)
{

    // Check that the bytes form a valid field
    validate_bytes(field_bytes);

    // Get the field length and descriptor (first and second) bytes
    length = field_bytes.at(0);
    descriptor = field_bytes.at(1);

    // Get field data if there is any after the length and descriptor bytes
    if (length > 2)
        data = avl::subvector(field_bytes, 2, length-2);

}

//------------------------------------------------------------------------------
// Name:        get_length
// Description: Gets the field length in number of bytes, including the
//              length byte.
// Returns:     field length in number of bytes, including the length
//              byte.
//------------------------------------------------------------------------------
uint8_t AafField::get_length()
{
    return length;
}

//------------------------------------------------------------------------------
// Name:        get_descriptor
// Description: Gets the field descriptor.
// Returns:     field descriptor byte.
//------------------------------------------------------------------------------
uint8_t AafField::get_descriptor()
{
    return descriptor;
}

//------------------------------------------------------------------------------
// Name:        set_descriptor
// Description: Sets the field descriptor.
// Arguments:   - field_descriptor: field descriptor byte
//------------------------------------------------------------------------------
void AafField::set_descriptor(uint8_t field_descriptor)
{
    descriptor = field_descriptor;
}

//------------------------------------------------------------------------------
// Name:        get_data
// Description: Gets the field data.
// Returns:     field data bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AafField::get_data()
{
    return data;
}

//------------------------------------------------------------------------------
// Name:        set_data
// Description: Sets the field data. Also updates the field length.
// Arguments:   - field_data: field data bytes
//------------------------------------------------------------------------------
void AafField::set_data(std::vector<uint8_t> field_data)
{

    data = field_data;

    // Update the field length. Field length is two bytes for the field length
    // byte and the descriptor byte, plus the number of data bytes
    length = 2 + data.size();

}

//------------------------------------------------------------------------------
// Name:        get_string
// Description: Gets a hex formatted string representing the field. The
//              string is formatted as per the following example:
//                  0x00 0x01 0x11 0xAA 0xFF
// Returns:     Hex formatted string representation of the field
//------------------------------------------------------------------------------
std::string AafField::get_string()
{
    return avl::byte_to_hex(get_bytes());
}

//------------------------------------------------------------------------------
// Name:        validate_bytes
// Description: Checks whether a vector of bytes is a properly formatted
//              field containing a length, descriptor, and data bytes.
//              Throws a std::runtime_error if the bytes are not a properly
//              formatted field.
// Arguments:   - bytes: vector of bytes to validate
//------------------------------------------------------------------------------
void AafField::validate_bytes(std::vector<uint8_t> bytes)
{

    // Check that the length byte (the first byte) matches the length of the
    // vector. The length given by the length byte should include the length
    // byte itself
    if (bytes.size() != bytes.at(0))
    {
        throw std::runtime_error("failed to parse improperly formatted AAF field bytes (length does not match)");
    }

}
