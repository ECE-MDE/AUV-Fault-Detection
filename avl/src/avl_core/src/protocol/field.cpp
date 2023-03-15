//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a field class conforming to the AVL binary packet
//              protocol.
//==============================================================================

#include "protocol/field.h"

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

namespace avl
{

//------------------------------------------------------------------------------
// Name:        parse_multiple
// Description: Parses a vector of bytes into a vector of fields.
// Arguments:   - bytes: Vector of bytes containing a number of fields.
// Returns:     Vector of fields parsed from the bytes.
//------------------------------------------------------------------------------
std::vector<Field> Field::parse_multiple(std::vector<uint8_t> bytes)
{

    // Vector of fields to return
    std::vector<Field> fields;

    try
    {

        while (bytes.size() > 0)
        {

            // Extract the field descriptor, which is the first byte of the
            // field.
            uint8_t descriptor = bytes.at(0);

            // Extract the field data length bytes, which are bytes 2 and 3
            uint16_t data_length = avl::from_bytes<uint16_t>(
                avl::subvector(bytes, 1, 2));

            // Extract the field data bytes
            std::vector<uint8_t> data;
            if (data_length > 0)
                data = avl::subvector(bytes, 3, data_length);

            // Create a field from the set of field bytes and add it to the
            // packet payload
            fields.push_back(Field(descriptor, data));

            // Remove the field's bytes from the payload bytes. The next
            // field is now at the beginning of the vector
            avl::remove(bytes, 0, data_length+3);

        }

    }
    catch (const std::exception& ex)
    {
        throw std::runtime_error(std::string("parse_multiple: failed to "
            "parse  improperly formatted field bytes (") + ex.what() + ")");
    }

    return fields;

}

//------------------------------------------------------------------------------
// Name:        AvlField constructor
// Description: Default constructor. Constructs a field with a descriptor of
//              0x00 and no data bytes.
//------------------------------------------------------------------------------
Field::Field() : Field(0x00)
{

}

//------------------------------------------------------------------------------
// Name:        Field constructor
// Description: Constructs the field with the given descriptor and no
//              data bytes.
// Arguments:   - descriptor: Field descriptor byte.
//------------------------------------------------------------------------------
Field::Field(uint8_t descriptor) : descriptor(descriptor)
{

}

//------------------------------------------------------------------------------
// Name:        Field constructor
// Description: Constructs the field with the given descriptor and data
//              bytes.
// Arguments:   - descriptor: Field descriptor byte.
//              - data: Field data bytes.
//------------------------------------------------------------------------------
Field::Field(uint8_t descriptor, std::vector<uint8_t> data) :
    descriptor(descriptor), data(data)
{

}

//------------------------------------------------------------------------------
// Name:        Field constructor
// Description: Constructs the field from a vector of bytes. The vector
//              should contain all field bytes (descriptor, length bytes,
//              and data bytes). Throws a std::runtime_error if the bytes do
//              not form a valid field.
//              - bytes: Field bytes.
//------------------------------------------------------------------------------
Field::Field(std::vector<uint8_t> bytes)
{
    set_bytes(bytes);
}

//------------------------------------------------------------------------------
// Name:        Field destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
Field::~Field()
{

}

//------------------------------------------------------------------------------
// Name:        get_descriptor
// Description: Gets the field descriptor.
// Returns:     Field descriptor byte.
//------------------------------------------------------------------------------
uint8_t Field::get_descriptor()
{
    return descriptor;
}

//------------------------------------------------------------------------------
// Name:        set_descriptor
// Description: Sets the field descriptor.
// Arguments:   - descriptor: Field descriptor byte.
//------------------------------------------------------------------------------
void Field::set_descriptor(uint8_t descriptor)
{
    this->descriptor = descriptor;
}

//------------------------------------------------------------------------------
// Name:        get_bytes
// Description: Gets the field as a vector of bytes.
// Returns:     Vector of field bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> Field::get_bytes()
{
    std::vector<uint8_t> bytes;
    bytes.push_back(descriptor);
    avl::append(bytes, avl::to_bytes<uint16_t>(data.size()));
    avl::append(bytes, data);
    return bytes;
}

//------------------------------------------------------------------------------
// Name:        to_string
// Description: Gets a hex formatted string representing the field. The
//              string is formatted as per the following example:
//                  0x75 0x65 0x00 0x00 0xDA 0x03
// Returns:     Hex formatted string representation of the field.
//------------------------------------------------------------------------------
std::string Field::to_string()
{
    return avl::byte_to_hex(get_bytes());
}

//------------------------------------------------------------------------------
// Name:        set_bytes
// Description: Constructs the field from a vector of bytes. The vector
//              should contain all field bytes (descriptor, length bytes,
//              and data bytes). Throws a std::runtime_error if the bytes do
//              not form a valid field. This function will overwrite all
//              current field values with the new ones.
// Arguments:   - bytes: Field bytes.
//------------------------------------------------------------------------------
void Field::set_bytes(std::vector<uint8_t> bytes)
{

    // Check that the bytes form a valid field
    validate_bytes(bytes);

    // Extract the field descriptor byte
    descriptor = bytes.at(0);

    // Extract the field data length bytes
    uint16_t data_length = avl::from_bytes<uint16_t>(
        avl::subvector(bytes, 1, 2));

    // Extract the field data bytes
    data = avl::subvector(bytes, 3, data_length);

}

//------------------------------------------------------------------------------
// Name:        get_data_length
// Description: Gets the field data length in number of bytes.
// Returns:     Field data length in number of bytes.
//------------------------------------------------------------------------------
uint16_t Field::get_data_length()
{
    return data.size();
}

//------------------------------------------------------------------------------
// Name:        get_data
// Description: Gets the field data.
// Returns:     Field data bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> Field::get_data()
{
    return data;
}

//------------------------------------------------------------------------------
// Name:        set_data
// Description: Sets the field data bytes.
// Arguments:   - data: Field data bytes.
//------------------------------------------------------------------------------
void Field::set_data(std::vector<uint8_t> data)
{
    this->data = data;
}

//------------------------------------------------------------------------------
// Name:        get_string
// Description: Gets a hex formatted string representing the field. The
//              string is formatted as per the following example:
//                  0x00 0x01 0x11 0xAA 0xFF
// Returns:     Hex formatted string representation of the field.
//------------------------------------------------------------------------------
std::string Field::get_string()
{
    return avl::byte_to_hex(get_bytes());
}

//------------------------------------------------------------------------------
// Name:        validate_bytes
// Description: Checks whether a vector of bytes is a properly formatted
//              field. Throws a std::runtime_error if the bytes are not a
//              properly formatted field.
// Arguments:   - bytes: Vector of bytes to validate.
//------------------------------------------------------------------------------
void Field::validate_bytes(std::vector<uint8_t> bytes)
{

    // A field must contain at least three bytes: the descriptor byte and
    // the two field data length bytes
    if (bytes.size() < 3)
    {
        throw std::runtime_error("failed to parse improperly formatted "
            "field bytes (insufficient number of bytes)");
        return;
    }

    // Check that the data length bytes (the second two bytes) are
    // consistent. There should be the descriptor byte, the two field length
    // bytes, and the number of data bytes specified by the data length
    // bytes.
    uint16_t field_data_length =
        avl::from_bytes<uint16_t>(avl::subvector(bytes, 1, 2));
    size_t num_field_bytes = field_data_length + 3;

    if (bytes.size() != num_field_bytes)
    {
        throw std::runtime_error("failed to parse improperly formatted "
            "field bytes (length does not match)");
        return;
    }

}

}
