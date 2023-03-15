//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a field class conforming to the AVL binary packet
//              protocol.
//==============================================================================

#ifndef FIELD_H
#define FIELD_H

// C++ includes
#include <vector>
#include <cstdint>
#include <cstddef>
#include <string>

// Util functions
#include "util/byte.h"
#include "util/vector.h"

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

namespace avl
{

class Field
{

public:

    //--------------------------------------------------------------------------
    // Name:        parse_multiple
    // Description: Parses a vector of bytes into a vector of fields.
    // Arguments:   - bytes: Vector of bytes containing a number of fields.
    // Returns:     Vector of fields parsed from the bytes.
    //--------------------------------------------------------------------------
    static std::vector<Field> parse_multiple(std::vector<uint8_t> bytes);

public:

    //--------------------------------------------------------------------------
    // Name:        AvlField constructor
    // Description: Default constructor. Constructs a field with a descriptor of
    //              0x00 and no data bytes.
    //--------------------------------------------------------------------------
    Field();

    //--------------------------------------------------------------------------
    // Name:        Field constructor
    // Description: Constructs the field with the given descriptor and no
    //              data bytes.
    // Arguments:   - descriptor: Field descriptor byte.
    //--------------------------------------------------------------------------
    Field(uint8_t descriptor);

    //--------------------------------------------------------------------------
    // Name:        Field constructor
    // Description: Constructs the field with the given descriptor and data
    //              bytes.
    // Arguments:   - descriptor: Field descriptor byte.
    //              - data: Field data bytes.
    //--------------------------------------------------------------------------
    Field(uint8_t descriptor, std::vector<uint8_t> data);

    //--------------------------------------------------------------------------
    // Name:        Field constructor
    // Description: Constructs the field from a vector of bytes. The vector
    //              should contain all field bytes (descriptor, length bytes,
    //              and data bytes). Throws a std::runtime_error if the bytes do
    //              not form a valid field.
    //              - bytes: Field bytes.
    //--------------------------------------------------------------------------
    Field(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        Field destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~Field();

    //--------------------------------------------------------------------------
    // Name:        get_descriptor
    // Description: Gets the field descriptor.
    // Returns:     Field descriptor byte.
    //--------------------------------------------------------------------------
    uint8_t get_descriptor();

    //--------------------------------------------------------------------------
    // Name:        set_descriptor
    // Description: Sets the field descriptor.
    // Arguments:   - descriptor: Field descriptor byte.
    //--------------------------------------------------------------------------
    void set_descriptor(uint8_t descriptor);

    //--------------------------------------------------------------------------
    // Name:        get_bytes
    // Description: Gets the field as a vector of bytes.
    // Returns:     Vector of field bytes.
    //--------------------------------------------------------------------------
    std::vector<uint8_t> get_bytes();

    //--------------------------------------------------------------------------
    // Name:        to_string
    // Description: Gets a hex formatted string representing the field. The
    //              string is formatted as per the following example:
    //                  0x75 0x65 0x00 0x00 0xDA 0x03
    // Returns:     Hex formatted string representation of the field.
    //--------------------------------------------------------------------------
    std::string to_string();

    //--------------------------------------------------------------------------
    // Name:        set_bytes
    // Description: Constructs the field from a vector of bytes. The vector
    //              should contain all field bytes (descriptor, length bytes,
    //              and data bytes). Throws a std::runtime_error if the bytes do
    //              not form a valid field. This function will overwrite all
    //              current field values with the new ones.
    // Arguments:   - bytes: Field bytes.
    //--------------------------------------------------------------------------
    void set_bytes(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        get_data_length
    // Description: Gets the field data length in number of bytes.
    // Returns:     Field data length in number of bytes.
    //--------------------------------------------------------------------------
    uint16_t get_data_length();

    //--------------------------------------------------------------------------
    // Name:        get_data
    // Description: Gets the field data.
    // Returns:     Field data bytes.
    //--------------------------------------------------------------------------
    std::vector<uint8_t> get_data();

    //--------------------------------------------------------------------------
    // Name:        set_data
    // Description: Sets the field data bytes.
    // Arguments:   - data: Field data bytes.
    //--------------------------------------------------------------------------
    void set_data(std::vector<uint8_t> data);

    //--------------------------------------------------------------------------
    // Name:        get_string
    // Description: Gets a hex formatted string representing the field. The
    //              string is formatted as per the following example:
    //                  0x00 0x01 0x11 0xAA 0xFF
    // Returns:     Hex formatted string representation of the field.
    //--------------------------------------------------------------------------
    std::string get_string();

private:

    // Field descriptor byte describing the contents of the field's data
    uint8_t descriptor;

    // Field data bytes
    std::vector<uint8_t> data;

private:

    //--------------------------------------------------------------------------
    // Name:        validate_bytes
    // Description: Checks whether a vector of bytes is a properly formatted
    //              field. Throws a std::runtime_error if the bytes are not a
    //              properly formatted field.
    // Arguments:   - bytes: Vector of bytes to validate.
    //--------------------------------------------------------------------------
    void validate_bytes(std::vector<uint8_t> bytes);

};

}

#endif // FIELD_H
