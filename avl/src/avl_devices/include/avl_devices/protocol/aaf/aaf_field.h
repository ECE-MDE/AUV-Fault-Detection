//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a field class conforming to the AAF binary packet
//              protocol. A field consists of a field length byte, a field
//              descriptor byte, and a number of data bytes. The field length
//              byte is the total number of bytes in the field, including the
//              length byte. The descriptor byte indicates the contents of the
//              field data. The field data is the data bytes themselves. A field
//              may have no data, in which case the field has only the length
//              and descriptor bytes.
//==============================================================================

#ifndef AAF_FIELD_H
#define AAF_FIELD_H

// C++ includes
#include <vector>
#include <cstdint>
#include <cstddef>
#include <string>

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class AafField
{

public:

    //--------------------------------------------------------------------------
    // Name:        Field constructor
    // Description: Default constructor. Constructs a field with a descriptor of
    //              0x00 and no data bytes.
    //--------------------------------------------------------------------------
    AafField();

    //--------------------------------------------------------------------------
    // Name:        Field constructor
    // Description: Constructs the field with the given descriptor and no
    //              data bytes.
    // Arguments:   - field_descriptor: field descriptor byte
    //--------------------------------------------------------------------------
    AafField(uint8_t field_descriptor);

    //--------------------------------------------------------------------------
    // Name:        Field constructor
    // Description: Constructs the field with the given descriptor and data
    //              bytes.
    // Arguments:   - field_descriptor: field descriptor byte
    //              - field_data: field data bytes
    //--------------------------------------------------------------------------
    AafField(uint8_t field_descriptor, std::vector<uint8_t> field_data);

    //--------------------------------------------------------------------------
    // Name:        Field constructor
    // Description: Constructs the field from a vector of bytes. The vector
    //              should contain all field bytes (length byte, descriptor
    //              byte, and data bytes). Throws a std::runtime_error if the
    //              bytes do not form a valid field.
    //              - field_data: field data bytes
    //--------------------------------------------------------------------------
    AafField(std::vector<uint8_t> field_bytes);

    //--------------------------------------------------------------------------
    // Name:        Field destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~AafField();

    //--------------------------------------------------------------------------
    // Name:        get_bytes
    // Description: Gets the field as a vector of bytes.
    // Returns:     Vector of field bytes.
    //--------------------------------------------------------------------------
    std::vector<uint8_t> get_bytes();

    //--------------------------------------------------------------------------
    // Name:        set_bytes
    // Description: Constructs the field from a vector of bytes. The vector
    //              should contain all field bytes (length byte, descriptor
    //              byte, and data bytes). Throws a std::runtime_error if the
    //              bytes do not form a valid field. This function will
    //              overwrite all current field values with the new ones.
    // Arguments:   - packet_bytes: vector of field bytes
    //--------------------------------------------------------------------------
    void set_bytes(std::vector<uint8_t> field_bytes);

    //--------------------------------------------------------------------------
    // Name:        get_length
    // Description: Gets the field length in number of bytes, including the
    //              length byte.
    // Returns:     field length in number of bytes, including the length
    //              byte.
    //--------------------------------------------------------------------------
    uint8_t get_length();

    //--------------------------------------------------------------------------
    // Name:        get_descriptor
    // Description: Gets the field descriptor.
    // Returns:     field descriptor byte.
    //--------------------------------------------------------------------------
    uint8_t get_descriptor();

    //--------------------------------------------------------------------------
    // Name:        set_descriptor
    // Description: Sets the field descriptor.
    // Arguments:   - field_descriptor: field descriptor byte
    //--------------------------------------------------------------------------
    void set_descriptor(uint8_t field_descriptor);

    //--------------------------------------------------------------------------
    // Name:        get_data
    // Description: Gets the field data.
    // Returns:     field data bytes.
    //--------------------------------------------------------------------------
    std::vector<uint8_t> get_data();

    //--------------------------------------------------------------------------
    // Name:        set_data
    // Description: Sets the field data. Also updates the field length.
    // Arguments:   - field_data: field data bytes
    //--------------------------------------------------------------------------
    void set_data(std::vector<uint8_t> field_data);

    //--------------------------------------------------------------------------
    // Name:        get_string
    // Description: Gets a hex formatted string representing the field. The
    //              string is formatted as per the following example:
    //                  0x00 0x01 0x11 0xAA 0xFF
    // Returns:     Hex formatted string representation of the field
    //--------------------------------------------------------------------------
    std::string get_string();

private:

    // Field length in number of bytes, including this length byte
    uint8_t length;

    // Field descriptor byte describing the contents of the data field
    uint8_t descriptor;

    // Field data bytes
    std::vector<uint8_t> data;

private:

    //--------------------------------------------------------------------------
    // Name:        validate_bytes
    // Description: Checks whether a vector of bytes is a properly formatted
    //              field containing a length, descriptor, and data bytes.
    //              Throws a std::runtime_error if the bytes are not a properly
    //              formatted field.
    // Arguments:   - bytes: vector of bytes to validate
    //--------------------------------------------------------------------------
    void validate_bytes(std::vector<uint8_t> bytes);

};

#endif // AAF_FIELD_H
