//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for manipulation of bytes (uint8_t) or vectors
//              of bytes.
//==============================================================================

#ifndef BYTE_H
#define BYTE_H

// C++ includes
#include <string>
#include <cstring>
#include <vector>

// Util functions
#include "vector.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        system_is_big_endian
// Description: Checks at runtime if the numbers are stored in bir-endian order.
// Returns:     True if big-endian, false if little-endian.
//------------------------------------------------------------------------------
bool system_is_big_endian();
//------------------------------------------------------------------------------
// Name:        byte_to_hex
// Description: Converts a byte into a hex formatted string. The
//              string is formatted as 0xHH where HH is the hex representation
//              of the byte.
// Arguments:   - bytes: vector of bytes to be converted to a string
// Returns:     Hex formatted representation of the bytes
//------------------------------------------------------------------------------
std::string byte_to_hex(uint8_t byte);

//------------------------------------------------------------------------------
// Name:        byte_to_hex
// Description: Converts a vector of bytes into a hex formatted string.
// Arguments:   - bytes: vector of bytes to be converted to a string
//              - formatted: True if bytes are formatted as
//                    "0x01 0x02 0x03"
//                or false if formatted as
//                    "010203"
// Returns:     Hex formatted representation of the bytes.
//------------------------------------------------------------------------------
std::string byte_to_hex(std::vector<uint8_t> bytes, bool formatted=true);

//------------------------------------------------------------------------------
// Name:        hex_to_byte
// Description: Converts a hex formatted string of bytes into a vector of bytes.
// Arguments:   - hex: String of hex formatted bytes to parse into bytes.
//              - formatted: True if bytes are formatted as
//                    "0x01 0x02 0x03"
//                or false if formatted as
//                    "010203"
// Returns:     Vector of bytes parsed from the hex formatted string.
//------------------------------------------------------------------------------
std::vector<uint8_t> hex_to_byte(std::string hex, bool formatted=true);

//------------------------------------------------------------------------------
// Name:        to_bytes
// Description: Converts a std::string to a vector of bytes.
// Arguments:   - string: String to be converted to bytes.
// Returns:     Vector of bytes representing the string.
//------------------------------------------------------------------------------
std::vector<uint8_t> to_bytes(std::string string);

//------------------------------------------------------------------------------
// Name:        is_ascii
// Description: Checks whether a vector of bytes is a printable ASCII string.
// Arguments:   - bytes: Bytes to check.
// Returns:     True if the bytes are a printable ASCII string, false if they
//              are not.
//------------------------------------------------------------------------------
bool is_ascii(std::vector<uint8_t> bytes);

//==============================================================================
//                          TEMPLATE FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        from_bytes
// Description: Converts a vector of bytes to the given type. Throws a
//              std::runtime_error if he number of bytes in the vector of bytes
//              differs from the size of the specified type. This is to prevent
//              unintended conversion results.
// Arguments:   - bytes: vector of bytes to be converted in big-endian order
//              - little_endian: true to indicate the input is little endian
// Returns:     Converted value.
//------------------------------------------------------------------------------
template<typename T>
T from_bytes(std::vector<uint8_t> bytes, bool little_endian=false)
{

    // Check that there are enough bytes in the vector to form the spcified type
    if (bytes.size() != sizeof(T))
        throw std::runtime_error("from_bytes: invalid number of bytes for "
            "conversion from bytes");

    // Reverse the bytes if the system's endianness different from the input's
    if ((system_is_big_endian() && little_endian) ||
        (!system_is_big_endian() && !little_endian))
        std::reverse(bytes.begin(), bytes.end());

    // Copy the data from the vector of bytes into the new variable
    T var;
    memcpy(&var, &bytes[0], sizeof(T));

    return var;

}

//------------------------------------------------------------------------------
// Name:        vector_from_bytes
// Description: Converts a vector of bytes to a vector of the given type. Throws
//              a std::runtime_error if the number of bytes in the vector of
//              bytes is not an integer multiple of the size of the specified
//              type. This is to prevent unintended conversion results.
// Arguments:   - bytes: vector of bytes to be converted in big-endian order
//              - little_endian: true to indicate the input is little endian
// Returns:     Converted value.
//------------------------------------------------------------------------------
template<typename T>
std::vector<T> vector_from_bytes(std::vector<uint8_t> bytes,
    bool little_endian=false)
{

    // Check that the vector of bytes contains an integer multiple of the number
    // of the size of the type to be converted to
    if (bytes.size() % sizeof(T) != 0)
        throw std::runtime_error("from_bytes: invalid number of bytes for "
            "conversion from bytes");

    // Convert each element individually
    std::vector<T> data_vect;
    for (size_t i = 0; i < bytes.size(); i+=sizeof(T))
    {
        std::vector<uint8_t> subvec = avl::subvector(bytes, i, sizeof(T));
        data_vect.push_back(from_bytes<T>(subvec, little_endian));
    }

    return data_vect;

}

//------------------------------------------------------------------------------
// Name:        to_bytes
// Description: Converts a given variable to a vector of bytes. Outputs in
//              big-endian order unless specified.
// Arguments:   - var: variable to be converted to bytes
//              - little_endian: true to set the output to be little endian
// Returns:     Converted value.
//------------------------------------------------------------------------------
template<typename T>
std::vector<uint8_t> to_bytes(T var, bool little_endian=false)
{

    // Cast the variable into an array of bytes
    uint8_t* i_start = reinterpret_cast<uint8_t*>(&var);

    // Construct a vector of bytes from the array of bytes using the size of
    // the variable's type
    std::vector<uint8_t> bytes(i_start, i_start+sizeof(T));

    // Reverse the bytes if the system's endianness is not what we want
    if ((system_is_big_endian() && little_endian) ||
        (!system_is_big_endian() && !little_endian))
        std::reverse(bytes.begin(), bytes.end());

    return bytes;

}

//------------------------------------------------------------------------------
// Name:        to_bytes
// Description: Converts a vector of variables to a vector of bytes. Outputs in
//              big-endian order unless specified.
// Arguments:   - var: Vector to be converted to bytes.
//              - little_endian: True to set the output to be little endian
// Returns:     Vector of bytes representing the input vector.
//------------------------------------------------------------------------------
template<typename T>
std::vector<uint8_t> to_bytes(std::vector<T> var, bool little_endian=false)
{
    std::vector<uint8_t> bytes;
    for (const T& elem : var)
        avl::append(bytes, to_bytes(elem, little_endian));
    return bytes;
}

} // namespace avl

#endif // BYTE_H
