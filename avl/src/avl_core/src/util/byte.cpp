//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for manipulation of bytes (uint8_t) or vectors
//              of bytes.
//==============================================================================

#include "util/byte.h"
#include "util/string.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        system_is_big_endian
// Description: Checks at runtime if the numbers are stored in bir-endian order.
// Returns:     True if big-endian, false if little-endian.
//------------------------------------------------------------------------------
bool system_is_big_endian()
{
   short int word = 0x0001;
   char *byte = (char *) &word;
   return !static_cast<bool>(byte[0]);
}

//------------------------------------------------------------------------------
// Name:        byte_to_hex
// Description: Converts a byte into a hex formatted string. The
//              string is formatted as 0xHH where HH is the hex representation
//              of the byte.
// Arguments:   - bytes: vector of bytes to be converted to a string
// Returns:     Hex formatted representation of the bytes
//------------------------------------------------------------------------------
std::string byte_to_hex(uint8_t byte)
{

    char buffer[5];
    buffer[0] = '0';
    buffer[1] = 'x';
    buffer[4] = 0;
    sprintf(&buffer[2], "%02X", byte);
    return std::string(buffer);

}

//------------------------------------------------------------------------------
// Name:        byte_to_hex
// Description: Converts a vector of bytes into a hex formatted string. The
//              string is formatted as per the following example:
//                  0x00 0x01 0x11 0xAA 0xFF
// Arguments:   - bytes: vector of bytes to be converted to a string
//              - formatted: (Optional) If false, returns string formatted as
//                  000111AAFF
// Returns:     Hex formatted representation of the bytes
//------------------------------------------------------------------------------
std::string byte_to_hex(std::vector<uint8_t> bytes, bool formatted)
{
    // String to return
    std::string str;

    // Handle the special case that the hex string should be unformatted
    if(!formatted)
    {
        for (size_t i = 0; i < bytes.size(); i++)
        {
            char buffer[3];
            buffer[2] = 0;
            sprintf(&buffer[0], "%02X", bytes.at(i));
            str += std::string(buffer);
        }
        return str;
    }

    // Format the hex string by default
    for (size_t i = 0; i < bytes.size(); i++)
    {
        str += byte_to_hex(bytes.at(i));
        str += " ";
    }
    return str;
}

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
std::vector<uint8_t> hex_to_byte(std::string hex, bool formatted)
{

    std::vector<uint8_t> bytes;

    if (formatted)
    {
        std::vector<std::string> split_str = avl::split(hex, " ");
        for (size_t i = 0; i < split_str.size(); i++)
        {
            std::string byte_str = split_str.at(i).substr(2, 2);
            uint8_t byte = (uint8_t) strtol(byte_str.c_str(), NULL, 16);
            bytes.push_back(byte);
        }
    }

    else
    {
        for (size_t i = 0; i < hex.length(); i += 2)
        {
            std::string byte_str = hex.substr(i, 2);
            uint8_t byte = (uint8_t) strtol(byte_str.c_str(), NULL, 16);
            bytes.push_back(byte);
        }
    }

    return bytes;

}

//------------------------------------------------------------------------------
// Name:        to_bytes
// Description: Converts a std::string to a vector of bytes.
// Arguments:   - string: String to be converted to bytes.
// Returns:     Vector of bytes representing the string.
//------------------------------------------------------------------------------
std::vector<uint8_t> to_bytes(std::string string)
{
    return std::vector<uint8_t>(string.begin(), string.end());
}

//------------------------------------------------------------------------------
// Name:        is_ascii
// Description: Checks whether a vector of bytes is a printable ASCII string.
// Arguments:   - bytes: Bytes to check.
// Returns:     True if the bytes are a printable ASCII string, false if they
//              are not.
//------------------------------------------------------------------------------
bool is_ascii(std::vector<uint8_t> bytes)
{
    for (const uint8_t& byte : bytes)
        if ((byte & ~0x7f) != 0 || !(isprint(byte) || isspace(byte)))
            return false;
    return true;
}

}
