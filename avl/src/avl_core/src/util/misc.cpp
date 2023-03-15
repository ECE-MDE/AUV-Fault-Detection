//==============================================================================
// Autonomous Vehicle Library
//
// Description: A miscellaneous collection of utility functions that are not
//              sorted into the more specific utility function files.
//==============================================================================

#include "util/misc.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        get_nmea_checksum
// Description: Calculates the checksum for an NMEA message. The $ character
//              and checksum characters in the message are ignored in the
//              calculation as per NMEA standards.
//------------------------------------------------------------------------------
std::string get_nmea_checksum(std::string message)
{

    int checksum = 0;

    // Calculate the checksum. Start at index 1 to skip the $ at the start
    // of an NMEA message
    for (size_t i = 1; i < message.length(); i++)
    {

        // End the calculation when the checksum indicator * is reached
        if(message.at(i) == '*')
            break;

        // XOR the character
        checksum ^= static_cast<int>(message.at(i));

        // Ensure the checksum is two bytes
        checksum &= 0xFF;

    }

    // Convert the checksum integer to a hex string
    std::stringstream stream;
    stream << std::uppercase << std::setfill('0') << std::setw(2)
           << std::hex << checksum;
    std::string crc_string(stream.str());

    return "*" + crc_string;

}

}
