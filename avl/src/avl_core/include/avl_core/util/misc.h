//==============================================================================
// Autonomous Vehicle Library
//
// Description: A miscellaneous collection of utility functions that are not
//              sorted into the more specific utility function files.
//==============================================================================

#ifndef MISC_H
#define MISC_H

// C++ includes
#include <string>
#include <sstream>
#include <iomanip>

namespace avl
{

//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        get_nmea_checksum
// Description: Calculates the checksum for an NMEA message. The $ character
//              and checksum characters in the message are ignored in the
//              calculation as per NMEA standards.
//------------------------------------------------------------------------------
std::string get_nmea_checksum(std::string message);

} // namespace avl

#endif // MISC_H
