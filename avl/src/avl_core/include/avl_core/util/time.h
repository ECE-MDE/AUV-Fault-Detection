//==============================================================================
// Autonomous Vehicle Library
//
// Description: A miscellaneous collection of utility functions that are not
//              sorted into the more specific utility function files.
//==============================================================================

#ifndef TIME_H
#define TIME_H

// C++ includes
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace avl
{

//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        get_epoch_time
// Description: Gets a high precision clock time since epoch in seconds.
// Returns:     Time since epoch in seconds.
//------------------------------------------------------------------------------
double get_epoch_time();

//------------------------------------------------------------------------------
// Name:        get_epoch_time_nanoseconds
// Description: Gets a high precision clock time since epoch in nanoseconds.
// Returns:     Time since epoch in nanoseconds.
//------------------------------------------------------------------------------
uint64_t get_epoch_time_nanoseconds();

//------------------------------------------------------------------------------
// Name:        get_epoch_time_milliseconds
// Description: Gets a high precision clock time since epoch in milliseconds.
// Returns:     Time since epoch in milliseconds.
//------------------------------------------------------------------------------
uint64_t get_epoch_time_milliseconds();

//------------------------------------------------------------------------------
// Name:        get_epoch_time_seconds
// Description: Gets a high precision clock time since epoch in seconds. This
//              function trucncates fractional seconds.
// Returns:     Time since epoch in seconds.
//------------------------------------------------------------------------------
uint32_t get_epoch_time_seconds();

//------------------------------------------------------------------------------
// Name:        hms_to_seconds
// Description: Converts a time string whose format is HHmmss.ffff to seconds
//              since the start of the day.
// Arguments:   - hms: HHmmss.ffff formatted time string
// Returns:     Seconds since start of day.
//------------------------------------------------------------------------------
double hms_to_seconds(const std::string& hms);

//------------------------------------------------------------------------------
// Name:        hms_to_epoch_time
// Description: Converts a time string whose format is HHmmss.ffff to seconds
//              since epoch, assuming the year, month, and day are the current
//              time.
// Arguments:   - hms: HHmmss.ffff formatted time string
// Returns:     Epoch time in seconds.
//------------------------------------------------------------------------------
double hms_to_epoch_time(const std::string& hms);

} // namespace avl

#endif // TIME_H
