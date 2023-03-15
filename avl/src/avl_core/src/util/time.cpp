//==============================================================================
// Autonomous Vehicle Library
//
// Description: A miscellaneous collection of utility functions that are not
//              sorted into the more specific utility function files.
//==============================================================================

#include "util/time.h"

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

namespace avl
{

//------------------------------------------------------------------------------
// Name:        get_epoch_time
// Description: Gets a high precision clock time since epoch in seconds.
// Returns:     Time since epoch in seconds.
//------------------------------------------------------------------------------
double get_epoch_time()
{
    using clock = std::chrono::high_resolution_clock;
    return std::chrono::duration<double>(
        clock::now().time_since_epoch()).count();
}

//------------------------------------------------------------------------------
// Name:        get_epoch_time_nanoseconds
// Description: Gets a high precision clock time since epoch in nanoseconds.
// Returns:     Time since epoch in nanoseconds.
//------------------------------------------------------------------------------
uint64_t get_epoch_time_nanoseconds()
{
    using namespace std::chrono;
    using clock = std::chrono::high_resolution_clock;
    nanoseconds ns = duration_cast<nanoseconds>(
        clock::now().time_since_epoch());
    return static_cast<uint64_t>(ns.count());
}

//------------------------------------------------------------------------------
// Name:        get_epoch_time_milliseconds
// Description: Gets a high precision clock time since epoch in milliseconds.
// Returns:     Time since epoch in milliseconds.
//------------------------------------------------------------------------------
uint64_t get_epoch_time_milliseconds()
{
    using namespace std::chrono;
    using clock = std::chrono::high_resolution_clock;
    milliseconds ms = duration_cast<milliseconds>(
        clock::now().time_since_epoch());
    return static_cast<uint64_t>(ms.count());
}

//------------------------------------------------------------------------------
// Name:        get_epoch_time_seconds
// Description: Gets a high precision clock time since epoch in seconds. This
//              function trucncates fractional seconds.
// Returns:     Time since epoch in seconds.
//------------------------------------------------------------------------------
uint32_t get_epoch_time_seconds()
{
    using namespace std::chrono;
    using clock = std::chrono::high_resolution_clock;
    seconds sec = duration_cast<seconds>(
        clock::now().time_since_epoch());
    return static_cast<uint32_t>(sec.count());
}

//------------------------------------------------------------------------------
// Name:        hms_to_seconds
// Description: Converts a time string whose format is HHmmss.ffff to seconds
//              since the start of the day.
// Arguments:   - hms: HHmmss.ffff formatted time string
// Returns:     Seconds since start of day.
//------------------------------------------------------------------------------
double hms_to_seconds(const std::string& hms)
{

    std::tm time_struct;
    std::istringstream ss(hms.substr(0,6));

    // Convert the time string to hours, minutes, and seconds and put those in
    // the time struct using the get_time function
    ss >> std::get_time(&time_struct, "%H%M%S");

    // Since std::tm cannot handle fractional time, we will just add the
    // fractional time on to the final result.
    double int_part;
    double fract_part = modf(std::stod(hms), &int_part);

    // Calculate the number of seconds from the time structure components. Add
    // the fractional component
    double seconds = time_struct.tm_hour * 3600
                   + time_struct.tm_min * 60
                   + time_struct.tm_sec
                   + fract_part;

    return seconds;

}

//------------------------------------------------------------------------------
// Name:        hms_to_epoch_time
// Description: Converts a time string whose format is HHmmss.ffff to seconds
//              since epoch, assuming the year, month, and day are the current
//              time.
// Arguments:   - hms: HHmmss.ffff formatted time string
// Returns:     Epoch time in seconds.
//------------------------------------------------------------------------------
double hms_to_epoch_time(const std::string& hms)
{

    // Get the number of since epoch of the that of the current day
    const double seconds_per_day = 86400;
    double day_epoch_time = get_epoch_time();
    day_epoch_time -= fmod(day_epoch_time, seconds_per_day);

    // Get seconds since the start of the day from the time string
    double seconds_into_day = hms_to_seconds(hms);

    // Return the epoch time in seconds as the the input number of seconds after
    // the epoch time of the start of the day
    return day_epoch_time + seconds_into_day;

}

}
