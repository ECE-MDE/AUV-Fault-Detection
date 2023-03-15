//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for std::string manipulation.
//==============================================================================

#ifndef STRING_H
#define STRING_H

// C++ includes
#include <string>
#include <sstream>
#include <vector>
#include <cstdarg>
#include <stdexcept>
#include <algorithm>

// Boost includes
#include <boost/algorithm/string.hpp>

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        format_string
// Description: Formats a string using a printf-style argument list.
// Arguments:   - format: Printf style format string.
//              - args: Printf style list of variables to be formatted.
// Returns:     Formatted string.
//------------------------------------------------------------------------------
std::string format_string(const char* format, va_list args);

//------------------------------------------------------------------------------
// Name:        split
// Description: Splits a string into a vector of strings according the the given
//              delimeter string. The delimeter is removed from the strings.
// Arguments:   - str: string to be split
//              - delim: split delimeter
// Returns:     a vector of split strings
//------------------------------------------------------------------------------
std::vector<std::string> split(const std::string& str,
    const std::string& delim);

//------------------------------------------------------------------------------
// Name:        contains
// Description: Checks whether or not a string contains a substring
// Arguments:   - string: string to search for substring
//              - substring: substring to check for
// Returns:     True if the string contains the substring, false otherwise
//------------------------------------------------------------------------------
bool contains(std::string string, std::string substring);

//------------------------------------------------------------------------------
// Name:        starts_with
// Description: Checks whether or not a string starts with a substring
// Arguments:   - string: string to search for substring
//              - substring: substring to check for
// Returns:     True if the string starts with the substring, false otherwise
//------------------------------------------------------------------------------
bool starts_with(std::string string, std::string substring);

//------------------------------------------------------------------------------
// Name:        strip
// Description: Removes all instances of a character from a string.
// Arguments:   - string: reference to string to remove character from
//              - character: character to remove
//------------------------------------------------------------------------------
void strip(std::string& string, char character);

//------------------------------------------------------------------------------
// Name:        to_upper
// Description: Converts all characters in the string to upper case.
// Arguments:   - string: reference to string to convert to upper case
//------------------------------------------------------------------------------
void to_upper(std::string& string);

//------------------------------------------------------------------------------
// Name:        to_lower
// Description: Converts all characters in the string to lower case.
// Arguments:   - string: reference to string to convert to lower case
//------------------------------------------------------------------------------
void to_lower(std::string& string);

} // namespace avl

#endif // STRING_H
