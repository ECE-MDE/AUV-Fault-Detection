//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for std::string manipulation.
//==============================================================================

#include "util/string.h"

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
std::string format_string(const char* format, va_list args)
{

    // Create a copy of the va list so we don't change the original when
    // determining the required buffer size
    va_list args_copy;
    va_copy(args_copy, args);

    // Determine the required buffer size by pretending to format the string but
    // not giving it a buffer. The function will return the number of characters
    // not counting the terminating null character. We must add one to the size
    // to account for the terminating null character
    int size = vsnprintf(nullptr, 0, format, args_copy) + 1;
    if (size < 0)
        throw std::runtime_error("format_string: format failed");
    std::vector<char> buf(static_cast<size_t>(size));

    // Do the formatting
    vsprintf(buf.data(), format, args);

    // Remove the terminating null character when constructing the string
    return std::string(buf.data(), static_cast<size_t>(size) - 1);

}

//------------------------------------------------------------------------------
// Name:        split
// Description: Splits a string into a vector of strings according the the given
//              delimeter string. The delimeter is removed from the strings.
// Arguments:   - str: string to be split
//              - delim: split delimeter
// Returns:     a vector of split strings
//------------------------------------------------------------------------------
std::vector<std::string> split(const std::string& str,
    const std::string& delim)
{
    std::vector<std::string> elems;
    boost::algorithm::split(elems, str, boost::is_any_of(delim));
    return elems;
}

//------------------------------------------------------------------------------
// Name:        contains
// Description: Checks whether or not a string contains a substring
// Arguments:   - string: string to search for substring
//              - substring: substring to check for
// Returns:     True if the string contains the substring, false otherwise
//------------------------------------------------------------------------------
bool contains(std::string string, std::string substring)
{
    return string.find(substring) != std::string::npos;
}

//------------------------------------------------------------------------------
// Name:        starts_with
// Description: Checks whether or not a string starts with a substring
// Arguments:   - string: string to search for substring
//              - substring: substring to check for
// Returns:     True if the string starts with the substring, false otherwise
//------------------------------------------------------------------------------
bool starts_with(std::string string, std::string substring)
{
    return string.compare(0, substring.size(), substring) == 0;
}

//------------------------------------------------------------------------------
// Name:        strip
// Description: Removes all instances of a character from a string.
// Arguments:   - string: reference to string to remove character from
//              - character: character to remove
//------------------------------------------------------------------------------
void strip(std::string& string, char character)
{
    string.erase(std::remove(string.begin(), string.end(), character),
        string.end());
}

//------------------------------------------------------------------------------
// Name:        to_upper
// Description: Converts all characters in the string to upper case.
// Arguments:   - string: reference to string to convert to upper case
//------------------------------------------------------------------------------
void to_upper(std::string& string)
{
    std::transform(string.begin(), string.end(), string.begin(), ::toupper);
}

//------------------------------------------------------------------------------
// Name:        to_lower
// Description: Converts all characters in the string to lower case.
// Arguments:   - string: reference to string to convert to lower case
//------------------------------------------------------------------------------
void to_lower(std::string& string)
{
    std::transform(string.begin(), string.end(), string.begin(), ::tolower);
}

}
