//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for manipulation of std::vectors
//==============================================================================

#include "util/vector.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        vec_from_string
// Description: Converts a string containing delimited numbers to a vector of
//              doubles.
// Arguments:   - str: String containing vector elements.
//              - delim: Delimiter between numbers.
// Returns:     Vector of doubles from the input string.
//------------------------------------------------------------------------------
std::vector<double> vec_from_string(const std::string& str, const char& delim)
{

    std::stringstream ss(str);
    std::vector<double> vec;
    std::string elem;

    // Loop through each data element in the string
    while (std::getline(ss, elem, delim))
        try
        {
            vec.push_back(std::stod(elem));
        }
        catch (const std::exception& ex)
        {
            vec.push_back(NAN);
        }

    return vec;

}

}
