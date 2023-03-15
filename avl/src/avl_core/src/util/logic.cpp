//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for generic logical operations.
//==============================================================================

#include "util/logic.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        bool_to_string
// Description: Turns a boolean into a "true" or "false" string.
// Arguments:   - b: boolean to be turned into a string
// Returns:     "true" or "false" string.
//------------------------------------------------------------------------------
const char* bool_to_string(bool b)
{
  return b ? "true" : "false";
}

//------------------------------------------------------------------------------
// Name:        any_true
// Description: Returns true if any of the elements in a vector of bools is
//              true.
// Returns:     True if any of the elements in a vector of bools is
//              true. Returns false if all elements are false.
//------------------------------------------------------------------------------
bool any_true(const std::vector<bool>& flags)
{
    return std::any_of(flags.begin(), flags.end(),
        [](bool b) { return b==true; });
}

//------------------------------------------------------------------------------
// Name:        any_false
// Description: Returns true if any of the elements in a vector of bools is
//              false.
// Returns:     True if any of the elements in a vector of bools is
//              false. Returns false if all elements are true.
//------------------------------------------------------------------------------
bool any_false(const std::vector<bool>& flags)
{
    return std::any_of(flags.begin(), flags.end(),
        [](bool b) { return b==false; });
}

//------------------------------------------------------------------------------
// Name:        all_true
// Description: Returns true if all of the elements in a vector of bools are
//              true.
// Returns:     True if all of the elements in a vector of bools are
//              true. Returns false if there are any false elements.
//------------------------------------------------------------------------------
bool all_true(const std::vector<bool>& flags)
{
    return std::all_of(flags.begin(), flags.end(),
        [](bool b) { return b==true; });
}

//------------------------------------------------------------------------------
// Name:        all_false
// Description: Returns true if all of the elements in a vector of bools are
//              false.
// Returns:     True if all of the elements in a vector of bools are
//              false. Returns false if there are any true elements.
//------------------------------------------------------------------------------
bool all_false(const std::vector<bool>& flags)
{
    return std::all_of(flags.begin(), flags.end(),
        [](bool b) { return b==false; });
}

//------------------------------------------------------------------------------
// Name:        has_both
// Description: Returns true if both variables are not NaN.
// Arguments:   - var1: First variable.
//              - var2: Second variable.
// Returns:     True if both variables are not NaN, false otherwise.
//------------------------------------------------------------------------------
bool has_both(double var1, double var2)
{
    return !std::isnan(var1) && !std::isnan(var2);
}

}
