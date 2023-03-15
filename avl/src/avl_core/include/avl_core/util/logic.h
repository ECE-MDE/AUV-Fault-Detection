//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for generic logical operations.
//==============================================================================

#ifndef LOGIC_H
#define LOGIC_H

// C++ includes
#include <vector>
#include <algorithm>
#include <math.h>

namespace avl
{

//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        bool_to_string
// Description: Turns a boolean into a "true" or "false" string.
// Arguments:   - b: boolean to be turned into a string
// Returns:     "true" or "false" string.
//------------------------------------------------------------------------------
const char* bool_to_string(bool b);

//------------------------------------------------------------------------------
// Name:        any_true
// Description: Returns true if any of the elements in a vector of bools is
//              true.
// Returns:     True if any of the elements in a vector of bools is
//              true. Returns false if all elements are false.
//------------------------------------------------------------------------------
bool any_true(const std::vector<bool>& flags);

//------------------------------------------------------------------------------
// Name:        any_false
// Description: Returns true if any of the elements in a vector of bools is
//              false.
// Returns:     True if any of the elements in a vector of bools is
//              false. Returns false if all elements are true.
//------------------------------------------------------------------------------
bool any_false(const std::vector<bool>& flags);

//------------------------------------------------------------------------------
// Name:        all_true
// Description: Returns true if all of the elements in a vector of bools are
//              true.
// Returns:     True if all of the elements in a vector of bools are
//              true. Returns false if there are any false elements.
//------------------------------------------------------------------------------
bool all_true(const std::vector<bool>& flags);

//------------------------------------------------------------------------------
// Name:        all_false
// Description: Returns true if all of the elements in a vector of bools are
//              false.
// Returns:     True if all of the elements in a vector of bools are
//              false. Returns false if there are any true elements.
//------------------------------------------------------------------------------
bool all_false(const std::vector<bool>& flags);

//------------------------------------------------------------------------------
// Name:        has_both
// Description: Returns true if both variables are not NaN.
// Arguments:   - var1: First variable.
//              - var2: Second variable.
// Returns:     True if both variables are not NaN, false otherwise.
//------------------------------------------------------------------------------
bool has_both(double var1, double var2);

//==============================================================================
//                          TEMPLATE FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        all_nan
// Description: Returns true if all of the elements in a vector are NaN.
// Returns:     True if all of the elements in a vector are NaN. Returns false
//              if there are any non-NaN elements.
//------------------------------------------------------------------------------
template<typename T>
bool all_nan(const std::vector<T>& vec)
{
    return std::all_of(vec.begin(), vec.end(),
        [](T element) { return std::isnan(element); });
}

//------------------------------------------------------------------------------
// Name:        any_nan
// Description: Returns true if any of the elements in a vector are NaN.
// Returns:     True if any of the elements in a vector are NaN. Returns false
//              if there are no NaN elements.
//------------------------------------------------------------------------------
template<typename T>
bool any_nan(const std::vector<T>& vec)
{
    return std::any_of(vec.begin(), vec.end(),
        [](T element) { return std::isnan(element); });
}

} // namespace avl

#endif // LOGIC_H
