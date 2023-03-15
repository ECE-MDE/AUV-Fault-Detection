//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to files.
//==============================================================================

#include "util/file.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        file_exists
// Description: Checks to see if a file exists.
// Arguments:   - path: Path to file to check for existance.
// Returns:     True if the file exists, false if it does not.
//------------------------------------------------------------------------------
bool file_exists(const std::string& path)
{
    return static_cast<bool>(std::ifstream(path.c_str()));
}

}
