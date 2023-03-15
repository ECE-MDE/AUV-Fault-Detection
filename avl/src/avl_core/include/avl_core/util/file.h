//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to files.
//==============================================================================

#ifndef FILE_H
#define FILE_H

// C++ includes
#include <string>
#include <fstream>

namespace avl
{

//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        file_exists
// Description: Checks to see if a file exists.
// Arguments:   - path: Path to file to check for existance.
// Returns:     True if the file exists, false if it does not.
//------------------------------------------------------------------------------
bool file_exists(const std::string& path);

} // namespace avl

#endif // FILE_H
