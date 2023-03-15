//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to networking.
//==============================================================================

#ifndef NETWORK_H
#define NETWORK_H

// C++ includes
#include <string>
#include <stdexcept>

// Network interface functionality
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace avl
{


//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        get_local_ip_address
// Description: Gets the local IP address. Returns the first interface
//              address that is not 127.0.0.1, or returns 127.0.0.1 if there
//              are no other interfaces. Throws an exception if unable to
//              get any interface.
// Returns:     Local IP address string.
//------------------------------------------------------------------------------
std::string get_local_ip_address();

} // namespace avl

#endif // NETWORK_H
