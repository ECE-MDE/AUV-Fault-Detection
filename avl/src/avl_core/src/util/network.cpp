//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to networking.
//==============================================================================

#include "util/network.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        get_local_ip_address
// Description: Gets the local IP address. Returns the first interface
//              address that is not 127.0.0.1, or returns 127.0.0.1 if there
//              are no other interfaces. Throws an exception if unable to
//              get any interface.
// Returns:     Local IP address string.
//------------------------------------------------------------------------------
std::string get_local_ip_address()
{

    std::string ip_address = "127.0.0.1";
    struct ifaddrs* interface_list = NULL;
    struct ifaddrs* interface = NULL;

    // get a linked list of structures describing the network interfaces of
    // the local system, returns 0 on success
    int success = getifaddrs(&interface_list);
    if (success == 0)
    {

        // Loop through linked list of interfaces
        interface = interface_list;
        while(interface != NULL)
        {

            // We are only interested in AF_INET interfaces
            if(interface->ifa_addr->sa_family == AF_INET)
            {

                // Get the interface address and check if it is not the default
                std::string interface_address = inet_ntoa(
                    ((struct sockaddr_in*)interface->ifa_addr)->sin_addr);
                if (interface_address != "127.0.0.1")
                {
                    ip_address = interface_address;
                    break;
                }

            }

            // Move to the next interface
            interface = interface->ifa_next;

        }

    }
    else
    {
        throw std::runtime_error("get_local_ip_address: unable to get local "
            "IP address");
    }

    // Free memory
    freeifaddrs(interface_list);

    return ip_address;

}

}
