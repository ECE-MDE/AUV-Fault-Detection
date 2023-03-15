//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to ROS.
//==============================================================================

#include "util/ros.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        get_vehicle_id
// Description: Gets the vehicle ID, which is formed by the vehicle computer's
//              local IP address.
// Returns:     Vehicle ID number.
//------------------------------------------------------------------------------
uint8_t get_vehicle_id()
{
    return static_cast<uint8_t>(get_param<int>("vehicle_id"));
}

}
