//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to ROS.
//==============================================================================

#ifndef UTIL_ROS_H
#define UTIL_ROS_H

// Standard ROS functions
#include "ros/ros.h"

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
uint8_t get_vehicle_id();

//==============================================================================
//                          TEMPLATE FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        check_param
// Description: Checks that a ROS parameter with the specified key exists on
//              the parameter server and its value can be read as the
//              specified data type.
// Arguments:   - key: ROS parameter key
// Returns:     true if the key exists and its value is the correct type,
//              false otherwise
//------------------------------------------------------------------------------
template <typename T>
bool check_param(const std::string key)
{

    T param_value;

    // Attempt to get the ROS parameter from the parameter server. A return
    // value of false means the get failed either because it wasn't found or
    // it wasn't able to get it as the given type
    return ros::param::get(key, param_value);

}

//------------------------------------------------------------------------------
// Name:        get_param
// Description: Gets a ROS parameter with the specified key as the specified
//              data type from the ROS parameter server. Prints an error if
//              the key is not found or has the incorrect type.
// Arguments:   - key: ROS parameter key
// Returns:     ROS parameter value as the given data type
//------------------------------------------------------------------------------
template <typename T>
T get_param(const std::string key)
{

    T param_value;

    // Attempt to get the ROS parameter from the parameter server. A return
    // value of false means the get failed either because it wasn't found or
    // it wasn't able to get it as the given type
    if (ros::param::getCached(key, param_value) == false)
    {
        std::string error_msg = std::string("get_param: failed to get "
            "parameter: ") + key.c_str();
        throw std::runtime_error(error_msg.c_str());
    }

    return param_value;

}

//------------------------------------------------------------------------------
// Name:        set_param
// Description: Sets a ROS parameter with the specified key as the specified
//              data type in the ROS parameter server. Prints an error if
//              the key is not found.
// Arguments:   - key: ROS parameter key
// Returns:     True if parameter value matches desired value
//------------------------------------------------------------------------------
template <typename T>
bool set_param(const std::string key, T value)
{

    // Attempt to set the parameter
    ros::param::set(key, value);

    // Pull the set value
    T verification_val = get_param<T>(key);

    // Return that the parameter was set correctly if it has the same
    // value that it was supposed to be set to.
    return verification_val == value;

}

} // namespace avl

#endif // UTIL_ROS_H
