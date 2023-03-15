#===============================================================================
# Autonomous Vehicle Library
#
# Description: ROS message defining an orbit setpoint. All setpoint messages
#              must have a bool enable element.
#===============================================================================

# Indicates whether control is enabled or disabled
bool enable

# Start and end coordinates of the line in radians
float64 lat
float64 lon

# Orbit radius in meters
float64 radius

# Indicates orbit direction when viewed from above
bool clockwise
