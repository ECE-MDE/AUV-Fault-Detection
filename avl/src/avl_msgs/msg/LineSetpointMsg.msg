#===============================================================================
# Autonomous Vehicle Library
#
# Description: ROS message defining a line setpoint. All setpoint messages
#              must have a bool enable element.
#===============================================================================

# Indicates whether control is enabled or disabled
bool enable

# Start and end coordinates of the line in radians
float64 lat_start
float64 lon_start
float64 lat_end
float64 lon_end
