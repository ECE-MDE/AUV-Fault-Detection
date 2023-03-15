#===============================================================================
# Autonomous Vehicle Library
#
# Description: ROS message defining a float64 valued setpoint. All setpoint
#              messages must have a bool enable element.
#===============================================================================

# Indicates whether control is enabled or disabled
bool enable

# Setpoint double value
float64 data
