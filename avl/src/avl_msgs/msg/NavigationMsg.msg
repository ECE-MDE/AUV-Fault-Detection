#===============================================================================
# Autonomous Vehicle Library
#
# Description: Navigation message with information about vehicle position,
#              orientation, and velocities. If a value is not known, use NaN.
#===============================================================================

float64 roll        # Roll in radians
float64 pitch       # Pitch in radians
float64 yaw         # Yaw in radians
float64 vn          # North velocity in m/s
float64 ve          # East velocity in m/s
float64 vd          # Down velocity in m/s
float64 lat         # Latitude in radians
float64 lon         # Longitude in radians
float64 alt         # Altitude in meters
float64 yaw_std     # Yaw standard deviation in radians
float64 avg_pos_err # Altitude in meters
