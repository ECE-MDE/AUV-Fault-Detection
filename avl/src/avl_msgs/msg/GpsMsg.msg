#===============================================================================
# Autonomous Vehicle Library
#
# Description: Describes a GPS message containing information about a GPS fix.
#===============================================================================

float64 lat           # Latitude in radians
float64 lon           # Longitude in radians
float64 alt           # Altitude in meters
float64 ground_speed  # Ground speed in m/s
float64 track_angle   # Track angle in radians
int8 fix_quality      # GPS fix quality (0 = invalid, 1 = GPS fix, etc.)
int8 num_sats         # Number of tracked satellites
int8 hdop             # Horizontal dilution of position
float64 stddev_lat    # Latitude standard deviation in m, 1-sigma
float64 stddev_lon    # Longitude standard deviation in m, 1-sigma
float64 stddev_alt    # Altitude standard deviation in m, 1-sigma
