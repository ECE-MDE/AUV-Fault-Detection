#===============================================================================
# Autonomous Vehicle Library
#
# Description: Ranging message containing information about a range to a
#              given location.
#===============================================================================

uint8 src_id    # Source ID of the range measurement
float64 lat     # Latitude in decimal degrees
float64 lon     # Longitude in decimal degrees
float64 alt     # Altitude in meters
float64 range   # Range in meters
