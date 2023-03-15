#===============================================================================
# Autonomous Vehicle Library
#
# PURPOSE: Describes a Xero GPS message containing information about a GPS fix.
#
# AUTHOR:  Cong
#===============================================================================

int64 position_type      # Position type
float64 lat              # Latitude in decimal degrees
float64 lon              # Longitude in decimal degrees
float64 alt              # Altitude in meters
int64 fix_quality        # GPS fix quality (0 = invalid, 1 = GPS fix, etc) See NMEA standard GGA sentence
int64 num_sats           # Number of tracked satellites
int64 max_SNR            # Maximum SNR of fix
int64 time_fix           # time to fix
float64 HDOP_fix         # HDOP ((Horizontal Dilution of Precision)) to fix
float64 loaded_voltage   # loaded_voltage
float64 unloaded_voltage # unloaded_voltage
