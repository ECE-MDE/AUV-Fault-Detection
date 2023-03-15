#===============================================================================
# Autonomous Vehicle Library
#
# PURPOSE: Describes a Fins message containing fin angles in degrees relative to
#          the fin's local axis. Follows the right hand rule by pointing the
#          thumb along the fin axis away from the vehicle, and the fingers curl
#          along the positive angle direction.
#
# AUTHOR:  stkrauss
#===============================================================================

float64 port      # Port fin angle
float64 starboard # Starboard fin angle
float64 top       # Top fin angle
float64 bottom    # Bottom fin angle
