#===============================================================================
# Autonomous Vehicle Library
#
# Description: Describes a message used to start a fin calibration.
#===============================================================================

uint16 fin_id        # Fin ID (0 = top, 1 = starboard, 2 = bottom, 3 = port)
float32 min_angle    # Fin minimum angle in degrees
float32 max_angle    # Fin maximum angle in degrees
float32 home_angle   # Fin home angle in degrees
