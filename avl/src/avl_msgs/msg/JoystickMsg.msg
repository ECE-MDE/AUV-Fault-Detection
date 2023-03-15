#===============================================================================
# Autonomous Vehicle Library
#
# PURPOSE: Describes a joystick message containing information about a joystick
#          event.
#
# AUTHOR:  stkrauss
#===============================================================================

int32 number   # Axis/button number
float32 value  # Axis/button value (axes are -1.0 to 1.0, buttons are 0.0 or 1.0)
bool is_button # True if the event is for a button
bool is_axis   # True if the event is for an axis
