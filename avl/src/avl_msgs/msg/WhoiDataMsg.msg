#===============================================================================
# Autonomous Vehicle Library
#
# Description: Describes a WHOI micromodem message containing data received by
#              the micromodem from another micromodem and its time of arrival.
#===============================================================================

float64 time_of_departure # Time of departure of data packet in epoch seconds
float64 time_of_arrival   # Time of arrival of data packet in epoch seconds
uint8[] data              # Message data
