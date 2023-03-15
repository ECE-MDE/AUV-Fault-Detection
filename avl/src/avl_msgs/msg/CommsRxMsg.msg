#===============================================================================
# Autonomous Vehicle Library
#
# Description: Defines a message that contains a packet that was received
#              by the comms manager node and information about the packet's
#              source.
#===============================================================================

# Channel that the message was received through
uint8 channel

# Interface that the message was received from
uint8 interface

# Message data
uint8[] data
