#===============================================================================
# Autonomous Vehicle Library
#
# Description: Definition of a data receive message. For use by the various
#              communication channel nodes to publish received data to the
#              comms manager node.
#===============================================================================

# Interface the message was received from
uint8 interface

# Data received on the channel
uint8[] data
