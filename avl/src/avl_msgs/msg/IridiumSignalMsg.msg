#===============================================================================
# Autonomous Vehicle Library
#
# Description: Describes a message containing information about Iridium signal
#              strength and availability.
#===============================================================================

bool service_available   # True if Iridium service is available, false if not
uint8 signal_strength           # Iridium signal strength (1-5)
