#===============================================================================
# Autonomous Vehicle Library
#
# PURPOSE: Describes a Springnav INS BIST message containing information about
#          vehicle status information.
#
# AUTHOR:  cong
#===============================================================================

uint64 firmware_version      # firmware version
uint64 imu                   # IMU
uint64 comms                 # commands
uint64 cca                   # cca
uint16 ahrs                  # AHRS
uint64 ins                   # INS
