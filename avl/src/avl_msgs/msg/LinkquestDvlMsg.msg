#===============================================================================
# Autonomous Vehicle Library
#
# PURPOSE: Describes a DVL message containing linear velocity, water current
#          velocity, 4 beam altitudes, and flags.
#          This DVL message is specific to the LinkQuest NavQuest 600 Micro DVL
#
#          The flags are used for the following reasons:
#          linear velocity flag:
#             flag = 2 water velocity (no valid bottom velocity)
#             flag = 1 bottom velocity
#             flag = 0 not valid
#          water current velocity flag:
#             flag = 2 valid measurement
#             flag = 0 not valid
#          4 beam flags:
#             flag = 1 good measurement
#             flag = 0 bad measurement
#
# AUTHOR:  sgibson2
#===============================================================================

geometry_msgs/Vector3 linear_velocity         # x, y, z components in (mm/sec)
geometry_msgs/Vector3 water_current_velocity  # x, y, z components in (mm/sec)
float32[] beam_altitude                       # in meters
uint8 linear_velocity_flag                    #
uint8 water_current_velocity_flag             #
uint8[] beam_flag                             #
