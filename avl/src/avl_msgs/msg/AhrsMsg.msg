#===============================================================================
# Autonomous Vehicle Library
#
# Description: Describes an AHRS message containing Euler angles, angular
#              velocity, linear acceleration, and magnetic field of the AHRS in
#              the AHRS x-, y-, and z- axes.
#===============================================================================

geometry_msgs/Vector3 theta # AHRS Euler angles in radians
geometry_msgs/Vector3 w     # AHRS angular velocity in rad/sec
geometry_msgs/Vector3 a     # AHRS linear acceleration in m/s^2
geometry_msgs/Vector3 mag   # AHRS magnetic field in gauss
