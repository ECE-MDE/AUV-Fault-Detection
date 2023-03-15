#===============================================================================
# Autonomous Vehicle Library
#
# Description: Describes a IMU message containing, angular velocity, linear
#              acceleration, temperature and a valid data flag. This IMU
#              message is specific to the KVH 1725 IMU.
#===============================================================================

float64 dt                                    # Sampling period
geometry_msgs/Vector3 angular_velocity        # x, y, z components in (rad/s)
geometry_msgs/Vector3 linear_acceleration     # x, y, z components in (m/s^2)
geometry_msgs/Vector3 magnetic_flux           # x, y, z components in (Gauss)
int32 temperature                             # Temperature
bool valid                                    # Valid data flag
