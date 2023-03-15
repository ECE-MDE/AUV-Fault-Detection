#===============================================================================
# Autonomous Vehicle Library
#
# Description: Describes a message containing the state of a vehicle.
#===============================================================================

# Propeller RPM
float64 rpm

# Euler angles to rotate from the NED frame to the body frame. These are the
# vehicle's roll, pitch, and heading. Units are radians.
float64 roll
float64 pitch
float64 yaw

# Vehicle body frame velocity relative to the earth frame, expressed in the body
# frame. This is the vehicle's ground-relative speed. Units are m/s.
float64 v_eb_b_x
float64 v_eb_b_y
float64 v_eb_b_z

# Vehicle body frame position expressed in geodetic coordinates. Units are
# radians for lat/lon and meters for alt.
float64 lat
float64 lon
float64 alt

# Vehicle body frame position expressed in a local cartesian frame with origin
# at lat0, lon0, and alt0. Units are radians for lat0/lon0 and meters for the
# rest.
float64 lat0
float64 lon0
float64 alt0
float64 r_b_n
float64 r_b_e
float64 r_b_d

# Vehicle body frame rotation rate relative to the inertial frame, expressed in
# the body frame. These are the values that a gyroscope would read if mounted to
# the vehicle body frame. Units are rad/s.
float64 w_ib_b_x
float64 w_ib_b_y
float64 w_ib_b_z

# Vehicle body frame acceleration relative to the inertial frame, expressed in
# the body frame. These are the values that an accelerometer would read if
# mounted to the vehicle body frame. Units are m/s^2.
float64 f_ib_b_x
float64 f_ib_b_y
float64 f_ib_b_z
