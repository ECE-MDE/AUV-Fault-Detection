//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for generic mathematical operations.
//==============================================================================

#ifndef MATH_H
#define MATH_H

// C++ includes
#include <vector>
#include <algorithm>
#include <math.h>

namespace avl
{

//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        wrap_to_pi
// Description: Wraps an angle in radians to the range -pi to pi.
// Arguments:   - angle: angle in radians to be wrapped.
// Returns:     Wrapped angle in radians.
//------------------------------------------------------------------------------
double wrap_to_pi(double angle);

//------------------------------------------------------------------------------
// Name:        wrap_to_2pi
// Description: Wraps an angle in radians to the range 0 to 2*pi.
// Arguments:   - angle: angle in radians to be wrapped.
// Returns:     Wrapped angle in radians.
//------------------------------------------------------------------------------
double wrap_to_2pi(double angle);

//------------------------------------------------------------------------------
// Name:        wrap_to_180
// Description: Wraps an angle in degrees to the range -180 to 180.
// Arguments:   - angle: angle in degrees to be wrapped.
// Returns:     Wrapped angle in degrees.
//------------------------------------------------------------------------------
double wrap_to_180(double angle);

//------------------------------------------------------------------------------
// Name:        wrap_to_360
// Description: Wraps an angle in degrees to the range 0 to 360.
// Arguments:   - angle: angle in degrees to be wrapped.
// Returns:     Wrapped angle in degrees.
//------------------------------------------------------------------------------
double wrap_to_360(double angle);

//------------------------------------------------------------------------------
// Name:        round_decimals
// Description: Rounds a double to a given number of decimals.
// Arguments:   - value: Number to be rounded.
//              - decimals: Number of decimals to round to.
// Returns:     Rounded number.
//------------------------------------------------------------------------------
double round_decimals(double value, unsigned char decimals);

//------------------------------------------------------------------------------
// Name:        clamp
// Description: Clamps a value between the minimum and maximum value.
// Arguments:   - input: value to be scaled
//              - min: minimum value of input's range
//              - max: maximum value of input's range
// Returns:     input value clamped to the range
//------------------------------------------------------------------------------
double clamp(double input, double min, double max);


//------------------------------------------------------------------------------
// Name:        linear_scale
// Description: Scales a value linearly from the input range to the output
//              range. The linear scale will still work for values beyond the
//              input min and max.
// Arguments:   - input: value to be scaled
//              - in_min: minimum value of input's range
//              - in_max: maximum value of input's range
//              - out_min: minimum value of new output range
//              - out_max: maximum value of new output range
// Returns:     input value scaled linearly
//------------------------------------------------------------------------------
double linear_scale(double input, double in_min, double in_max,
    double out_min, double out_max);

//------------------------------------------------------------------------------
// Name:        point_in_poly
// Description: Determines if the provided point (lat/lon) is inside a polygon
//              defined by a vector of polygon vertex locations in lat/lon. This
//              functions uses a "ray casting" algorithm testing to the "right".
//              This algorithm counts the number of times a ray along the
//              longitude line crosses the line segments connecting each pair of
//              the polygon vertices. If it is an odd number, the point is
//              inside the polygon. An even number of crossings means the point
//              is outside. This function is adapted directly from:
//                  https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
//              NOTE: This function works for points in either degrees or
//              radians, but all arguments must be in the same unit.
//              NOTE: This function works for all polygons, including concave
//              polygons.
// Arguments:   - lat: test point latitude (degrees or radians)
//              - lon: test point longitude (degrees or radians)
//              - poly_lats: polygon vertex latitudes (degrees or radians)
//              - poly_lons: polygon vertex longitudes (degrees or radians)
// Returns:     True if the test point lies within the polygon, false otherwise.
//------------------------------------------------------------------------------
bool point_in_poly(const double lat, const double lon,
                          const std::vector<double> poly_lats,
                          const std::vector<double> poly_lons);

//==============================================================================
//                          TEMPLATE FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        sec
// Description: Calculates the secant of an angle in radians.
// Arguments:   - angle: angle in radians
// Returns:     Secant of angle.
//------------------------------------------------------------------------------
template<typename T>
T sec(T angle)
{
    return 1.0 / cos(angle);
}

//------------------------------------------------------------------------------
// Name:        cot
// Description: Calculates the cotangent of an angle in radians.
// Arguments:   - angle: angle in radians
// Returns:     Cotangent of angle.
//------------------------------------------------------------------------------
template<typename T>
T cot(T angle)
{
    return cos(angle) / sin(angle);
}

//------------------------------------------------------------------------------
// Name:        deg_to_rad
// Description: Converts an angle in degrees to an angle in radians.
// Arguments:   - angle: angle in degrees to be converted to radians.
// Returns:     Angle in radians.
//------------------------------------------------------------------------------
template<typename T>
T deg_to_rad(T angle)
{
    return angle*M_PI/180.0;
}

//------------------------------------------------------------------------------
// Name:        rad_to_deg
// Description: Converts an angle in radians to an angle in degrees.
// Arguments:   - angle: angle in radians to be converted to degrees.
// Returns:     Angle in degrees.
//------------------------------------------------------------------------------
template<typename T>
T rad_to_deg(T angle)
{
    return angle*180.0/M_PI;
}

//------------------------------------------------------------------------------
// Name:        arc_angle
// Description: Determine the angular distance from theta1 to theta2 in the
//              specified clockwise or counterclockwise direction.
// Arguments:   - theta1: First input angle.
//              - theta2: Second input angle.
//              - direction: 0 for clockwise, 1 for counterclockwise.
//              - radians: True if units are radians, false if degrees
// Returns:     the minimum angular distance from theta_1 to theta_2
//              in the clockwise direction.
//------------------------------------------------------------------------------
template<typename T>
T arc_angle(T theta1, T theta2, bool clockwise, bool radians=true)
{
    T full_rotation = (radians) ? 2.0*M_PI : 360.0;
    if (clockwise)
        return std::fmod(full_rotation + theta1 - theta2, full_rotation);
    else
        return std::fmod(full_rotation - theta1 + theta2, full_rotation);
}

//------------------------------------------------------------------------------
// Name:        wrap_cw
// Description: Determine the minimum angular distance from theta_1 to theta_2
//              theta increases in the clockwise direction.
//              in the clockwise direction
// Arguments:   - theta_1: first input angle
//              - theta_2: second input angle
//              - radians: units
// Returns:     the minimum angular distance from theta_1 to theta_2
//              in the clockwise direction.
//------------------------------------------------------------------------------
template<typename T>
T wrap_cw(T theta_1, T theta_2, bool radians = true)
{

    T full_rotation;
    if(radians)
    {
        full_rotation = 2.0*M_PI;
    }
    else
    {
        full_rotation = 360.0;
    }
    return std::fmod(full_rotation + theta_1 - theta_2,full_rotation);

}

//------------------------------------------------------------------------------
// Name:        wrap_ccw
// Description: Determine the minimum angular distance from theta_1 to theta_2
//              in the counterclockwise direction
// Arguments:   - theta_1: first input angle
//              - theta_2: second input angle
//              - radians: units
// Returns:     the minimum angular distance from theta_1 to theta_2
//              in the counterclockwise direction.
//------------------------------------------------------------------------------
template<typename T>
T wrap_ccw(T theta_1, T theta_2, bool radians = true)
{

    T full_rotation;
    if(radians)
    {
        full_rotation = 2.0*M_PI;
    }
    else
    {
        full_rotation = 360.0;
    }

    return std::fmod(full_rotation - theta_1 + theta_2,full_rotation);
}

//------------------------------------------------------------------------------
// Name:        in_range
// Description: Checks whether a value is within the bounds, inclusive.
// Arguments:   - value: value to check against bounds
//              - min: minimum allowable value
//              - max: maximum allowable value
// Returns:     True if value is within bounds, false otherwise.
//------------------------------------------------------------------------------
template<typename T>
bool in_range(T value, T min, T max)
{
    return (value >= min) && (value <= max);
}

//------------------------------------------------------------------------------
// Name:        wrap_deg
// Description: Finds the shortest angular distance between 2 angles.
//                Assumes radians by default
// Arguments:   - angle: angle to wrap
//              - radians: True if input is in radians
// Returns:     wrapped angle.
//------------------------------------------------------------------------------
template<typename T>
T wrap(T angle, bool radians=true)
{
    // Maximum allowable absolute value of angle in radians
    T maximum;

    if(radians)
    {
        maximum = M_PI; // Confine the output to be between -pi and pi
    }
    else
    {
        maximum = 180.0; // Confine the output to be between -180.0 and 180.0
    }

    // While the angle is greater than the maximum allowable
    while(fabs(angle) > maximum)
    {
        // If angle is positive, subtract 1 full rotation, else add 1 rotation
        angle -= (angle > 0)? 2*maximum : -2*maximum;
    }

    return angle;
}

//------------------------------------------------------------------------------
// Name:        round_up_to_multiple
// Description: Rounds a number up to the next nearest multiple.
// Arguments:   - value: Value to round up.
//              - multiple: Multiple to round to.
// Returns:     Rounded up multiple of value.
//------------------------------------------------------------------------------
template<typename T>
T round_up_to_multiple( T value, T multiple )
{
    if (multiple == 0) return value;

    double value_d = static_cast<double>(value);
    double multiple_d = static_cast<double>(multiple);
    double quot = std::ceil(value_d / multiple_d);

    return static_cast<T>(quot * multiple_d);
}

} // namespace avl

#endif // MATH_H
