//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions for generic mathematical operations.
//==============================================================================

#include "util/math.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        wrap_to_pi
// Description: Wraps an angle in radians to the range -pi to pi.
// Arguments:   - angle: angle in radians to be wrapped.
// Returns:     Wrapped angle in radians.
//------------------------------------------------------------------------------
double wrap_to_pi(double angle)
{
    angle = fmod(angle + M_PI, 2*M_PI);
    if (angle < 0.0)
        angle += 2*M_PI;
    return angle - M_PI;
}

//------------------------------------------------------------------------------
// Name:        wrap_to_2pi
// Description: Wraps an angle in radians to the range 0 to 2*pi.
// Arguments:   - angle: angle in radians to be wrapped.
// Returns:     Wrapped angle in radians.
//------------------------------------------------------------------------------
double wrap_to_2pi(double angle)
{
    angle = fmod(angle, 2*M_PI);
    if (angle < 0)
        angle += 2*M_PI;
    return angle;
}

//------------------------------------------------------------------------------
// Name:        wrap_to_180
// Description: Wraps an angle in degrees to the range -180 to 180.
// Arguments:   - angle: angle in degrees to be wrapped.
// Returns:     Wrapped angle in degrees.
//------------------------------------------------------------------------------
double wrap_to_180(double angle)
{
    angle = fmod(angle + 180.0, 360.0);
    if (angle < 0.0)
        angle += 360.0;
    return angle - 180.0;
}

//------------------------------------------------------------------------------
// Name:        wrap_to_360
// Description: Wraps an angle in degrees to the range 0 to 360.
// Arguments:   - angle: angle in degrees to be wrapped.
// Returns:     Wrapped angle in degrees.
//------------------------------------------------------------------------------
double wrap_to_360(double angle)
{
    angle = fmod(angle, 360.0);
    if (angle < 0.0)
        angle += 360.0;
    return angle;
}

//------------------------------------------------------------------------------
// Name:        round_decimals
// Description: Rounds a double to a given number of decimals.
// Arguments:   - value: Number to be rounded.
//              - decimals: Number of decimals to round to.
// Returns:     Rounded number.
//------------------------------------------------------------------------------
double round_decimals(double value, unsigned char decimals)
{

      double pow_10 = pow(10.0f, (double)decimals);
      double rounded = round(value * pow_10) / pow_10;

      // Handle -0... I don't really understand this one but I don't want
      // negative zeros
      if (rounded == -0)
        rounded = 0;

      return rounded;

}

//------------------------------------------------------------------------------
// Name:        clamp
// Description: Clamps a value between the minimum and maximum value.
// Arguments:   - input: value to be scaled
//              - min: minimum value of input's range
//              - max: maximum value of input's range
// Returns:     input value clamped to the range
//------------------------------------------------------------------------------
double clamp(double input, double min, double max)
{
    return std::max(min, std::min(input, max));
}

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
    double out_min, double out_max)
{

    // Calculate the linear fit between minimum and maximum input and output
    double m = (out_max - out_min) / (in_max - in_min);
    double b = out_min - m*in_min;
    return m * input + b;

}

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
                   const std::vector<double> poly_lons)
{

    // Number of vertices
    size_t N = poly_lats.size();


    // Loop through every pair of vertices
    bool inside = false;
    for (size_t i = 0, j = N - 1; i < N; j = i++)
    {

        if ( ((poly_lons[i] > lon) != (poly_lons[j] > lon)) &&
             (lat < (poly_lats[j]-poly_lats[i]) * (lon-poly_lons[i])
                / (poly_lons[j]-poly_lons[i]) + poly_lats[i]) )
        {
            inside = !inside;
        }

    }

    return inside;

}

}
