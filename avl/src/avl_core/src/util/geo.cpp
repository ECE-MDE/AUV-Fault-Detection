//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to geographic calculations.
//==============================================================================

#include "util/geo.h"

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        dms_to_deg
// Description: Converts a lat or lon string in DMS format (DDMM.ssss) with
//              cardinal direction (N, E, S, W) to decimal degrees.
// Arguments:   - dms: DMS formatted string (DDMM.ssss)
//              - dir: cardinal direction (N, E, S, W)
// Returns:     Lat or lon in decimal degrees.
//------------------------------------------------------------------------------
double dms_to_deg(const std::string& dms, std::string& dir)
{

    double value = std::stod(dms);

    // Get the degrees part of the DMS
    double D = floor(value / 100.0);

    // Get the minutes and seconds part of the DMS
    double MS = value - (D * 100.0);

    // Add the minutes and seconds to the degrees
    double deg = D + (MS / 60.0);

    // Make the degrees negative if direction is West or South
    if(dir == "W" || dir == "S")
        deg = -deg;

    return deg;

}

//------------------------------------------------------------------------------
// Name:        dms_to_rad
// Description: Converts a lat or lon string in DMS format (DDMM.ssss) with
//              cardinal direction (N, E, S, W) to radians.
// Arguments:   - dms: DMS formatted string (DDMM.ssss)
//              - dir: cardinal direction (N, E, S, W)
// Returns:     Lat or lon in radians.
//------------------------------------------------------------------------------
double dms_to_rad(const std::string& dms, std::string& dir)
{
    return avl::deg_to_rad(dms_to_deg(dms, dir));
}

//------------------------------------------------------------------------------
// Name:        m_sq_to_rad_sq
// Description: Converts units of m^2 to rad^2 assuming a spherical earth
//              with a radius of 6371000 meters. Useful for covariance
//              calculations.
// Arguments:   - m_sq: Value in m^2 to be converted to rad^2.
// Returns:     Value in m^2 converted to rad^2.
//------------------------------------------------------------------------------
double m_sq_to_rad_sq(double m_sq)
{
    m_sq = sqrt(m_sq);          // m^2 -> m
    m_sq = m_sq / EARTH_RADIUS; // m -> rad
    return m_sq*m_sq;           // rad -> rad^2
}

//------------------------------------------------------------------------------
// Name:        rad_sq_to_m_sq
// Description: Converts units of rad^2 to m^2 assuming a spherical earth
//              with a radius of 6371000 meters. Useful for covariance
//              calculations.
// Arguments:   - rad_sq: Value in rad^2 to be converted to m^2.
// Returns:     Value in rad^2 converted to m^2.
//------------------------------------------------------------------------------
double rad_sq_to_m_sq(double rad_sq)
{
    rad_sq = sqrt(rad_sq);          // rad^2 -> rad
    rad_sq = rad_sq * EARTH_RADIUS; // rad -> m
    return rad_sq*rad_sq;           // m -> m^2
}

//------------------------------------------------------------------------------
// Name:        geo_to_ned
// Description: Converts geodetic coordinates to NED coordinates in meters from
//              a geodetic origin point.
// Arguments:   - lat0: Origin point latitude in radians.
//              - lon0: Origin point longitude in radians.
//              - alt0: Origin point altitude in meters.
//              - lat: Point latitude in radians.
//              - lon: Point longitude in radians.
//              - alt: Point altitude in meters.
//              - N: Resulting North position in meters from origin point.
//              - E: Resulting East position in meters from origin point.
//              - D: Resulting Down position in meters from origin point.
//------------------------------------------------------------------------------
void geo_to_ned(double lat0, double lon0, double alt0,
                double lat,  double lon,  double alt,
                double& N,   double& E,   double& D)
{
    N = (lat - lat0) * EARTH_RADIUS;
    E = (lon - lon0) * EARTH_RADIUS * cos(lat);
    D = (alt - alt0) * -1.0;
}

//------------------------------------------------------------------------------
// Name:        ned_to_geo
// Description: Converts NED coordinates in meters from a geodetic origin point
//              to geodetic coordinates.
// Arguments:   - lat0: Origin point latitude in radians.
//              - lon0: Origin point longitude in radians.
//              - alt0: Origin point altitude in meters.
//              - N: North position in meters from origin point.
//              - E: East position in meters from origin point.
//              - D: Down position in meters from origin point.
//              - lat: Resulting latitude in radians.
//              - lon: Resulting longitude in radians.
//              - alt: Resulting altitude in meters.
//------------------------------------------------------------------------------
void ned_to_geo(double lat0, double lon0, double alt0,
                double N,    double E,    double D,
                double& lat, double& lon, double& alt)
{
    lat = lat0 + (N / EARTH_RADIUS);
    lon = lon0 + (E / (EARTH_RADIUS * cos(lat)));
    alt = alt0 - D;
}

//------------------------------------------------------------------------------
// Name:        initial_bearing
// Description: Calculates the initial bearing between two points expressed in
//              latitude and longitude in radians.
// Arguments:   - lat_start: start point latitude in radians
//              - lon_start: start point longitude in radians
//              - lat_end: end point latitude in radians
//              - lon_end: end point longitude in radians
// Returns:     Initial bearing in radians between the start and end points.
//------------------------------------------------------------------------------
double initial_bearing(double lat_start, double lon_start,
                       double lat_end,   double lon_end)
{

    double delta_lon = lon_end - lon_start;
    double y = sin(delta_lon)*cos(lat_end);
    double x = cos(lat_start)*sin(lat_end) -
        sin(lat_start)*cos(lat_end)*cos(delta_lon);

    return atan2(y, x);

}

//------------------------------------------------------------------------------
// Name:        final_bearing
// Description: Calculates the final bearing between two points expressed in
//              latitude and longitude in radians.
// Arguments:   - lat_start: start point latitude in radians
//              - lon_start: start point longitude in radians
//              - lat_end: end point latitude in radians
//              - lon_end: end point longitude in radians
// Returns:     Final bearing in radians between the start and end points.
//------------------------------------------------------------------------------
double final_bearing(double lat_start, double lon_start,
                     double lat_end,   double lon_end)
{
    double theta = initial_bearing(lat_end, lon_end, lat_start, lon_start);
    return fmod(theta + M_PI, 2*M_PI);
}

//------------------------------------------------------------------------------
// Name:        angular_distance
// Description: Calculates the angular distance in radians between two
//              points expressed in latitude and longitude in radians.
// Arguments:   - lat_start: start point latitude in radians
//              - lon_start: start point longitude in radians
//              - lat_end: end point latitude in radians
//              - lon_end: end point longitude in radians
// Returns:     Angular distance in radians between the start and end points.
//------------------------------------------------------------------------------
double angular_distance(double lat_start, double lon_start,
                        double lat_end,   double lon_end)
{

    double delta_lat = lat_end - lat_start;
    double delta_lon = lon_end - lon_start;

    double a = sin(delta_lat/2) * sin(delta_lat/2) + cos(lat_start)*cos(lat_end)
        * sin(delta_lon/2) *sin(delta_lon/2);
    double c = 2*atan2(sqrt(a), sqrt(1-a));

    return c;

}

//------------------------------------------------------------------------------
// Name:        cross_track_error
// Description: Calculates the cross track error in meters of a point relative
//              to a line between a start and an end point. All points are
//              expressed as latitude and longitude in radians.
// Arguments:   - lat_start: latitude of start point of the line in radians
//              - lon_start: longitude of end point of the line in radians
//              - lat_start: latitude of start point of the line in radians
//              - lon_end: longitude of end point of the line in radians
//              - lat: latitude of point of interest in radians
//              - lon: longitude of point of interest in radians
// Returns:     Cross track error between the point and the line in meters.
//------------------------------------------------------------------------------
double cross_track_error(double lat_start, double lon_start,
                         double lat_end,   double lon_end,
                         double lat,       double lon)
{

    // Angular distance between the start point and the point of interest
    double delta_13 = angular_distance(lat_start, lon_start, lat, lon);

    // Initial bearing from the start point to the point of interest
    double theta_13 = initial_bearing(lat_start, lon_start, lat, lon);

    // Initial bearing between the start and end points
    double theta_12 = initial_bearing(lat_start, lon_start, lat_end, lon_end);

    return asin(sin(delta_13)*sin(theta_13-theta_12)) * EARTH_RADIUS;

}

//------------------------------------------------------------------------------
// Name:        distance
// Description: Calculates the distance in meters between two points expressed
//              in latitude and longitude in radians.
// Arguments:   - lat_start: start point latitude in radians
//              - lon_start: start point longitude in radians
//              - lat_end: end point latitude in radians
//              - lon_end: end point longitude in radians
// Returns:     Distance between the start and end points in meters.
//------------------------------------------------------------------------------
double distance(double lat_start, double lon_start,
                double lat_end,   double lon_end)
{
    return EARTH_RADIUS * angular_distance(lat_start, lon_start,
                                           lat_end,   lon_end);
}

//------------------------------------------------------------------------------
// Name:        in_radius
// Description: Checks to see if point 2 is within a circle of the given
//              radius of point 1, with both points expressed in latitude
//              and longitude in radians.
// Arguments:   - lat1: point 1 latitude in radians
//              - lon1: point 1 longitude in radians
//              - radius: radius around point 1 to check
//              - lat2: point 2 latitude in radians
//              - lon2: point 2 longitude in radians
// Returns:     True if point 2 is within the given radius of point 1. False
//              otherwise.
//------------------------------------------------------------------------------
bool in_radius(double lat1, double lon1, double radius,
               double lat2, double lon2)
{
    return distance(lat1, lon1, lat2, lon2) <= radius;
}

//------------------------------------------------------------------------------
// Name:        crossed_half_plane
// Description: Checks to see if a point is has crossed a half plane defined by
//              the final heading of a line defined between a start and an end
//              point.
// Arguments:   - lat_start: latitude of start point of the line in radians
//              - lon_start: longitude of end point of the line in radians
//              - lat_start: latitude of start point of the line in radians
//              - lon_end: longitude of end point of the line in radians
//              - lat: latitude of point of interest in radians
//              - lon: longitude of point of interest in radians
// Returns:     True if the point has crossed the half plane, false otherwise.
//------------------------------------------------------------------------------
bool crossed_half_plane(double lat_start, double lon_start,
                        double lat_end,   double lon_end,
                        double lat,       double lon)
{

    // Calculate the final bearings for the lines from start to end and from
    // the point to the end, and find the difference
    double line_bearing = final_bearing(lat_start, lon_start,
                                        lat_end,   lon_end);
    double point_bearing = final_bearing(lat,     lon,
                                         lat_end, lon_end);
    double delta_bearing = wrap_to_pi(line_bearing - point_bearing);

    // If the bearing difference is greater than 90 degrees, we are past the
    // half plane because the line from the point to the end is pointed
    // backwards
    return abs(delta_bearing) >= (M_PI/2.0);

}

}
