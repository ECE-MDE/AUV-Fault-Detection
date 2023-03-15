//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to geographic calculations.
//==============================================================================

#ifndef GEO_H
#define GEO_H

// Util functions
#include "math.h"

// C++ includes
#include <string>

namespace avl
{

// Median radius of the earth in meters
#define EARTH_RADIUS 6371000.0

//==============================================================================
//                             FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        dms_to_deg
// Description: Converts a lat or lon string in DMS format (DDMM.ssss) with
//              cardinal direction (N, E, S, W) to decimal degrees.
// Arguments:   - dms: DMS formatted string (DDMM.ssss)
//              - dir: cardinal direction (N, E, S, W)
// Returns:     Lat or lon in decimal degrees.
//------------------------------------------------------------------------------
double dms_to_deg(const std::string& dms, std::string& dir);

//------------------------------------------------------------------------------
// Name:        dms_to_rad
// Description: Converts a lat or lon string in DMS format (DDMM.ssss) with
//              cardinal direction (N, E, S, W) to radians.
// Arguments:   - dms: DMS formatted string (DDMM.ssss)
//              - dir: cardinal direction (N, E, S, W)
// Returns:     Lat or lon in radians.
//------------------------------------------------------------------------------
double dms_to_rad(const std::string& dms, std::string& dir);

//------------------------------------------------------------------------------
// Name:        m_sq_to_rad_sq
// Description: Converts units of m^2 to rad^2 assuming a spherical earth
//              with a radius of 6371000 meters. Useful for covariance
//              calculations.
// Arguments:   - m_sq: Value in m^2 to be converted to rad^2.
// Returns:     Value in m^2 converted to rad^2.
//------------------------------------------------------------------------------
double m_sq_to_rad_sq(double m_sq);

//------------------------------------------------------------------------------
// Name:        rad_sq_to_m_sq
// Description: Converts units of rad^2 to m^2 assuming a spherical earth
//              with a radius of 6371000 meters. Useful for covariance
//              calculations.
// Arguments:   - rad_sq: Value in rad^2 to be converted to m^2.
// Returns:     Value in rad^2 converted to m^2.
//------------------------------------------------------------------------------
double rad_sq_to_m_sq(double rad_sq);

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
                double& N,   double& E,   double& D);

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
                double& lat, double& lon, double& alt);

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
                       double lat_end,   double lon_end);

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
                     double lat_end,   double lon_end);

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
                        double lat_end,   double lon_end);

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
                         double lat,       double lon);

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
                double lat_end,   double lon_end);

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
               double lat2, double lon2);

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
                        double lat,       double lon);

} // namespace avl

#endif // GEO_H
