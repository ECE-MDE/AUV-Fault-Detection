//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to dubins path planning in a local NED
//              frame.
//        See https://gieseanw.files.wordpress.com/2012/10/dubins.pdf
//==============================================================================

#include <avl_guidance/algorithm/dubins.h>

// Util functions
#include <avl_core/util/math.h>
#include <avl_core/util/matrix.h>

namespace avl
{

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        get_arc_angle
// Description: Calculates the angle in radians formed by drawing lines from
//              the center of a given orbit to the start and positions.
//              Calculated using the orbit's direction.
// Arguments:   - orbit: Orbit to calculate angle relative to.
//              - start: Starting position.
//              - end: Ending position.
// Returns:     Angle in radians formed by drawing lines from the center of a
//              given orbit to the start and positions.
//------------------------------------------------------------------------------
double get_arc_angle(Orbit orbit, Position start, Position end)
{

    // Calculate the angle centered at the orbit from east to the start position
    double theta_start = avl::wrap_to_2pi(atan2(start.north - orbit.north,
                                                start.east - orbit.east));

    // Calculate the angle centered at the orbit from east to the end position
    double theta_end = avl::wrap_to_2pi(atan2(end.north - orbit.north,
                                              end.east - orbit.east));

    return avl::arc_angle(theta_start, theta_end, !orbit.direction);

}

//------------------------------------------------------------------------------
// Name:        get_arc_length
// Description: Calculates the arc length in meters formed by drawing lines from
//              the center of a given orbit to the start and end positions and
//              following the orbit between them. Calculated using the orbit's
//              direction.
// Arguments:   - orbit: Orbit to calculate arc distance relative to.
//              - start: Starting position.
//              - end: Ending position.
// Returns:     Arc length in meters formed by drawing lines from the center of
//              a given orbit to the start and positions and following the orbit
//              between them.
//------------------------------------------------------------------------------
double get_arc_length(Orbit orbit, Position start, Position end)
{
    return orbit.radius * get_arc_angle(orbit, start, end);
}

//------------------------------------------------------------------------------
// Name:        get_tangent_lines
// Description: Gets the tangent lines between two orbits of the same radius.
//              If the orbits are separated, the two inner and two outer tangent
//              lines will be returned. If the orbits are touching or
//              overlapping, the inner tangents will not be returned.
// Arguments:   - orbit: Orbit to calculate arc distance relative to.
//              - start: Starting position.
//              - end: Ending position.
// Returns:     Arc length in meters formed by drawing lines from the center of
//              a given orbit to the start and positions and following the orbit
//              between them.
//------------------------------------------------------------------------------
std::vector<Line> get_tangent_lines(Orbit orbit1, Orbit orbit2)
{

    // This algorithm only works if the orbits have the same radius
    if (orbit1.radius != orbit2.radius)
        throw std::runtime_error("get_tangent_lines: orbits must have the same "
            "radius");

    std::vector<Line> lines;
    double n_s = orbit1.north;
    double e_s = orbit1.east;
    double n_e = orbit2.north;
    double e_e = orbit2.east;
    double radius = orbit1.radius;

    //==========================================================================
    // Black box from Ben Biggs to calculate tangent points, to be cleaned up

    // Solve for the distance between the circle centers
    double centers_distance = sqrt(pow(e_e-e_s,2) + pow(n_e-n_s,2));

    // If the orbit centers are too close, do not try to calculate tangents,
    // since concentric circles have infinite tangent lines
    if (centers_distance <= 1.0e-9) return lines;

    // Vector to contain the tangent points
    std::vector<double> tangent_points;

    // Vector from start center to end center
    Vector2d v1;
    v1 << n_e - n_s , e_e - e_s;

    // Normalize the vector
    Vector2d v1_n = v1.normalized();

    // Find each pair of tangent points
    for (double sign1 : {1.0, -1.0})
    {

        double c = (radius - sign1 * radius) / centers_distance;

        // Want to be subtracting small from large, not adding
        if (c*c > 1.0) continue;
        double h = sqrt(std::max(0.0, 1.0 - c*c));

        for (double sign2 : {1.0, -1.0})
        {

            double nx = v1_n(1) * c - sign2 * h * v1_n(0);
            double ny = v1_n(0) * c + sign2 * h * v1_n(1);

            // Create a line from the tangent points
            Line line;
            line.start.north = n_s + radius * ny;
            line.start.east = e_s + radius * nx;
            line.end.north = n_e + sign1 * radius * ny;
            line.end.east = e_e + sign1 * radius * nx;
            if (line.get_length() > 1.0e-9)
                lines.push_back(line);

        }

    }

    //==========================================================================

    return lines;

}

//------------------------------------------------------------------------------
// Name:        get_tangent_orbits
// Description: Gets the left and right orbits tangent to two orbits of the same
//              radius. The input orbits must be closer than 4 radii such that
//              two tangent circles can be created.
// Arguments:   - orbit1: First orbit.
//              - orbit2: Second orbit.
//              - tangent_right: Reference to store tangent right orbit.
//              - tangent_left: Reference to store tangent left orbit.
//------------------------------------------------------------------------------
void get_tangent_orbits(Orbit orbit1, Orbit orbit2,
    Orbit& tangent_right, Orbit& tangent_left,
    Position& tangent_right_pos1, Position& tangent_right_pos2,
    Position& tangent_left_pos1, Position& tangent_left_pos2)
{

    // This algorithm only works if the orbits have the same radius
    if (orbit1.radius != orbit2.radius)
        throw std::runtime_error("get_tangent_orbits: orbits must have the same"
            " radius");

    // Calculate the distance between the orbit centers
    double distance = sqrt(pow(orbit2.east - orbit1.east, 2) +
                           pow(orbit2.north - orbit1.north, 2));

    // If the orbit centers are too close, do not try to calculate tangents,
    // since concentric circles have infinite tangent lines
    if (distance <= 1.0e-9)
       throw std::runtime_error("get_tangent_orbits: concentric orbits cannot "
           "have tangent orbits");

    // If the orbit centers are too close, do not try to calculate tangents,
    // since concentric circles have infinite tangent lines
    if (distance > 4.0*orbit1.radius)
        throw std::runtime_error("get_tangent_orbits: orbits are too far apart "
            "to have tangent orbits");

    // Variables to simplify notation
    double D = distance;
    double r = orbit1.radius;

    // Use the law of cosines to calculate the angle that the line between the
    // two orbits must rotate to point at the tangent orbit's center
    double theta_rel = acos(D/(4.0*r));

    // Calculate the angle between the east axis and the line between the
    // two orbits
    double theta_v1 = atan2(orbit2.north - orbit1.north,
                            orbit2.east - orbit1.north);

    // Calculate the angles between the east axis and the left and right orbit
    // centers
    double theta_left =  theta_v1 + theta_rel;
    double theta_right = theta_v1 - theta_rel;

    // Construct the right tangent orbit
    tangent_right.north =  orbit1.north + 2.0*r*sin(theta_right);
    tangent_right.east =   orbit1.east + 2.0*r*cos(theta_right);
    tangent_right.radius = r;
    tangent_right.direction = static_cast<Direction>(!orbit1.direction);

    // Construct the left tangent orbit
    tangent_left.north =  orbit1.north + 2.0*r*sin(theta_left);
    tangent_left.east =   orbit1.east + 2.0*r*cos(theta_left);
    tangent_left.radius = r;
    tangent_left.direction = static_cast<Direction>(!orbit1.direction);

    // Get the tangent points
    tangent_right_pos1 = get_tangent_point(orbit1, tangent_right);
    tangent_right_pos2 = get_tangent_point(tangent_right, orbit2);
    tangent_left_pos1 = get_tangent_point(orbit1, tangent_left);
    tangent_left_pos2 = get_tangent_point(tangent_left, orbit2);

}

//------------------------------------------------------------------------------
// Name:        get_tangent_orbits
// Description: Gets the tangent points between two orbits.
// Arguments:   - orbit1: First orbit.
//              - orbit2: Second orbit.
//              - tangent_right: Reference to store tangent right orbit.
//              - tangent_left: Reference to store tangent left orbit.
//------------------------------------------------------------------------------
Position get_tangent_point(Orbit orbit1, Orbit orbit2)
{

    // This algorithm only works if the orbits have the same radius
    if (orbit1.radius != orbit2.radius)
        throw std::runtime_error("get_tangent_point: orbits must have the same "
            "radius");

    // Calculate the distance between the orbit centers
    double distance = sqrt(pow(orbit2.east - orbit1.east,2) +
                           pow(orbit2.north - orbit1.north,2));

    // The orbits must be touching to be tangent
    if (abs(distance - 2.0*orbit1.radius) > 1.0e-9)
       throw std::runtime_error("get_tangent_point: orbits must be touching to "
           "be tangent");

    // Calculate the coordinates of the tangent point that is exactly half way
    // between the center of each orbit
    Position tangent_point;
    tangent_point.north = (orbit1.north + orbit2.north) / 2.0;
    tangent_point.east =  (orbit1.east + orbit2.east) / 2.0;

    return tangent_point;

}

//------------------------------------------------------------------------------
// Name:        get_point_orbits
// Description: Gets the left and right orbits corresponding to a dubins point.
// Arguments:   - point: Dubins point to generate left and right orbits from.
//              - orbit_right: Reference to store right orbit.
//              - orbit_left: Reference to store left orbit.
//------------------------------------------------------------------------------
void get_point_orbits(DubinsPoint point, Orbit& orbit_right, Orbit& orbit_left)
{

    double theta = -point.yaw + M_PI/2.0;

    // Solve for the left turning circle ned position wrt input coordinates
    double n_cl = point.position.north + point.radius*sin(theta + M_PI/2.0);
    double e_cl = point.position.east + point.radius*cos(theta + M_PI/2.0);
    orbit_left.north = n_cl;
    orbit_left.east = e_cl;
    orbit_left.radius = point.radius;
    orbit_left.direction = LEFT;

    // Solve for the right turning circle ned position wrt input coordinates
    double n_cr = point.position.north + point.radius*sin(theta - M_PI/2.0);
    double e_cr = point.position.east + point.radius*cos(theta - M_PI/2.0);
    orbit_right.north = n_cr;
    orbit_right.east = e_cr;
    orbit_right.radius = point.radius;
    orbit_right.direction = RIGHT;

}

//------------------------------------------------------------------------------
// Name:        get_transition_line
// Description: Gets the directional transition line that can be followed to
//              transition from the first orbit to the second if a valid one
//              exists.
// Arguments:   - orbit1: First orbit.
//              - orbit2: Second orbit.
//              - transition_line: Reference to store transition line.
// Returns:     True if a valid transition line exists, false otherwise.
//------------------------------------------------------------------------------
bool get_transition_line(Orbit orbit1, Orbit orbit2, Line& transition_line)
{

    try
    {

        std::vector<Line> lines = get_tangent_lines(orbit1, orbit2);

        if (orbit1.direction == RIGHT && orbit2.direction == RIGHT)
            transition_line = lines.at(0);
        else if (orbit1.direction == LEFT && orbit2.direction == LEFT)
            transition_line = lines.at(1);
        else if (orbit1.direction == RIGHT && orbit2.direction == LEFT)
            transition_line = lines.at(2);
        else
            transition_line = lines.at(3);
        return true;
    }
    catch (const std::out_of_range& oor)
    {
        return false;
    }

    return false;

}

//------------------------------------------------------------------------------
// Name:        get_transition_orbit
// Description: Gets the directional transition orbit that can be followed to
//              transition from the first orbit to the second if a valid one
//              exists.
// Arguments:   - orbit1: First orbit.
//              - orbit2: Second orbit.
//              - transition_orbit: Reference to store transition orbit.
// Returns:     True if a valid transition orbit exists, false otherwise.
//------------------------------------------------------------------------------
bool get_transition_orbit(Orbit orbit1, Orbit orbit2, Orbit& transition_orbit,
    Position& transition_pos1, Position& transition_pos2)
{

    try
    {

        Orbit tangent_right;
        Position tangent_right_pos1;
        Position tangent_right_pos2;
        Orbit tangent_left;
        Position tangent_left_pos1;
        Position tangent_left_pos2;

        get_tangent_orbits(orbit1, orbit2, tangent_right, tangent_left,
            tangent_right_pos1, tangent_right_pos2,
            tangent_left_pos1, tangent_left_pos2);

        if (orbit1.direction == RIGHT && orbit2.direction == RIGHT)
        {
            transition_orbit = tangent_right;
            transition_pos1 = tangent_right_pos1;
            transition_pos2 = tangent_right_pos2;
        }
        else if (orbit1.direction == LEFT && orbit2.direction == LEFT)
        {
            transition_orbit = tangent_left;
            transition_pos1 = tangent_left_pos1;
            transition_pos2 = tangent_left_pos2;
        }
        else
            return false;

        return true;

    }
    catch (const std::exception& ex)
    {
        return false;
    }

    return false;

}

//------------------------------------------------------------------------------
// Name:        get_dubins_path
// Description: Gets the Dubins path of the given type between the start and end
//              points, if it exists.
// Arguments:   - start_point: Starting Dubins point.
//              - end_point: Ending Dubins point.
//              - type: Dubins path type.
//              - path: Reference to store resulting Dubins path.
// Returns:     True if a valid path of the given type exists, false otherwise.
//------------------------------------------------------------------------------
bool get_dubins_path(DubinsPoint start_point, DubinsPoint end_point,
    DubinsPathType type, DubinsPath& path)
{

    // Generate the start point left and right orbits
    Orbit start_orbit_left;
    Orbit start_orbit_right;
    get_point_orbits(start_point, start_orbit_right, start_orbit_left);

    // Generate the end point left and right orbits
    Orbit end_orbit_left;
    Orbit end_orbit_right;
    get_point_orbits(end_point, end_orbit_right, end_orbit_left);

    bool path_valid;
    DubinsPath p;
    p.start_point = start_point;
    p.end_point = end_point;
    p.type = type;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Handle CLC paths
    if (type == RSR || type == RSL || type == LSR || type == LSL)
    {

        switch (type)
        {

            case RSR:
            {
                p.start_orbit = start_orbit_right;
                p.end_orbit = end_orbit_right;
                break;
            }

            case RSL:
            {
                p.start_orbit = start_orbit_right;
                p.end_orbit = end_orbit_left;
                break;
            }

            case LSR:
            {
                p.start_orbit = start_orbit_left;
                p.end_orbit = end_orbit_right;
                break;
            }

            case LSL:
            {
                p.start_orbit = start_orbit_left;
                p.end_orbit = end_orbit_left;
                break;
            }

            default:
            {
                throw std::runtime_error("get_dubins_path: unhandled CLC path "
                    "type, this should never happen");
            }

        }

        // There is one edge case to handle: the case where the the start and
        // end point are exactly one turn radius apart and pointing 180 degrees
        // from eachother. In this case, the right or left orbits are concentric
        // and there are no transition points; you can just follow the orbit
        // directly from the start to the end. This case can be detected when
        // the start and end orbits are concentric and the start and end points
        // are not in the same place.

        // The case where the start and end orbits were concentric and the start
        // and end points ARE in the same place is already handles correctly by
        // a CCC path

        double point_distance = sqrt(pow(p.start_point.position.east -
                                         p.end_point.position.east, 2) +
                                     pow(p.start_point.position.north -
                                         p.end_point.position.north, 2));
        double orbit_distance = sqrt(pow(p.end_orbit.east -
                                         p.start_orbit.east, 2) +
                                     pow(p.end_orbit.north -
                                         p.start_orbit.north, 2));
        if (orbit_distance < 1.0e-9 && point_distance > 1.0e-9)
        {
            path_valid = true;
            p.transition_line.start = start_point.position;
            p.transition_line.end = start_point.position;
            p.transition_pos1 = start_point.position;
            p.transition_pos2 = start_point.position;
        }
        else
        {
            path_valid = get_transition_line(p.start_orbit, p.end_orbit,
                p.transition_line);
            p.transition_pos1 = p.transition_line.start;
            p.transition_pos2 = p.transition_line.end;
        }

    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Handle CCC paths
    else
    {

        switch (type)
        {

            case LRL:
            {
                p.start_orbit = start_orbit_left;
                p.end_orbit = end_orbit_left;
                break;
            }

            case RLR:
            {
                p.start_orbit = start_orbit_right;
                p.end_orbit = end_orbit_right;
                break;
            }

            default:
            {
                throw std::runtime_error("get_dubins_path: unhandled CCC path "
                    "type, this should never happen");
            }

        }

        path_valid = get_transition_orbit(
            p.start_orbit,
            p.end_orbit,
            p.transition_orbit,
            p.transition_pos1,
            p.transition_pos2);

    }

    if (path_valid)
        path = p;
    return path_valid;

}

//------------------------------------------------------------------------------
// Name:        get_dubins_path
// Description: Gets the total length of the Dubins path in meters.
// Arguments:   - path: Dubins path to calculate length of.
// Returns:     The total length of the Dubins path in meters.
//------------------------------------------------------------------------------
double get_path_length(DubinsPath path)
{

    double start_arc_len;
    double transition_len;
    double end_arc_len;
    start_arc_len = get_arc_length(path.start_orbit,
        path.start_point.position, path.transition_pos1);
    end_arc_len = get_arc_length(path.end_orbit,
        path.transition_pos2, path.end_point.position);

    if (path.type_is_ccc())
        transition_len = get_arc_length(path.transition_orbit,
            path.transition_pos1, path.transition_pos2);
    else
        transition_len = path.transition_line.get_length();

    return start_arc_len + transition_len + end_arc_len;

}

//------------------------------------------------------------------------------
// Name:        crossed_orbit_half_plane
// Description: Determines if a position has passed the orbit half plane defined
//              by the line between the orbit center and the goal position based
//              on the given previous position and new position.
// Arguments:   - orbit: Orbit being followed.
//              - goal_pos: Goal position that the half plane is drawn
//                through.
//              - prev_pos: Previous position of object to be checked for  half
//                plane crossing.
//              - new_pos: New position of object to be checked for half plane
//                crossing.
// Returns:     True if the orbit half plane was crossed, false otherwise.
//------------------------------------------------------------------------------
bool crossed_orbit_half_plane(Orbit orbit, Position goal_pos, Position prev_pos,
    Position new_pos)
{

    // When the position crosses the orbit half plane, the arc angle will jump
    // by 2PI because it will now have to go all the way around again. We can
    // check for a jump of 1.5PI to account for slower position update rates
    double arc_angle = get_arc_angle(orbit, new_pos, goal_pos);
    double prev_arc_angle = get_arc_angle(orbit, prev_pos, goal_pos);
    return abs(arc_angle - prev_arc_angle) > 1.5*M_PI;

}

//------------------------------------------------------------------------------
// Name:        get_dubins_path
// Description: Gets the shortest Dubins path between the start and end points.
// Arguments:   - start_point: Starting Dubins point.
//              - end_point: Ending Dubins point.
// Returns:     The shortest Dubins path between the start and end points.
//------------------------------------------------------------------------------
DubinsPath get_dubins_path(DubinsPoint start_point, DubinsPoint end_point)
{


    // Generate the start point left and right orbits
    Orbit start_orbit_left;
    Orbit start_orbit_right;
    get_point_orbits(start_point, start_orbit_left, start_orbit_right);

    // Generate the end point left and right orbits
    Orbit end_orbit_left;
    Orbit end_orbit_right;
    get_point_orbits(end_point, end_orbit_left, end_orbit_right);

    std::vector<DubinsPath> paths;
    std::vector<double> lengths;
    for (size_t i = 0; i < 6; i++)
    {
        DubinsPath path;
        DubinsPathType path_type = static_cast<DubinsPathType>(i);
        if(get_dubins_path(start_point, end_point, path_type, path))
            lengths.push_back(get_path_length(path));
        else
            lengths.push_back(NAN);
        paths.push_back(path);
    }

    auto min_it = std::min_element(lengths.begin(), lengths.end());
    size_t i_min = std::distance(lengths.begin(), min_it);

    return paths.at(i_min);

}

} // namespace avl
