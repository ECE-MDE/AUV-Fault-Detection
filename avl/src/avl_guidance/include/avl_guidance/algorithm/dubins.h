//==============================================================================
// Autonomous Vehicle Library
//
// Description: Utility functions related to dubins path planning in a local NED
//              frame. Implements the algorithms explained at:
//                  https://gieseanw.files.wordpress.com/2012/10/dubins.pdf
//==============================================================================

#ifndef DUBINS_H
#define DUBINS_H

// Util functions
#include <avl_core/util/math.h>

namespace avl
{

//==============================================================================
//                             ENUM DEFINITIONS
//==============================================================================

enum DubinsPathType
{
    RSR = 0, // Right - Straight - Right
    RSL = 1, // Right - Straight - Left
    LSR = 2, // Left - Straight - Right
    LSL = 3, // Left - Straight - Left
    LRL = 4, // Left - Right - Left
    RLR = 5 // Right - Left - Right
};

// Enum listing possible orbit directions
enum Direction
{
    RIGHT,
    LEFT
};

//==============================================================================
//                             STRUCT DEFINITIONS
//==============================================================================

// Struct containing information about an NED position
typedef struct Position
{
    double north = 0;
    double east = 0;
} Position;

// Struct containing information about a dubins point with a position, yaw,
// and turn radius
typedef struct DubinsPoint
{
    Position position;
    double yaw;
    double radius;
} DubinsPoint;

// Struct containing information about an orbit with a position, radius, and
// direction
typedef struct Orbit : Position
{
    double radius;
    Direction direction;
} Orbit;

// Struct containing information about a line with a start and end position
typedef struct Line
{
    Position start;
    Position end;
    double get_length()
    {
        return sqrt(pow(end.east-start.east,2) +
                    pow(end.north-start.north,2));
    }
} Line;


// Struct containing information about a Dubins path
typedef struct DubinsPath
{
    DubinsPoint start_point;
    DubinsPoint end_point;
    DubinsPathType type;
    Orbit start_orbit;
    Orbit end_orbit;
    Position transition_pos1;
    Position transition_pos2;
    Line transition_line;
    Orbit transition_orbit;
    bool type_is_ccc()
    {
        return type == LRL || type == RLR;
    }
} DubinsPath;

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
double get_arc_angle(Orbit orbit, Position start, Position end);

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
double get_arc_length(Orbit orbit, Position start, Position end);

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
std::vector<Line> get_tangent_lines(Orbit orbit1, Orbit orbit2);

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
    Position& tangent_left_pos1, Position& tangent_left_pos2);

//------------------------------------------------------------------------------
// Name:        get_tangent_orbits
// Description: Gets the tangent points between two orbits.
// Arguments:   - orbit1: First orbit.
//              - orbit2: Second orbit.
//              - tangent_right: Reference to store tangent right orbit.
//              - tangent_left: Reference to store tangent left orbit.
//------------------------------------------------------------------------------
Position get_tangent_point(Orbit orbit1, Orbit orbit2);

//------------------------------------------------------------------------------
// Name:        get_point_orbits
// Description: Gets the left and right orbits corresponding to a dubins point.
// Arguments:   - point: Dubins point to generate left and right orbits from.
//              - orbit_right: Reference to store right orbit.
//              - orbit_left: Reference to store left orbit.
//------------------------------------------------------------------------------
void get_point_orbits(DubinsPoint point, Orbit& orbit_right, Orbit& orbit_left);

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
bool get_transition_line(Orbit orbit1, Orbit orbit2, Line& transition_line);

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
    Position& transition_pos1, Position& transition_pos2);

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
    DubinsPathType type, DubinsPath& path);

//------------------------------------------------------------------------------
// Name:        get_dubins_path
// Description: Gets the total length of the Dubins path in meters.
// Arguments:   - path: Dubins path to calculate length of.
// Returns:     The total length of the Dubins path in meters.
//------------------------------------------------------------------------------
double get_path_length(DubinsPath path);

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
    Position new_pos);

//------------------------------------------------------------------------------
// Name:        get_dubins_path
// Description: Gets the shortest Dubins path between the start and end points.
// Arguments:   - start_point: Starting Dubins point.
//              - end_point: Ending Dubins point.
// Returns:     The shortest Dubins path between the start and end points.
//------------------------------------------------------------------------------
DubinsPath get_dubins_path(DubinsPoint start_point, DubinsPoint end_point);

} // namespace avl

#endif // DUBINS_H
