//==============================================================================
// Autonomous Vehicle Library
//
// Description: This guidance node attempts to guide the vehicle to a waypoint
//              defined by a position and an optional entry yaw. If no entry
//              yaw is specified, the vehicle will follow a straight line
//              between its starting position and the waypoint position. If an
//              entry yaw is specified, the vehicle will follow a Dubins
//              path from its starting position to the waypoint position such
//              that it reaches the waypoint position at the specified entry
//              yaw.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/line (avl_msgs/LineSetpointMsg)
//              setpoint/orbit (avl_msgs/OrbitSetpointMsg)
//              setpoint/ground_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/water_speed (avl_msgs/Float64)
//              setpoint/rpm (avl_msgs/Float64)
//              setpoint/depth (avl_msgs/Float64)
//              setpoint/height (avl_msgs/Float64)
//
// Subscribers: nav/inertial_nav (avl_msgs/NavigationMsg)
//==============================================================================

// Guidance node base class
#include <avl_guidance/guidance_node.h>

// Dubins path util functions
#include <avl_guidance/algorithm/dubins.h>
using namespace avl;

// Geographic util functions
#include <avl_core/util/geo.h>

// Monitored subscriber class
#include <avl_core/monitored_subscriber.h>

// ROS message includes
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/LineSetpointMsg.h>
#include <avl_msgs/OrbitSetpointMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
using namespace avl_msgs;

//==============================================================================
//                                 ENUMS
//==============================================================================

// Enum listing methods to reach the waypoint
enum WaypointType
{
    WAYPOINT_STRAIGHT,
    WAYPOINT_DUBINS
};

// Enum listing possible states for Dubins path following
enum DubinsState
{
    DUBINS_START_ORBIT,
    DUBINS_TRANSITION_ORBIT,
    DUBINS_TRANSITION_LINE,
    DUBINS_END_ORBIT
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class WaypointGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    WaypointGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/WAYPOINT", argc, argv)
    {

    }

private:

    // Subscriber for navigation data
    MonitoredSubscriber<NavigationMsg> nav_sub;

    // Vehicle navigation info variables
    bool nav_valid = false;
    double vehicle_lat_prev;
    double vehicle_lon_prev;
    double vehicle_lat;
    double vehicle_lon;
    double vehicle_yaw;

    // Vehicle NED position variables
    double origin_lat;
    double origin_lon;
    Position vehicle_pos;
    Position vehicle_pos_prev;

    // Publishers for controller setpoints
    ros::Publisher line_pub;
    ros::Publisher orbit_pub;
    std::map<std::string, ros::Publisher> pub_map;

    // Action to be executed
    avl::Action action;
    double waypoint_lat;
    double waypoint_lon;
    double entry_yaw;

    // Type of waypoint being executed depending on whether yaw is specified
    WaypointType waypoint_type;

    // Dubins path and current Dubins state for the path generated
    DubinsState state;
    DubinsPath dubins_path;

    // Vehicle turn radius from config file
    double turn_radius;

private:

    //--------------------------------------------------------------------------
    // Name:        start_new_action
    // Description: Called when a new action is received.
    //--------------------------------------------------------------------------
    bool start_new_action(Action action)
    {

        // If the vehicle position is NaN, do not accept the action
        if (!nav_valid)
        {
            log_error("action rejected, navigation is invalid");
            return false;
        }

        // Save the action and get the waypoint parameters
        this->action = action;
        waypoint_lat = avl::deg_to_rad(action.parameters.get("LAT").to_double());
        waypoint_lon = avl::deg_to_rad(action.parameters.get("LON").to_double());

        if (action.parameters.has("ENTRY YAW"))
            entry_yaw = avl::deg_to_rad(action.parameters.get("ENTRY YAW").to_double());
        else
            entry_yaw = NAN;

        // Save the current vehicle position as the origin for conversion from
        // geodetic coordinates to NED coordinates
        origin_lat = vehicle_lat;
        origin_lon = vehicle_lon;

        // If the goal has an entry yaw, we will plan a Dubins path to the
        // waypoint. Otherwise, we will plan a straight line to the waypoint
        if (std::isnan(entry_yaw))
        {
            log_info("waypoint type is straight line");
            waypoint_type = WAYPOINT_STRAIGHT;
        }
        else
        {

            log_info("waypoint type is dubins path");

            waypoint_type = WAYPOINT_DUBINS;
            state = DUBINS_START_ORBIT;

            // Convert the waypoint location to NED coordinates
            double end_north;
            double end_east;
            double end_down;
            geo_to_ned(origin_lat,   origin_lon,   0.0,
                       waypoint_lat, waypoint_lon, 0.0,
                       end_north,    end_east,     end_down);

            // Create the start point. Since the origin lat/lon is set to the
            // vehicle's current position, the vehicle always starts at the
            // NED origin
            DubinsPoint start_point;
            start_point.position.north = 0.0;
            start_point.position.east = 0.0;
            start_point.yaw = vehicle_yaw;
            start_point.radius = turn_radius;

            // Create the end point
            DubinsPoint end_point;
            end_point.position.north = end_north;
            end_point.position.east = end_east;
            end_point.yaw = entry_yaw;
            end_point.radius = turn_radius;

            // Calculate the Dubins path between the start and end points
            dubins_path = get_dubins_path(start_point, end_point);

            log_data("[path] %f %f %f %f %f %f %f %f %d %f %f %f %d %f %f %f "
                "%d %f %f %f %f %f %f %f %f %f %f %f %d",
                dubins_path.start_point.position.north,
                dubins_path.start_point.position.east,
                dubins_path.start_point.yaw,
                dubins_path.start_point.radius,

                dubins_path.end_point.position.north,
                dubins_path.end_point.position.east,
                dubins_path.end_point.yaw,
                dubins_path.end_point.radius,

                dubins_path.type,

                dubins_path.start_orbit.north,
                dubins_path.start_orbit.east,
                dubins_path.start_orbit.radius,
                dubins_path.start_orbit.direction,

                dubins_path.end_orbit.north,
                dubins_path.end_orbit.east,
                dubins_path.end_orbit.radius,
                dubins_path.end_orbit.direction,

                dubins_path.transition_pos1.north,
                dubins_path.transition_pos1.east,

                dubins_path.transition_pos2.north,
                dubins_path.transition_pos2.east,

                dubins_path.transition_line.start.north,
                dubins_path.transition_line.start.east,
                dubins_path.transition_line.end.north,
                dubins_path.transition_line.end.east,

                dubins_path.transition_orbit.north,
                dubins_path.transition_orbit.east,
                dubins_path.transition_orbit.radius,
                dubins_path.transition_orbit.direction);

        }

        // Mark the action as accepted
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        publish_orbit_msg
    // Description: Publishes an orbit setpoint message corresponding to the
    //              input orbit.
    // Arguments:   - orbit: Orbit struct to be published.
    //--------------------------------------------------------------------------
    void publish_orbit_msg(Orbit orbit)
    {

        // Convert the orbit center point to geodetic coordinates
        double orbit_lat;
        double orbit_lon;
        double orbit_alt;
        ned_to_geo(origin_lat,  origin_lon, 0.0,
                   orbit.north, orbit.east, 0.0,
                   orbit_lat,   orbit_lon,  orbit_alt);

        // Publish the orbit setpoint message
        OrbitSetpointMsg msg;
        msg.enable = true;
        msg.lat = orbit_lat;
        msg.lon = orbit_lon;
        msg.radius = turn_radius;
        msg.clockwise = (orbit.direction == RIGHT) ? true : false;
        orbit_pub.publish(msg);

    }

    //--------------------------------------------------------------------------
    // Name:        publish_orbit_msg
    // Description: Publishes a line setpoint message corresponding to the
    //              input line.
    // Arguments:   - line: Line struct to be published.
    //--------------------------------------------------------------------------
    void publish_line_msg(Line line)
    {

        // Convert the line start point to geodetic coordinates
        double line_start_lat;
        double line_start_lon;
        double line_start_alt;
        ned_to_geo(origin_lat,       origin_lon,      0.0,
                   line.start.north, line.start.east, 0.0,
                   line_start_lat,   line_start_lon,  line_start_alt);

        // Convert the line end point to geodetic coordinates
        double line_end_lat;
        double line_end_lon;
        double line_end_alt;
        ned_to_geo(origin_lat,     origin_lon,    0.0,
                   line.end.north, line.end.east, 0.0,
                   line_end_lat,   line_end_lon,  line_end_alt);

        // Publish the line setpoint message
        LineSetpointMsg msg;
        msg.enable = true;
        msg.lat_start = line_start_lat;
        msg.lon_start = line_start_lon;
        msg.lat_end = line_end_lat;
        msg.lon_end = line_end_lon;
        line_pub.publish(msg);

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_action
    // Description: Called at the iteration rate while an action is
    //              executing. Main guidance node logic, such as publishing
    //              controller setpoints, should be executed here.
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {

        Feedback feedback;

        // Calculate the vehicle's current position in NED coordinates
        vehicle_pos_prev = vehicle_pos;
        double down;
        geo_to_ned(origin_lat,        origin_lon,       0.0,
                   vehicle_lat,       vehicle_lon,      0.0,
                   vehicle_pos.north, vehicle_pos.east, down);

        // Handle iteration different based on whether the waypoint is a
        // straight line waypoint or a Dubins waypoint
        switch (waypoint_type)
        {

            // For a straight line waypoint, we must check if the vehicle has
            // reached the waypoint. If it as reached the waypoint, we can
            // finish the task successfully. If it has not, publish a line
            // setpoint message to continue moving toward the waypoint. Reaching
            // the waypoint is defined as entering the capture radius or
            // crossing the half plane
            case WAYPOINT_STRAIGHT:
            {

                // Check whether the vehicle has entered the capture radius
                bool capture_radius_entered = in_radius(waypoint_lat,
                    waypoint_lon, turn_radius, vehicle_lat, vehicle_lon);

                // Check whether the vehicle has crossed the half plane
                bool half_plane_crossed = crossed_half_plane(
                    origin_lat,   origin_lon,
                    waypoint_lat, waypoint_lon,
                    vehicle_lat,  vehicle_lon);

                // Check if the waypoint has been achieved. If it has, finish
                // the action. If not, publish a line setpoint message
                if (capture_radius_entered || half_plane_crossed)
                {

                    if (capture_radius_entered)
                        log_info("task succeeded, entered capture radius");
                    else
                        log_info("task succeeded, crossed half plane");

                    // Finish the action successfully
                    Result result;
                    result.success = true;
                    finish_action(true, result);
                    feedback.percent = 100.0;
                    return feedback;

                }
                else
                {

                    // Publish a line setpoint message
                    LineSetpointMsg msg;
                    msg.enable = true;
                    msg.lat_start = origin_lat;
                    msg.lon_start = origin_lon;
                    msg.lat_end = waypoint_lat;
                    msg.lon_end = waypoint_lon;
                    line_pub.publish(msg);

                }

                break;

            }

            // For a Dubins waypoint, our action depends on which segment of the
            // Dubins path we are in.
            case WAYPOINT_DUBINS:
            {

                // Check which segment of the Dubins path we are on and publish
                // the correct line or orbit setpoint message

                switch (state)
                {

                    case DUBINS_START_ORBIT:
                    {

                        // Check if we've finished the start orbit by checking
                        // if the vehicle crossed the half plane between the
                        // orbtit center and the first transition point
                        bool finished = crossed_orbit_half_plane(
                            dubins_path.start_orbit,
                            dubins_path.transition_pos1,
                            vehicle_pos_prev,
                            vehicle_pos);

                        // If the start orbit is finished, switch to the
                        // transition orbit or line depending on the Dubins path
                        // type. Otherwise, continue publishing start orbit
                        // setpoint messages
                        if (finished)
                        {
                            if (dubins_path.type_is_ccc())
                            {
                                log_info("finished start orbit, executing transition orbit...");
                                state = DUBINS_TRANSITION_ORBIT;
                            }
                            else
                            {
                                state = DUBINS_TRANSITION_LINE;
                                log_info("finished start orbit, executing transition line...");
                            }
                        }
                        else
                        {
                            publish_orbit_msg(dubins_path.start_orbit);
                        }

                        break;

                    }

                    case DUBINS_TRANSITION_ORBIT:
                    {

                        // Check if we've finished the transition orbit. If not,
                        // publish an orbit setpoint message

                        // Check if we've finished the transition orbit by
                        // checking if the vehicle crossed the half plane
                        // between the transition orbtit center and the second
                        // transition point
                        bool finished = crossed_orbit_half_plane(
                            dubins_path.transition_orbit,
                            dubins_path.transition_pos2,
                            vehicle_pos_prev,
                            vehicle_pos);

                        // If the transition orbit is finished, switch to
                        // the end orbit. Otherwise, continue publishing
                        // transition orbit setpoint messages
                        if (finished)
                        {
                            log_info("finished transition orbit, executing end orbit...");
                            state = DUBINS_END_ORBIT;
                        }
                        else
                        {
                            publish_orbit_msg(dubins_path.transition_orbit);
                        }


                        break;

                    }

                    case DUBINS_TRANSITION_LINE:
                    {

                        // Check if we've finished the transition line. If not,
                        // publish a line setpoint message

                        // Check if we've finished the transition line by
                        // checking if the vehicle crossed the half plane
                        // between the end orbtit center and the second
                        // transition point
                        bool finished = crossed_orbit_half_plane(
                            dubins_path.end_orbit,
                            dubins_path.transition_pos2,
                            vehicle_pos_prev,
                            vehicle_pos);

                        // If the transition line is finished, switch to
                        // the end orbit. Otherwise, continue publishing
                        // transition line setpoint messages
                        if (finished)
                        {
                            log_info("finished transition line, executing end orbit...");
                            state = DUBINS_END_ORBIT;
                        }
                        else
                        {
                            publish_line_msg(dubins_path.transition_line);
                        }

                        break;

                    }

                    case DUBINS_END_ORBIT:
                    {

                        // Check if we've finished the end orbit. If not,
                        // publish an orbit setpoint message

                        // Check if we've finished the end orbit by
                        // checking if the vehicle crossed the half plane
                        // between the end orbtit center and the end point
                        bool finished = crossed_orbit_half_plane(
                            dubins_path.end_orbit,
                            dubins_path.end_point.position,
                            vehicle_pos_prev,
                            vehicle_pos);

                        // If the end orbit is finished, we have reached the
                        // end and finished the Dubins path. The action can be
                        // finished. Otherwise, continue publishing end orbit
                        // setpoint messages
                        if (finished)
                        {
                            // Finish the action successfully
                            log_info("finished end orbit");
                            Result result;
                            result.success = true;
                            finish_action(true, result);
                            feedback.percent = 100.0;
                            return feedback;
                        }
                        else
                        {
                            publish_orbit_msg(dubins_path.end_orbit);
                        }

                        break;

                    }

                } // switch(state)

                break;

            } // case WAYPOINT_DUBINS

        } // switch(waypoint_type)

        // Publish the other setpoint messages
        Float64SetpointMsg setpoint_msg;
        setpoint_msg.enable = true;

        // Loop through all publishers in the map. If the action has a parameter
        // corresponding to the publisher, publish the setpoint
        for (const auto& entry : pub_map)
        {
            std::string name = entry.first;
            if (action.parameters.has(name))
            {
                setpoint_msg.data =
                    action.parameters.get(name).to_double();
                entry.second.publish(setpoint_msg);
            }
        }

        // Calculate percent complete
        return feedback;

    }

    //--------------------------------------------------------------------------
    // Name:        stop_action
    // Description: Called when a action has been finished or has been canceled
    //              by the client.
    //--------------------------------------------------------------------------
    void stop_action()
    {

        // Publish a disable setpoint to all primitive publishers
        Float64SetpointMsg disable_msg;
        disable_msg.enable = false;
        for (const auto& entry : pub_map)
            entry.second.publish(disable_msg);

        LineSetpointMsg line_msg;
        line_msg.enable = false;
        line_pub.publish(line_msg);

        OrbitSetpointMsg orbit_msg;
        line_msg.enable = false;
        orbit_pub.publish(orbit_msg);

    }

    //--------------------------------------------------------------------------
    // Name:        nav_msg_callback
    // Description: Callback for the navigation topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void nav_msg_callback(const NavigationMsg& message)
    {

        if (!std::isnan(message.lat) && !std::isnan(message.lon))
        {

            nav_valid = true;

            // Save vehicle navigation info
            vehicle_lat_prev = vehicle_lat;
            vehicle_lon_prev = vehicle_lon;
            vehicle_lat = message.lat;
            vehicle_lon = message.lon;
            vehicle_yaw = message.yaw;

        }
        else
        {
            nav_valid = false;
        }

    }

    //--------------------------------------------------------------------------
    // Name:        fault_callback
    // Description: Called when a fault is detected by a monitored subscriber.
    // Arguments:   - fault: fault event structure
    //--------------------------------------------------------------------------
    void fault_callback(const avl::SubscriberFault& fault)
    {

        // If there is an active action, abort it
        if (action_is_active())
        {
            log_warning("timeout fault on topic %s, stopping active action",
                fault.topic.c_str());
            Result result;
            result.success = false;
            finish_action(false, result);
        }

        // Reset the subscriber to allow it to receive messages again
        nav_sub.reset();

        // Mark the navigation values as invalid
        nav_valid = false;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {

        // Configure the nav subscriber
        nav_sub.set_message_rate(get_param<double>("~input_rate"));
        nav_sub.enable_message_rate_check(true);
        nav_sub.subscribe("nav/inertial_nav", 1,
            &WaypointGuidanceNode::nav_msg_callback,
            &WaypointGuidanceNode::fault_callback, this);

        // Configure the setpoint publishers
        line_pub = node_handle->advertise<LineSetpointMsg>("setpoint/line", 1);
        orbit_pub = node_handle->advertise<OrbitSetpointMsg>("setpoint/orbit", 1);
        pub_map["GROUND SPEED"] = node_handle->advertise<Float64SetpointMsg>("setpoint/ground_speed", 1);
        pub_map["WATER SPEED"]  = node_handle->advertise<Float64SetpointMsg>("setpoint/water_speed", 1);
        pub_map["RPM"]          = node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1);
        pub_map["DEPTH"]        = node_handle->advertise<Float64SetpointMsg>("setpoint/depth", 1);
        pub_map["HEIGHT"]       = node_handle->advertise<Float64SetpointMsg>("setpoint/height", 1);

        // Get the turn radius from the config file
        turn_radius = get_param<double>("~turn_radius");

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    WaypointGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
