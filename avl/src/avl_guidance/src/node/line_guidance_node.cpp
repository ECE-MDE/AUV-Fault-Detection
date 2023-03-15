//==============================================================================
// Autonomous Vehicle Library
//
// Description: Guidance node for handling line actions. Line actions cause the
//              vehicle to navigate in a straight line between a start and an
//              end location by planning a Dubins path from wherever the vehicle
//              starts to the start point such that the vehicle is pointing
//              along the line. It then navigates along the line to the end
//              point.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/line (avl_msgs/LineSetpointMsg)
//              setpoint/orbit (avl_msgs/OrbitSetpointMsg)
//              setpoint/ground_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/water_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/rpm (avl_msgs/Float64SetpointMsg)
//              setpoint/depth (avl_msgs/Float64SetpointMsg)
//              setpoint/height (avl_msgs/Float64SetpointMsg)
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
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/LineSetpointMsg.h>
#include <avl_msgs/OrbitSetpointMsg.h>
using namespace avl_msgs;

//==============================================================================
//                                 ENUMS
//==============================================================================

// Enum listing possible states for Dubins path following
enum DubinsState
{
    DUBINS_START_ORBIT,
    DUBINS_TRANSITION_ORBIT,
    DUBINS_TRANSITION_LINE,
    DUBINS_END_ORBIT,
    DUBINS_END_LINE
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class LineGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    LineGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/LINE", argc, argv)
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

    // Dubins flag indicates to use a dubins paths when true
    bool use_dubins = true;

    // Publishers for controller setpoints
    ros::Publisher line_pub;
    ros::Publisher orbit_pub;
    std::map<std::string, ros::Publisher> pub_map;

    // Action to be executed
    avl::Action action;
    double start_lat;
    double start_lon;
    double end_lat;
    double end_lon;

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

        // Save the action and get the start and end points of the line
        this->action = action;
        start_lat = avl::deg_to_rad(action.parameters.get("START LAT").to_double());
        start_lon = avl::deg_to_rad(action.parameters.get("START LON").to_double());
        end_lat =   avl::deg_to_rad(action.parameters.get("END LAT").to_double());
        end_lon =   avl::deg_to_rad(action.parameters.get("END LON").to_double());
        use_dubins = action.parameters.get("DUBINS").to_bool();

        // Save the current vehicle position as the origin for conversion from
        // geodetic coordinates to NED coordinates
        origin_lat = vehicle_lat;
        origin_lon = vehicle_lon;

        // If we are going to follow a Dubins path, set the state to the start
        // orbit to begin the Dubins path. If we are not going to follow a
        // Dubins path, set the state directly to dubins end line
        state = use_dubins ? DUBINS_START_ORBIT : DUBINS_END_LINE;

        // We wish to calculate a Dubins path between the vehicle's starting
        // point to the line's start point, and then we will follow a line
        // between the line start and end points

        // Calculate the yaw between the line start and end points
        double line_yaw = initial_bearing(start_lat, start_lon,
                                          end_lat,   end_lon);

        // Convert the line start location to NED coordinates
        double line_start_north;
        double line_start_east;
        double line_start_down;
        geo_to_ned(origin_lat,       origin_lon,       0.0,
                   start_lat,        start_lon,        0.0,
                   line_start_north, line_start_east,  line_start_down);

        // Convert the line end location to NED coordinates
        double line_end_north;
        double line_end_east;
        double line_end_down;
        geo_to_ned(origin_lat,     origin_lon,     0.0,
                   end_lat,        end_lon,        0.0,
                   line_end_north, line_end_east,  line_end_down);

        // Create the start point and end points for the Dubins path. Since the
        // origin lat/lon is set to the vehicle's current position, the vehicle
        // always starts at the NED origin
        DubinsPoint vehicle_start_point;
        vehicle_start_point.position.north = 0.0;
        vehicle_start_point.position.east = 0.0;
        vehicle_start_point.yaw = vehicle_yaw;
        vehicle_start_point.radius = turn_radius;

        // Create the line start point
        DubinsPoint line_start_point;
        line_start_point.position.north = line_start_north;
        line_start_point.position.east = line_start_east;
        line_start_point.yaw = line_yaw;
        line_start_point.radius = turn_radius;

        // Create the line end point
        DubinsPoint line_end_point;
        line_end_point.position.north = line_end_north;
        line_end_point.position.east = line_end_east;
        line_end_point.yaw = line_yaw;
        line_end_point.radius = turn_radius;

        // Calculate the Dubins path between the start and end points
        dubins_path = get_dubins_path(vehicle_start_point, line_start_point);

        log_data("[path] %f %f %f %f %f %f %f %f %d %f %f %f %d %f %f %f "
            "%d %f %f %f %f %f %f %f %f %f %f %f %d %f %f",
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
            dubins_path.transition_orbit.direction,

            line_end_point.position.north,
            line_end_point.position.east);

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
    // Description: Called at the iteration interval while a action is executing.
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

        log_data("[vehicle_pos] %f %f", vehicle_pos.north, vehicle_pos.east);

        // Check which segment of the Dubins path we are on and publish
        // the correct line or orbit setpoint message
        switch (state)
        {

            case DUBINS_START_ORBIT:
            {

                // Check if we've finished the start orbit by checking
                // if the vehicle crossed the half plane between the
                // orbit center and the first transition point
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
                // between the transition orbit center and the second
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
                // between the end orbit center and the second
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
                // between the end orbit center and the end point
                bool finished = crossed_orbit_half_plane(
                    dubins_path.end_orbit,
                    dubins_path.end_point.position,
                    vehicle_pos_prev,
                    vehicle_pos);

                // If the end orbit is finished, switch to the end line.
                // Otherwise, continue publishing end line setpoint messages
                if (finished)
                {
                    log_info("finished end orbit, executing end line...");
                    state = DUBINS_END_LINE;
                }
                else
                {
                    publish_orbit_msg(dubins_path.end_orbit);
                }

                break;

            }

            case DUBINS_END_LINE:
            {

                // Check if we've finished the transition line. If not,
                // publish a line setpoint message

                // Check if we've finished the final line by checking if the
                // vehicle crossed the half plane at the end point of the line
                bool finished = crossed_half_plane(
                    start_lat,   start_lon,
                    end_lat,     end_lon,
                    vehicle_lat, vehicle_lon);

                // If the end line is finished, we have reached the end and
                // finished the line. The action can be finished. Otherwise,
                // continue publishing end line setpoint messages
                if (finished)
                {
                    // Finish the action successfully
                    log_info("finished end line");
                    Result result;
                    result.success = true;
                    finish_action(true, result);
                    feedback.percent = 100.0;
                    return feedback;
                }
                else
                {
                    LineSetpointMsg msg;
                    msg.enable = true;
                    msg.lat_start = start_lat;
                    msg.lon_start = start_lon;
                    msg.lat_end = end_lat;
                    msg.lon_end = end_lon;
                    line_pub.publish(msg);
                }

                break;

            }

        } // switch(state)

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
        orbit_msg.enable = false;
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

        // Add data log headers
        add_data_header("[vehicle_pos] North East");
        add_data_header("[vehicle_pos] m m");

        // Configure the nav subscriber
        nav_sub.set_message_rate(get_param<double>("~input_rate"));
        nav_sub.enable_message_rate_check(true);
        nav_sub.subscribe("nav/inertial_nav", 1,
            &LineGuidanceNode::nav_msg_callback,
            &LineGuidanceNode::fault_callback, this);

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
    LineGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
