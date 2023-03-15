//==============================================================================
// Autonomous Vehicle Library
//
// Description: Guidance node for executing a figure eight maneuver.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/orbit (avl_msgs/OrbitSetpointMsg)
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
#include <avl_msgs/OrbitSetpointMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class FigureEightGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    FigureEightGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/FIGURE_EIGHT", argc, argv)
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

    // Vehicle NED position variables
    double origin_lat;
    double origin_lon;
    Position origin_pos;
    Position vehicle_pos;
    Position vehicle_pos_prev;

    // Publishers for controller setpoints
    ros::Publisher orbit_pub;
    std::map<std::string, ros::Publisher> pub_map;

    // Figure eight parameters
    avl::Action action;
    double heading;
    double radius;

    // Orbits
    Orbit orbit1;
    Orbit orbit2;
    OrbitSetpointMsg orbit1_msg;
    OrbitSetpointMsg orbit2_msg;

    // Flag indicating which orbit is being executed
    bool on_orbit1;

    // Total arc angle remaining to be traversed to complete the current orbit.
    // When this reaches 0 radians, we switch orbits and reset it to 2pi
    double remaining_arc_angle;

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

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Save the action and get the start and end points of the line
        this->action = action;
        origin_lat = avl::deg_to_rad(action.parameters.get("LAT").to_double());
        origin_lon = avl::deg_to_rad(action.parameters.get("LON").to_double());
        heading = avl::deg_to_rad(action.parameters.get("HEADING").to_double());
        radius =  action.parameters.get("RADIUS").to_double();

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Create the two orbits that form the figure eight
        orbit1.north =     radius*cos(heading);
        orbit1.east =      radius*sin(heading);
        orbit1.radius =    radius;
        orbit1.direction = RIGHT;

        orbit2.north =    -orbit1.north;
        orbit2.east =     -orbit1.east;
        orbit2.radius =    radius;
        orbit2.direction = LEFT;

        // Convert the orbit center points to geodetic coordinates
        double orbit1_lat, orbit1_lon, orbit1_alt;
        ned_to_geo(origin_lat,   origin_lon, 0.0,
                   orbit1.north, orbit1.east, 0.0,
                   orbit1_lat,   orbit1_lon, orbit1_alt);

        double orbit2_lat, orbit2_lon, orbit2_alt;
        ned_to_geo(origin_lat,   origin_lon, 0.0,
                   orbit2.north, orbit2.east, 0.0,
                   orbit2_lat,   orbit2_lon, orbit2_alt);

        // Construct orbit setpoint messages
        orbit1_msg.enable = true;
        orbit1_msg.lat = orbit1_lat;
        orbit1_msg.lon = orbit1_lon;
        orbit1_msg.radius = radius;
        orbit1_msg.clockwise = true;

        orbit2_msg.enable = true;
        orbit2_msg.lat = orbit2_lat;
        orbit2_msg.lon = orbit2_lon;
        orbit2_msg.radius = radius;
        orbit2_msg.clockwise = false;

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Choose the initial orbit

        // Calculate the vehicle's current position in NED coordinates
        double down;
        geo_to_ned(origin_lat,        origin_lon,       0.0,
                   vehicle_lat,       vehicle_lon,      0.0,
                   vehicle_pos.north, vehicle_pos.east, down);

        // Determine which orbit to start on by calculating which orbit we are
        // closest to.
        double distance_orbit1 = sqrt(pow(vehicle_pos.east -  orbit1.east,2) +
                                      pow(vehicle_pos.north - orbit1.north,2));

        double distance_orbit2 = sqrt(pow(vehicle_pos.east -  orbit2.east,2) +
                                      pow(vehicle_pos.north - orbit2.north,2));

        on_orbit1 = distance_orbit1 <= distance_orbit2;

        // Calculate how much arc angle is left to complete the first orbit
        remaining_arc_angle = get_arc_angle(on_orbit1 ? orbit1 : orbit2,
            vehicle_pos, origin_pos);

        log_info("starting on orbit %d", on_orbit1 ? 1 : 2);

        // Mark the action as accepted
        return true;

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

        // Remove from the total remaining arc angle to be traversed
        remaining_arc_angle -= avl::wrap_to_pi(
            get_arc_angle(on_orbit1 ? orbit1 : orbit2,
                vehicle_pos_prev, vehicle_pos));

        // Check for orbit completion, which is when we've completed a rotation
        // of 2pi radians through the orbit
        if (remaining_arc_angle <= 0.0)
        {
            log_info("orbit completed, switching orbits...");
            on_orbit1 = !on_orbit1;
            remaining_arc_angle = 2.0*M_PI;
        }

        // Publish the orbit setpoint message
        orbit_pub.publish(on_orbit1 ? orbit1_msg : orbit2_msg);

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
            &FigureEightGuidanceNode::nav_msg_callback,
            &FigureEightGuidanceNode::fault_callback, this);

        // Configure the setpoint publishers
        orbit_pub = node_handle->advertise<OrbitSetpointMsg>("setpoint/orbit", 1);
        pub_map["GROUND SPEED"] = node_handle->advertise<Float64SetpointMsg>("setpoint/ground_speed", 1);
        pub_map["WATER SPEED"]  = node_handle->advertise<Float64SetpointMsg>("setpoint/water_speed", 1);
        pub_map["RPM"]          = node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1);
        pub_map["DEPTH"]        = node_handle->advertise<Float64SetpointMsg>("setpoint/depth", 1);
        pub_map["HEIGHT"]       = node_handle->advertise<Float64SetpointMsg>("setpoint/height", 1);

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    FigureEightGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
