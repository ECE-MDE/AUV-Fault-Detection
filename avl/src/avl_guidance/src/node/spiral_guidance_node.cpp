//==============================================================================
// Autonomous Vehicle Library
//
// Description: This guidance node commands the vehicle to perform a spiral
//              upwards or downwards to a specified depth. This spiral is
//              defined by a center point at the vehicle's position when the
//              action starts, and the radius, pitch, and speed defined by the
//              action parameters.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/orbit (avl_msgs/OrbitSetpointMsg)
//              setpoint/ground_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/water_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/rpm (avl_msgs/Float64SetpointMsg)
//              setpoint/pitch (avl_msgs/Float64SetpointMsg)
//
// Subscribers: nav/inertial_nav (avl_msgs/NavigationMsg)
//              device/depth (std_msgs/Float64)
//==============================================================================

// Guidance node base class
#include <avl_guidance/guidance_node.h>

// Util functions
#include <avl_core/util/math.h>

// Monitored subscriber class
#include <avl_core/monitored_subscriber.h>

// ROS message includes
#include <avl_msgs/NavigationMsg.h>
#include <std_msgs/Float64.h>
#include <avl_msgs/OrbitSetpointMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class SpiralGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    SpiralGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/SPIRAL", argc, argv)
    {

    }

private:

    // Subscriber for navigation data
    MonitoredSubscriber<NavigationMsg> nav_sub;

    // Subscriber for depth messages
    ros::Subscriber depth_sub;

    // Publishers for controller setpoints
    ros::Publisher orbit_pub;
    ros::Publisher pitch_pub;
    std::map<std::string, ros::Publisher> pub_map;

    // Action to be executed
    avl::Action action;
    double radius;
    double goal_depth;
    double pitch;

    // Vehicle navigation info variables
    bool nav_valid = false;
    double vehicle_lat;
    double vehicle_lon;
    double vehicle_depth;

    // Surfacing orbit parameters
    double orbit_center_lat;
    double orbit_center_lon;
    double direction;

private:

    //--------------------------------------------------------------------------
    // Name:        depth_msg_callback
    // Description: Callback for the depth topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void depth_msg_callback(const std_msgs::Float64& message)
    {
        vehicle_depth = message.data;
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
        radius = action.parameters.get("RADIUS").to_double();
        goal_depth = action.parameters.get("GOAL DEPTH").to_double();
        pitch = avl::deg_to_rad(action.parameters.get("PITCH").to_double());

        // Save the vehicle's current position as the spiral orbit center
        orbit_center_lat = vehicle_lat;
        orbit_center_lon = vehicle_lon;
        log_info("orbit_center_lat: %f", orbit_center_lat);
        log_info("orbit_center_lon: %f", orbit_center_lat);

        // Determine if we are going up (1) or down (-1) based on the current
        // depth and the depth specified in the action
        direction = (vehicle_depth > goal_depth) ? 1.0 : -1.0;

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_action
    // Description: Called at the iteration interval while a action is executing.
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {

        // If the goal depth has been passed based on the direction we are
        // going, finish the action successfully
        if (direction*vehicle_depth <= direction*goal_depth)
        {
            Result result;
            result.success = true;
            finish_action(true, result);
            Feedback feedback;
            feedback.percent = 100.0;
            return feedback;
        }

        // Publish the setpoint messages
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

        // Create the orbit message and publish it
        OrbitSetpointMsg orbit_msg;
        orbit_msg.enable = true;
        orbit_msg.lat = orbit_center_lat;
        orbit_msg.lon = orbit_center_lon;
        orbit_msg.radius = radius;
        orbit_msg.clockwise = true;
        orbit_pub.publish(orbit_msg);

        // Create the pitch message and publish it
        Float64SetpointMsg pitch_msg;
        pitch_msg.enable = true;
        pitch_msg.data = direction * abs(pitch);
        pitch_pub.publish(pitch_msg);

        return Feedback();

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
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {

        // Configure the depth subscriber
        depth_sub = node_handle->subscribe("device/depth", 1,
            &SpiralGuidanceNode::depth_msg_callback, this);

        // Configure the nav subscriber
        nav_sub.set_message_rate(get_param<double>("~input_rate"));
        nav_sub.enable_message_rate_check(true);
        nav_sub.subscribe("nav/inertial_nav", 1,
            &SpiralGuidanceNode::nav_msg_callback,
            &SpiralGuidanceNode::fault_callback, this);

        // Configure the setpoint publishers
        orbit_pub = node_handle->advertise<OrbitSetpointMsg>("setpoint/orbit", 1);
        pitch_pub = node_handle->advertise<Float64SetpointMsg>("setpoint/pitch", 1);
        pub_map["GROUND SPEED"] = node_handle->advertise<Float64SetpointMsg>("setpoint/ground_speed", 1);
        pub_map["WATER SPEED"]  = node_handle->advertise<Float64SetpointMsg>("setpoint/water_speed", 1);
        pub_map["RPM"]          = node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1);

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    SpiralGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
