//==============================================================================
// Autonomous Vehicle Library
//
// Description: This guidance node commands the vehicle to maintain a surface
//              position defined by a center point and an inner and outer
//              radius. While outside of the outer radius, the vehicle will
//              attempt to navigate back to the inner radius. Outside of the
//              outer radius, the vehicle will navigate with the maximum
//              configured RPM. The RPM will scale down linearly from the max
//              to zero between the outer and inner radius.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  /setpoint/rpm (avl_msgs/Float64SetpointMsg)
//              /setpoint/yaw (avl_msgs/Float64SetpointMsg)
//
// Subscribers: /nav/inertial_nav (avl_msgs/NavigationMsg)
//==============================================================================

// Guidance node base class
#include <avl_guidance/guidance_node.h>

// Util functions
#include <avl_core/util/math.h>

// Monitored subscriber class
#include <avl_core/monitored_subscriber.h>

// ROS message includes
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/OrbitSetpointMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
using namespace avl_msgs;

// Packet handler class
#include <avl_comms/packet_handler.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class FollowGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    FollowGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/FOLLOW", argc, argv)
    {

    }

private:

    // Subscriber for navigation data
    MonitoredSubscriber<NavigationMsg> nav_sub;

    // Publishers for controller setpoints
    ros::Publisher orbit_pub;
    std::map<std::string, ros::Publisher> pub_map;

    // Action to be executed
    avl::Action action;
    double duration;
    uint8_t target_id = 0;
    double radius;

    // Vehicle navigation info variables
    bool nav_valid = false;
    double vehicle_lat;
    double vehicle_lon;

    // Packet handler for handling incoming packets
    PacketHandler packet_handler;

    // Most recent heartbeat from the target vehicle
    Heartbeat target_heartbeat;

private:

    //--------------------------------------------------------------------------
    // Name:        packet_callback
    // Description: Called when an AVL packet is received by the communication
    //              architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - packet: Received packet.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool packet_callback(CommsChannel channel, CommsInterface interface,
        Packet packet, bool& result, std::vector<uint8_t>& data)
    {

        PacketDescriptor desc = packet.get_descriptor();
        PacketHeader header = packet.get_header();

        try
        {

            // Handle HEARTBEAT packets
            if (desc == HEARTBEAT_PACKET && header.source_id == target_id)
            {
                HeartbeatPacket heartbeat_packet(packet);
                target_heartbeat = heartbeat_packet.get_heartbeat();
            }

        }
        catch (const std::exception& ex)
        {
            std::string message(ex.what());
            data = std::vector<uint8_t>(message.begin(), message.end());
            log_warning("failed to parse HEARTBEAT packet (%s)", ex.what());
            result = false;
            return true;
        }

        return false;

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

        // Save the action and get the loiter point coordunates
        this->action = action;
        duration =  action.parameters.get("DURATION").to_double();
        target_id = action.parameters.get("TARGET ID").to_int();
        radius =    action.parameters.get("RADIUS").to_double();

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_action
    // Description: Called at the iteration interval while a action is executing.
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {

        // Publish the orbit setpoint
        OrbitSetpointMsg orbit_msg;
        orbit_msg.enable = true;
        orbit_msg.lat = avl::deg_to_rad(target_heartbeat.nav_lat);
        orbit_msg.lon = avl::deg_to_rad(target_heartbeat.nav_lon);
        orbit_msg.radius = radius;
        orbit_msg.clockwise = true;
        orbit_pub.publish(orbit_msg);

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

        // Calculate feedback
        Feedback feedback;
        feedback.percent = get_time_since_start() / duration * 100.0;
        return feedback;

    }

    //--------------------------------------------------------------------------
    // Name:        stop_controllers
    // Description: Sends NAN setpoints to the controllers to stop control.
    //--------------------------------------------------------------------------
    void stop_controllers()
    {

        OrbitSetpointMsg orbit_msg;
        orbit_msg.enable = false;
        orbit_pub.publish(orbit_msg);

        // Publish a disable setpoint to all primitive publishers
        Float64SetpointMsg disable_msg;
        disable_msg.enable = false;
        for (const auto& entry : pub_map)
            entry.second.publish(disable_msg);

    }

    //--------------------------------------------------------------------------
    // Name:        stop_action
    // Description: Called when a action has been finished or has been canceled
    //              by the client.
    //--------------------------------------------------------------------------
    void stop_action()
    {
        stop_controllers();
        target_id = 0;
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
            &FollowGuidanceNode::nav_msg_callback,
            &FollowGuidanceNode::fault_callback, this);

        // Configure the setpoint publishers
        orbit_pub = node_handle->advertise<OrbitSetpointMsg>("setpoint/orbit", 1);
        pub_map["GROUND SPEED"] = node_handle->advertise<Float64SetpointMsg>("setpoint/ground_speed", 1);
        pub_map["WATER SPEED"]  = node_handle->advertise<Float64SetpointMsg>("setpoint/water_speed", 1);
        pub_map["RPM"]          = node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1);
        pub_map["DEPTH"]        = node_handle->advertise<Float64SetpointMsg>("setpoint/depth", 1);
        pub_map["HEIGHT"]       = node_handle->advertise<Float64SetpointMsg>("setpoint/height", 1);

        // Set the packet handler's callback to handle incoming packets
        packet_handler.set_callback(&FollowGuidanceNode::packet_callback, this);

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    FollowGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
