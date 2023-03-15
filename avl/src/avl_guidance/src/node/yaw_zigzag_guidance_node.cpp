//==============================================================================
// Autonomous Vehicle Library
//
// Description: A guidance node for executing zigzag yaw maneuvers.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/rudder (avl_msgs/Float64SetpointMsg)
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

// AVL Includes
#include <avl_core/util/math.h>
#include <avl_core/monitored_subscriber.h>

// ROS message includes
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class YawZigzagGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    YawZigzagGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/YAW_ZIGZAG", argc, argv)
    {

    }

private:

    // Subscriber for navigation data
    MonitoredSubscriber<NavigationMsg> nav_sub;

    // Vehicle navigation info variables
    bool nav_valid = false;
    double vehicle_yaw = NAN;

    // Action to be executed
    avl::Action action;

    // Rudder angle to be published
    double rudder_angle;

    // Publisher for rudder output for control node
    ros::Publisher rudder_angle_pub;
    std::map<std::string, ros::Publisher> pub_map;

private:

    //--------------------------------------------------------------------------
    // Name:        nav_msg_callback
    // Description: Callback for the navigation topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void nav_msg_callback(const NavigationMsg& message)
    {
        nav_valid = true;
        vehicle_yaw = message.yaw;
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
    // Name:        start_new_task
    // Description:
    //--------------------------------------------------------------------------
    bool start_new_action(Action action)
    {

        this->action = action;

        // Get the initial rudder angle from the action
        rudder_angle = abs(avl::deg_to_rad(
            action.parameters.get("RUDDER ANGLE").to_double()));

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_task
    // Description:
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {

        // Get the zigzag parameters from the action
        double center_yaw = avl::deg_to_rad(
            action.parameters.get("CENTER YAW").to_double());
        double yaw_deviation = abs(avl::deg_to_rad(
            action.parameters.get("OFFSET").to_double()));

        // Flip the rudder angle if the vehicle yaw passes the yaw deviation
        // angle and the rudder has not already been flipped
        double offset = avl::wrap(vehicle_yaw - center_yaw);
        if ((offset >= yaw_deviation && rudder_angle > 0.0) ||
            (offset < -yaw_deviation && rudder_angle < 0.0))
            rudder_angle = -rudder_angle;

        // Create the rudder setpoint message and publish it
        Float64SetpointMsg rudder_angle_msg;
        rudder_angle_msg.enable = true;
        rudder_angle_msg.data = rudder_angle;
        rudder_angle_pub.publish(rudder_angle_msg);

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

        log_data("[zigzag] %.4f %.4f %.4f %.4f" ,
            vehicle_yaw, center_yaw, yaw_deviation, rudder_angle);

        // Calculate completion percentage for feedback
        Feedback feedback;
        feedback.percent =  100.0;
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

        Float64SetpointMsg rudder_angle_msg;
        rudder_angle_msg.enable = false;
        rudder_angle_pub.publish(rudder_angle_msg);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {

      // Add data log headers
      add_data_header("[zigzag] vehicle\\_yaw center\\_yaw yaw\\_deviation rudder\\_angle");
      add_data_header("[zigzag] rad rad rad rad");

      // Set up the publishers and subscribers
      rudder_angle_pub = node_handle->advertise<Float64SetpointMsg>("setpoint/rudder", 1);
      pub_map["GROUND SPEED"] = node_handle->advertise<Float64SetpointMsg>("setpoint/ground_speed", 1);
      pub_map["WATER SPEED"]  = node_handle->advertise<Float64SetpointMsg>("setpoint/water_speed", 1);
      pub_map["RPM"]          = node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1);
      pub_map["DEPTH"]        = node_handle->advertise<Float64SetpointMsg>("setpoint/depth", 1);
      pub_map["HEIGHT"]       = node_handle->advertise<Float64SetpointMsg>("setpoint/height", 1);

      nav_sub.set_message_rate(get_param<double>("~input_rate"));
      nav_sub.enable_message_rate_check(true);
      nav_sub.subscribe("nav/inertial_nav", 1,
          &YawZigzagGuidanceNode::nav_msg_callback,
          &YawZigzagGuidanceNode::fault_callback, this);

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    YawZigzagGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
