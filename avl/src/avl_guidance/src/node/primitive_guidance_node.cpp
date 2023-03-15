//==============================================================================
// Autonomous Vehicle Library
//
// Description: Guidance node for primitive control of vehicle controllers such
//              attitude and depth.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/roll (avl_msgs/LineSetpointMsg)
//              setpoint/pitch (avl_msgs/OrbitSetpointMsg)
//              setpoint/yaw (avl_msgs/Float64SetpointMsg)
//              setpoint/ground_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/water_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/rpm (avl_msgs/Float64SetpointMsg)
//              setpoint/depth (avl_msgs/Float64SetpointMsg)
//              setpoint/height (avl_msgs/Float64SetpointMsg)
//
// Subscribers: None
//==============================================================================

// Guidance node base class
#include <avl_guidance/guidance_node.h>

// Util functions
#include <avl_core/util/math.h>

// ROS message includes
#include <avl_msgs/Float64SetpointMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class PrimitiveGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    PrimitiveGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/PRIMITIVE", argc, argv)
    {

    }

private:

    // Publishers for controller setpoints
    std::map<std::string, std::pair<ros::Publisher, bool>> pub_map;

    // Action to be executed
    avl::Action action;
    double duration;

private:

    //--------------------------------------------------------------------------
    // Name:        start_new_action
    // Description: Called when a new action is received.
    //--------------------------------------------------------------------------
    bool start_new_action(Action action)
    {
        this->action = action;
        duration = action.parameters.get("DURATION").to_double();
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        iterate_action
    // Description: Called at the iteration interval while a action is executing.
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {

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
                if (entry.second.second)
                    setpoint_msg.data = avl::deg_to_rad(setpoint_msg.data);
                entry.second.first.publish(setpoint_msg);
            }
        }

        // Calculate completion percentage for feedback
        Feedback feedback;
        feedback.percent = get_time_since_start() / duration * 100.0;
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
            entry.second.first.publish(disable_msg);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {

        // Configure the setpoint publishers in the map
        pub_map["ROLL"]         = {node_handle->advertise<Float64SetpointMsg>("setpoint/roll", 1),         true};
        pub_map["PITCH"]        = {node_handle->advertise<Float64SetpointMsg>("setpoint/pitch", 1),        true};
        pub_map["YAW"]          = {node_handle->advertise<Float64SetpointMsg>("setpoint/yaw", 1),          true};
        pub_map["GROUND SPEED"] = {node_handle->advertise<Float64SetpointMsg>("setpoint/ground_speed", 1), false};
        pub_map["WATER SPEED"]  = {node_handle->advertise<Float64SetpointMsg>("setpoint/water_speed", 1),  false};
        pub_map["RPM"]          = {node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1),          false};
        pub_map["DEPTH"]        = {node_handle->advertise<Float64SetpointMsg>("setpoint/depth", 1),        false};
        pub_map["HEIGHT"]       = {node_handle->advertise<Float64SetpointMsg>("setpoint/height", 1),       false};
        pub_map["RUDDER"]       = {node_handle->advertise<Float64SetpointMsg>("setpoint/rudder", 1),       true};
        pub_map["ELEVATOR"]     = {node_handle->advertise<Float64SetpointMsg>("setpoint/elevator", 1),     true};
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    PrimitiveGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
