//==============================================================================
// Autonomous Vehicle Library
//
// Description: A guidance node for executing zigzag yaw maneuvers.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/elevator (avl_msgs/Float64SetpointMsg)
//              setpoint/ground_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/water_speed (avl_msgs/Float64SetpointMsg)
//              setpoint/rpm (avl_msgs/Float64SetpointMsg)
//              setpoint/pitch (avl_msgs/Float64SetpointMsg)
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

class PitchZigzagGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    PitchZigzagGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/PITCH_ZIGZAG", argc, argv)
    {

    }

private:

    // Subscriber for navigation data
    MonitoredSubscriber<NavigationMsg> pitch_sub;

    // Vehicle navigation info variables
    bool pitch_valid = false;
    double vehicle_pitch = NAN;

    // Action to be executed
    avl::Action action;

    // Mean elevator angle and elevator offset angle
    double elevator_mean = 0;
    double elevator_offset = 0;

    // elevator angle to be published
    double elevator_angle;

    // Publisher for elevator output for control node
    ros::Publisher elevator_angle_pub;
    std::map<std::string, std::pair<ros::Publisher, bool>> pub_map;

private:

    //--------------------------------------------------------------------------
    // Name:        pitch_msg_callback
    // Description: Callback for the pitch topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void pitch_msg_callback(const NavigationMsg& message)
    {
        pitch_valid = true;
        vehicle_pitch = message.pitch;
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
        pitch_sub.reset();

        // Mark the navigation values as invalid
        pitch_valid = false;

    }

    //--------------------------------------------------------------------------
    // Name:        start_new_task
    // Description:
    //--------------------------------------------------------------------------
    bool start_new_action(Action action)
    {

        this->action = action;

        // Get the elevator offset from the action
        elevator_offset = abs(avl::deg_to_rad(
            action.parameters.get("ELEVATOR ANGLE").to_double()));

        // Get the elevator mean if it was commanded
        if (action.parameters.has("ELEVATOR MEAN"))
        {
            elevator_mean = avl::deg_to_rad(
                action.parameters.get("ELEVATOR MEAN").to_double());
        }
        // If not commanded, assume the mean is 0 (symmetric elevator zigzag)
        else
        {
            elevator_mean = 0;
        }

        // Compute the initial elevator angle from the action
        elevator_angle = elevator_mean + elevator_offset;

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_task
    // Description:
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {

        // Get the zigzag parameters from the action
        double center_pitch = avl::deg_to_rad(
            action.parameters.get("CENTER PITCH").to_double());
        double pitch_deviation = avl::deg_to_rad(
            abs(action.parameters.get("OFFSET").to_double()));

        // Flip the elevator angle if the vehicle pitch passes the pitch deviation
        // angle and the elevator has not already been flipped
        double offset = vehicle_pitch - center_pitch;
        if ((offset >= pitch_deviation && elevator_offset > 0.0) ||
            (offset < -pitch_deviation && elevator_offset < 0.0))
        {
            // Flip the offset
            elevator_offset = -elevator_offset;

            // Compute the new elevator angle
            elevator_angle = elevator_mean + elevator_offset;
        }

        // Create the elevator setpoint message and publish it
        Float64SetpointMsg elevator_angle_msg;
        elevator_angle_msg.enable = true;
        elevator_angle_msg.data = elevator_angle;
        elevator_angle_pub.publish(elevator_angle_msg);

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
                double data = action.parameters.get(name).to_double();

                // If this is yaw, convert to radians
                if(entry.second.second)
                {
                    data = avl::deg_to_rad(data);
                }

                setpoint_msg.data = data;

                entry.second.first.publish(setpoint_msg);
            }
        }

        log_data("[zigzag] %.4f %.4f %.4f %.4f" ,
            vehicle_pitch, center_pitch, pitch_deviation, elevator_angle);

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
            entry.second.first.publish(disable_msg);

        Float64SetpointMsg elevator_angle_msg;
        elevator_angle_msg.enable = false;
        elevator_angle_pub.publish(elevator_angle_msg);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {
        // Add data log headers
        add_data_header("[zigzag] vehicle\\_pitch center\\_pitch pitch\\_deviation elevator\\_angle");
        add_data_header("[zigzag] m m m rad");

        // Set up the publishers and subscribers
        elevator_angle_pub      = node_handle->advertise<Float64SetpointMsg>("setpoint/elevator", 1);
        pub_map["GROUND SPEED"] = {node_handle->advertise<Float64SetpointMsg>("setpoint/ground_speed", 1), false};
        pub_map["WATER SPEED"]  = {node_handle->advertise<Float64SetpointMsg>("setpoint/water_speed", 1), false};
        pub_map["RPM"]          = {node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1), false};
        pub_map["YAW"]          = {node_handle->advertise<Float64SetpointMsg>("setpoint/yaw", 1), true};

        pitch_sub.set_message_rate(get_param<double>("~input_rate"));
        pitch_sub.enable_message_rate_check(true);
        pitch_sub.subscribe("nav/inertial_nav", 1,
            &PitchZigzagGuidanceNode::pitch_msg_callback,
            &PitchZigzagGuidanceNode::fault_callback, this);
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    PitchZigzagGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
