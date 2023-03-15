//==============================================================================
// Autonomous Vehicle Library
//
// Description: This guidance node commands the vehicle to perform a sequence
//              of maneuvers in order to dive from the surface to a trigger
//              depth. Maneuvers are performed in a straight line with a
//              specified yaw. The dive sequence consists of the following
//              steps:
//
//                  1. Accelerate from the start RPM to the end RPM over the
//                     course of the specified acceleration duration.
//                  2. Publish a pitch setpoint to maintain a pitch until the
//                     specified goal depth is reached.
//                  3. Upon reaching the goal depth, the action will be
//                     completed.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/rpm (avl_msgs/Float64SetpointMsg)
//              setpoint/yaw (avl_msgs/Float64SetpointMsg)
//              setpoint/elevator (avl_msgs/Float64SetpointMsg)
//
// Subscribers: device/depth (std_msgs/Float64)
//==============================================================================

// Guidance node base class
#include <avl_guidance/guidance_node.h>

// Util functions
#include <avl_core/util/math.h>

// ROS messages
#include <std_msgs/Float64.h>
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/SafetyAbortSrv.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class DiveGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    DiveGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/DIVE", argc, argv)
    {

    }

private:

    // Publishers for controller setpoints
    ros::Publisher yaw_pub;
    ros::Publisher pitch_pub;
    ros::Publisher rpm_pub;

    // Subscriber for depth messages
    ros::Subscriber depth_sub;
    double current_depth = NAN;

    // Minimum depth threshold settings from config file
    double min_depth_threshold;
    double depth_threshold_duration;

    // Action parameters
    double yaw;
    double start_rpm;
    double end_rpm;
    double accel_time;
    double goal_depth;
    double pitch;

    // Slope and intercept for the RPM sweep function
    double m_rpm;
    double b_rpm;

    // Service client for safety abort
    ros::ServiceClient abort_client;

private:

    //--------------------------------------------------------------------------
    // Name:        depth_msg_callback
    // Description: Callback for the depth topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void depth_msg_callback(const std_msgs::Float64& message)
    {
        current_depth = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        start_new_action
    // Description: Called when a new action is received.
    //--------------------------------------------------------------------------
    bool start_new_action(Action action)
    {

        // Get action parameters
        yaw =   avl::deg_to_rad(action.parameters.get("YAW").to_double());
        pitch = avl::deg_to_rad(action.parameters.get("PITCH").to_double());
        goal_depth = action.parameters.get("DEPTH").to_double();
        accel_time = action.parameters.get("ACCEL TIME").to_double();
        start_rpm = action.parameters.get("START RPM").to_double();
        end_rpm =   action.parameters.get("END RPM").to_double();

        // Calculate the slope and intercept for the RPM sweep function
        m_rpm = (end_rpm - start_rpm) / accel_time;
        b_rpm = start_rpm;

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_action
    // Description: Called at the iteration interval while a action is executing.
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {

        // Get the time since the start of the action so that we can get the
        // correct RPM and elevator angle for the period we are in
        double t = get_time_since_start();

        // If the goal depth has been achieved, finish the action successfully
        if (current_depth >= goal_depth)
        {
            Result result;
            result.success = true;
            finish_action(true, result);
            Feedback feedback;
            feedback.percent = 100.0;
            return feedback;
        }

        // If the minimum depth threshold is not reached within the minimum
        // depth duration after the accel period, assume we failed to dive and
        // throw a safety abort
        if (t >= accel_time + depth_threshold_duration &&
            current_depth < min_depth_threshold)
        {
            std::string msg = "dive sequence failed to reach minimum depth";
            log_error(msg);
            SafetyAbortSrv abort_srv;
            abort_srv.request.message = msg;
            abort_client.call(abort_srv);
        }

        // Create the RPM message and publish it
        Float64SetpointMsg rpm_msg;
        rpm_msg.enable = true;
        rpm_msg.data = get_rpm(t);
        rpm_pub.publish(rpm_msg);

        // Create the yaw message and publish it
        Float64SetpointMsg yaw_msg;
        yaw_msg.enable = true;
        yaw_msg.data = yaw;
        yaw_pub.publish(yaw_msg);

        // Create the pitch message and publish it if the acceleration period is
        // finished
        Float64SetpointMsg pitch_msg;
        pitch_msg.enable = (t >= accel_time);
        pitch_msg.data =   (t >= accel_time) ? pitch : NAN;
        pitch_pub.publish(pitch_msg);

        log_data("[depth] %.3f %.3f",
            current_depth, goal_depth);
        log_data("[output] %.3f %.3f %.3f",
            rpm_msg.data, yaw_msg.data, pitch_msg.data);

        Feedback feedback;
        feedback.percent = current_depth / goal_depth * 100.0;
        return Feedback();

    }

    //--------------------------------------------------------------------------
    // Name:        stop_action
    // Description: Called when a action has been finished or has been canceled
    //              by the client.
    //--------------------------------------------------------------------------
    void stop_action()
    {
        Float64SetpointMsg disable_msg;
        disable_msg.enable = false;
        rpm_pub.publish(disable_msg);
        yaw_pub.publish(disable_msg);
        pitch_pub.publish(disable_msg);
    }

    //--------------------------------------------------------------------------
    // Name:        get_rpm
    // Description: Sweep RPM linearly up to the max based on time.
    // Arguments:   - t: time in seconds since the dive controller started
    // Returns:     RPM output for the given time.
    //--------------------------------------------------------------------------
    double get_rpm(double t)
    {
        if (t < accel_time)
            return m_rpm*t + b_rpm;
        else
            return start_rpm;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[depth] depth goal\\_depth");
        add_data_header("[depth] m m");
        add_data_header("[output] RPM yaw pitch");
        add_data_header("[output] RPM rad rad");

        // Get config file settings
        min_depth_threshold = get_param<double>("~min_depth_threshold");
        depth_threshold_duration = get_param<double>("~depth_threshold_duration");

        // Set up the publishers and subscribers
        rpm_pub = node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1);
        yaw_pub = node_handle->advertise<Float64SetpointMsg>("setpoint/yaw", 1);
        pitch_pub = node_handle->advertise<Float64SetpointMsg>("setpoint/pitch", 1);
        depth_sub = node_handle->subscribe("device/depth", 1,
            &DiveGuidanceNode::depth_msg_callback, this);
        abort_client = node_handle->serviceClient<SafetyAbortSrv>(
            "system/abort_safety_node");

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    DiveGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
