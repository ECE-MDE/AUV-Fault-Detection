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
// Publishers:  setpoint/rpm (avl_msgs/Float64SetpointMsg)
//              setpoint/yaw (avl_msgs/Float64SetpointMsg)
//
// Subscribers: nav/inertial_nav (avl_msgs/NavigationMsg)
//==============================================================================

// Guidance node base class
#include <avl_guidance/guidance_node.h>

// Monitored subscriber class
#include <avl_core/monitored_subscriber.h>

// ROS message includes
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/Float64SetpointMsg.h>
using namespace avl_msgs;

// Geographic util functions
#include <avl_core/util/geo.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class LoiterGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    LoiterGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/LOITER", argc, argv)
    {

    }

private:

    // Subscriber for navigation data
    MonitoredSubscriber<NavigationMsg> nav_sub;

    // Publishers for controller setpoints
    ros::Publisher rpm_pub;
    ros::Publisher yaw_pub;


    // Action to be executed
    avl::Action action;
    double duration;
    double loiter_lat;
    double loiter_lon;

    // Vehicle navigation info variables
    bool nav_valid = false;
    double vehicle_lat;
    double vehicle_lon;

    // Loiter parameters
    double inner_radius;
    double outer_radius;
    double max_rpm;

    // Flag indicating if the vehicle is current repositioning to reach the
    // inner radius
    bool repositioning = false;

private:

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
        duration = action.parameters.get("DURATION").to_double();
        loiter_lat = avl::deg_to_rad(action.parameters.get("LAT").to_double());
        loiter_lon = avl::deg_to_rad(action.parameters.get("LON").to_double());

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_action
    // Description: Called at the iteration interval while a action is executing.
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {

        // Calculate the distance between the loiter center point and the
        // vehicle to determine if the vehicle is within the inner or
        // outer radius
        double distance = avl::distance(loiter_lat,  loiter_lon,
                                        vehicle_lat, vehicle_lon);

        // Calculate the yaw that the vehicle has to point toward to
        // reach the loiter center point
        double yaw = avl::initial_bearing(vehicle_lat, vehicle_lon,
                                          loiter_lat,  loiter_lon);

        log_data("[loiter] %f %f %f %f",
            distance, yaw, inner_radius, outer_radius);

        // If the vehicle is outside of the outer radius, navigate at maximum
        // RPM back to the inner radius
        if (distance > outer_radius)
        {

            log_debug("outside outer radius");
            repositioning = true;

            // Publish the maximum RPM message
            Float64SetpointMsg rpm_msg;
            rpm_msg.enable = true;
            rpm_msg.data = max_rpm;
            rpm_pub.publish(rpm_msg);

            // Publish the yaw message
            Float64SetpointMsg yaw_msg;
            yaw_msg.enable = true;
            yaw_msg.data = yaw;
            yaw_pub.publish(yaw_msg);

            log_data("[output] %f %f", yaw, max_rpm);

        }

        // If the vehicle is between the inner and outer radius
        else if (distance > inner_radius && repositioning)
        {

           // If the vehicle is currently trying to respoition to the inner
           // radius, navigate at the linearly scaled RPM.
           // outer radius
            if (repositioning)
            {

                log_debug("between outer and radius, repositioning");

                // Calculate the RPM scale factor for maximum RPM at the
                // outer radius and zero RPM at the inner radius
                double rpm_scale = (distance - inner_radius) /
                                   (outer_radius - inner_radius);

                // Publish the scaled RPM message
                Float64SetpointMsg rpm_msg;
                rpm_msg.enable = true;
                rpm_msg.data = rpm_scale*max_rpm;
                rpm_pub.publish(rpm_msg);

                // Publish the yaw message
                Float64SetpointMsg yaw_msg;
                yaw_msg.enable = true;
                yaw_msg.data = yaw;
                yaw_pub.publish(yaw_msg);

                log_data("[output] %f %f", yaw, rpm_scale*max_rpm);

            }

            // If the vehicle is not repositioning, it means we are just
            // drifting out and we do not need to navigate until we reach the
            else
            {
                log_debug("between outer and radius, drifting");
                stop_controllers();
            }

        }

        // If the vehicle is within the inner radius, we do not need to
        // navigate and can disable the controllers
        else
        {
            log_debug("inside inner radius");
            repositioning = false;
            stop_controllers();
        }

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
        Float64SetpointMsg disable_msg;
        disable_msg.enable = false;
        rpm_pub.publish(disable_msg);
        yaw_pub.publish(disable_msg);
    }

    //--------------------------------------------------------------------------
    // Name:        stop_action
    // Description: Called when a action has been finished or has been canceled
    //              by the client.
    //--------------------------------------------------------------------------
    void stop_action()
    {
        stop_controllers();
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[loiter] distance yaw inner_radius outer_radius");
        add_data_header("[loiter] m rad m m");
        add_data_header("[output] yaw rpm");
        add_data_header("[output] rad rpm");

        // Get the loiter parameters from the config file
        inner_radius = get_param<double>("~inner_radius");
        outer_radius = get_param<double>("~outer_radius");
        max_rpm = get_param<double>("~max_rpm");

        // Configure the nav subscriber
        nav_sub.set_message_rate(get_param<double>("~input_rate"));
        nav_sub.enable_message_rate_check(true);
        nav_sub.subscribe("nav/inertial_nav", 1,
            &LoiterGuidanceNode::nav_msg_callback,
            &LoiterGuidanceNode::fault_callback, this);

        // Configure the setpoint publishers
        rpm_pub = node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1);
        yaw_pub = node_handle->advertise<Float64SetpointMsg>("setpoint/yaw", 1);

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    LoiterGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
