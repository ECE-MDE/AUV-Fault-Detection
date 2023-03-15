//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node that monitors a collection of topics and verifies there
//              are no failures within the system. In this instance we'll define
//              a failure as some monitored, reported value being out of range.
//
//              Rather than embedding the fault logic into the various nodes
//              publishing data, monitoring for faults is done by subscribing to
//              and monitoring topics of interest within this node. That way we
//              have a centralized place to detect and respond to faults.
//
//              Parameter limits are set in this node's config file. The
//              following parameters and their corresponding topics are
//              monitored:
//
//                  - Height (device/height): Vehicle height in meters
//                  - Attitude (device/ahrs): Vehicle roll/pitch in degrees
//                  - Depth (device/depth): Vehicle depth in meters
//                  - Voltage (device/voltage): Battery voltage in volts
//                  - Position (/navigation/nav): Vehicle position in lat/lon
//
//              When a satefy limit is violated, the safety node will publish to
//              the device/actuator_control topic to disable actuators, disable
//              missions using the mission manager's system/mission_mode
//              service, and log an error message.
//
// Servers:     system/abort_safety_node (std_srvs/SafetyAbortSrv)
//              system/reset_safety_node (std_srvs/Trigger)
//
// Clients:     system/mission_mode (avl_msgs/MissionModeSrv)
//
// Publishers:  system/safety_status (avl_msgs/SafetyStatusMsg)
//              device/actuator_control (avl_devices/ActuatorControlMsg)
//
// Subscribers: nav/inertial_nav (avl_navigation/NavigationMsg)
//              device/depth (std_msgs/Float64)
//              device/height (std_msgs/Float64)
//              device/voltage (std_msgs/Float64)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Monitored subscriber class
#include <avl_core/monitored_subscriber.h>

// Command handler class
#include <avl_comms/command_handler.h>
using namespace avl;

// Util functions
#include <avl_core/util/math.h>
#include <avl_core/util/logic.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>
#include <avl_core/util/misc.h>

// ROS message includes
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>
#include <avl_msgs/SafetyAbortSrv.h>
#include <avl_msgs/MissionModeSrv.h>
#include <avl_msgs/SafetyStatusMsg.h>
#include <avl_msgs/ActuatorControlMsg.h>
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/PathfinderDvlMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class SafetyNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        SafetyNode constructor
    //--------------------------------------------------------------------------
    SafetyNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;
    PacketHandler packet_handler;

    // Service servers
    ros::ServiceServer abort_safety_server;
    ros::ServiceServer reset_safety_server;

    // Service clients
    ros::ServiceClient mission_mode_client;

    // Publishers
    ros::Publisher safety_status_pub;
    ros::Publisher actuator_control_pub;

    // Monitored Subscribers
    MonitoredSubscriber<NavigationMsg> nav_sub;
    MonitoredSubscriber<std_msgs::Float64> depth_sub;
    MonitoredSubscriber<std_msgs::Float64> height_sub;
    MonitoredSubscriber<std_msgs::Float64> voltage_sub;
    MonitoredSubscriber<PathfinderDvlMsg> dvl_sub;

    // Geofence points
    std::vector<double> geofence_lats;
    std::vector<double> geofence_lons;

    // Fault detection flag. The node stops running when this flag is set true
    // due to a limit violation
    bool fault_detected = false;

    // Maximum depth limit. Initially from config file but can be overwritten by
    // a DEPTH LIMIT command
    double max_depth;

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

        // Handle ABORT packets
        if (desc == ABORT_PACKET)
        {

            std::string msg = "abort packet received from ID " +
                std::to_string(header.source_id) + " on channel " +
                channel_to_string(channel);
            log_info(msg);
            violation_handler(msg);

            result = true;
            return true;

        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when a COMMAND packet is received by the
    //              communication architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - command_name: Command name of the received command.
    //              - params: Command parameter list.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool command_callback(CommsChannel channel, CommsInterface interface,
        std::string command_name, ParameterList params,
        bool& result, std::vector<uint8_t>& data)
    {

        // Handle RESET SAFETY commands
        if (command_name == "RESET SAFETY")
        {
            reset_safety_checks();
            result = true;
            return true;
        }

        // Handle DEPTH LIMIT commands
        if (command_name == "DEPTH LIMIT")
        {
            max_depth = params.get("MAX").to_double();
            result = true;
            return true;
        }

        // Handle GEOFENCE commands
        else if (command_name == "GEOFENCE")
        {

            // If there are parameters, it is a geofence set command
            if (params.has("LATS") && params.has("LONS"))
            {
                log_debug("handling geofence write command");
                geofence_lats = params.get("LATS").to_double_vector();
                geofence_lons = params.get("LONS").to_double_vector();
            }

            // If it doesn't have the lats and lons parameters, it is a geofence
            // read command
            else
            {
                log_debug("handling geofence read command");
                Parameter lats("LATS", geofence_lats);
                Parameter lons("LONS", geofence_lons);
                std::vector<Field> param_fields;
                avl::append(param_fields, lats.to_fields());
                avl::append(param_fields, lons.to_fields());
                for (Field field : param_fields)
                    avl::append(data, field.get_bytes());
                result = true;
                return true;
            }

            result = true;
            return true;

        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        violation_handler
    // Description: Called by the various monitoring callbacks when a limit
    //              violation has been detected. Logs an error message and sets
    //              the fault_detected flag to end node execution.
    // Arguments:   - message: message with limnit violation details
    //--------------------------------------------------------------------------
    void violation_handler(const std::string& message)
    {

        // Do not handle a violation if a violation was already detected
        if (!fault_detected)
        {

            log_error("safety limit violation detected (" + message + ")");
            fault_detected = true;

            // Publish an actuator control message to disable actuators
            ActuatorControlMsg actuator_control_msg;
            actuator_control_msg.enable = false;
            actuator_control_pub.publish(actuator_control_msg);

            // Call the mission mode service to switch the mission manager into
            // DISABLED mode indefinitely
            MissionModeSrv mission_mode_srv;
            mission_mode_srv.request.mode = MISSION_MODE_DISABLED;
            mission_mode_srv.request.duration = 0;
            mission_mode_client.call(mission_mode_srv);
            log_debug("mission mode service response: %s",
                mission_mode_srv.response.message.c_str());

            // Publish the safety status
            SafetyStatusMsg safety_status_msg;
            safety_status_msg.status = SAFETY_STATUS_FAULT;
            safety_status_msg.message = message;
            safety_status_pub.publish(safety_status_msg);

        }

    }

    //--------------------------------------------------------------------------
    // Name:        fault_callback
    // Description: Called when a fault is detected by the monitored subscriber.
    // Arguments:   - fault: fault event structure
    //--------------------------------------------------------------------------
    void fault_callback(const avl::SubscriberFault& fault)
    {

        switch (fault.type)
        {
            case avl::TIMEOUT_FAULT:
                violation_handler("timeout fault on topic "
                    + fault.topic);
                break;
            case avl::OUT_OF_BOUNDS_FAULT:
                violation_handler("out of bounds fault on topic "
                    + fault.topic);
                break;
            case avl::REPEATED_MESSAGE_FAULT:
                violation_handler("repeated message fault on topic "
                    + fault.topic);
                break;
        }

    }

    //--------------------------------------------------------------------------
    // Name:        reset_srv_callback
    // Description: Called when the reset_safety_node service is requested.
    //              Re-enables actuator control and resumes checking for safety
    //              violations.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool reset_srv_callback(std_srvs::Trigger::Request& request,
                            std_srvs::Trigger::Response& response)
    {
        log_debug("reset service called");
        reset_safety_checks();
        response.success = true;
        response.message = "success";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        abort_srv_callback
    // Description: Called when the abort_safety_node service is requested.
    //              Triggers a safety node abort with the specified message.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool abort_srv_callback(SafetyAbortSrv::Request& request,
                            SafetyAbortSrv::Response& response)
    {
        log_info("abort service called (%s)", request.message.c_str());
        violation_handler(request.message);
        response.success = true;
        response.message = "success";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        nav_msg_callback
    // Description: Called when a nav message is received.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void nav_msg_callback(const NavigationMsg& message)
    {
        if (!fault_detected)
            log_data("[position] %.9f, %.9f", message.lat, message.lon);
    }

    //--------------------------------------------------------------------------
    // Name:        depth_msg_callback
    // Description: Called when a depth message is received.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void depth_msg_callback(const std_msgs::Float64& message)
    {
        if (!fault_detected)
            log_data("[depth] %.2f %.2f", message.data, max_depth);
    }

    //--------------------------------------------------------------------------
    // Name:        height_msg_callback
    // Description: Called when an height message is received.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void height_msg_callback(const std_msgs::Float64& message)
    {
        if (!fault_detected)
        {
            float min_height = get_param<double>("~height/min");
            float height = message.data;
            log_data("[height] %.2f %.2f", height, min_height);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        voltage_msg_callback
    // Description: Called when a battery voltage message is received.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void voltage_msg_callback(const std_msgs::Float64& message)
    {
        if (!fault_detected)
        {
            float min_voltage = get_param<double>("~voltage/min");
            float max_voltage = get_param<double>("~voltage/max");
            float voltage = message.data;
            log_data("[voltage] %.2f %.2f %.2f",
                voltage, min_voltage, max_voltage);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        dvl_msg_callback
    // Description: Called when a DVL message is received.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void dvl_msg_callback(const avl_msgs::PathfinderDvlMsg& message)
    {
        if (!fault_detected)
        {
            log_data("[dvl] %.3f %.3f %.3f %.3f %d",
                message.vx, message.vy, message.vz, message.ve, message.valid);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        reset_safety_checks
    // Description: Resets the safety node by re-enabling actuator control and
    //              resuming safety checks.
    //--------------------------------------------------------------------------
    void reset_safety_checks()
    {

        // Ignore any safety node reset if there is not a fault detected
        if (fault_detected)
        {

            // Reset the violation flag
            fault_detected = false;

            // Reset the monitored subscribers
            nav_sub.reset();
            depth_sub.reset();
            height_sub.reset();
            voltage_sub.reset();

            // Publish an actuator control message to enable actuators
            ActuatorControlMsg actuator_control_msg;
            actuator_control_msg.enable = true;
            actuator_control_pub.publish(actuator_control_msg);

            // Call the mission mode service to switch the mission manager into
            // FSD mode indefinitely (it should be in DISABLED mode if there is
            // a fault detected)
            MissionModeSrv mission_mode_srv;
            mission_mode_srv.request.mode = MISSION_MODE_FSD;
            mission_mode_srv.request.duration = 0;
            mission_mode_client.call(mission_mode_srv);
            log_debug("mission mode service response: %s",
                mission_mode_srv.response.message.c_str());

            // Publish the safety status
            SafetyStatusMsg safety_status_msg;
            safety_status_msg.status = SAFETY_STATUS_READY;
            safety_status_msg.message = "READY";
            safety_status_pub.publish(safety_status_msg);

            log_debug("safety node reset");

        }
        else
        {
            log_debug("ignoring safety node reset, no fault detected");
        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log data headers
        add_data_header("[roll] roll min\\_roll max\\_roll");
        add_data_header("[roll] deg deg deg");
        add_data_header("[pitch] pitch min\\_pitch max\\_pitch");
        add_data_header("[pitch] deg deg deg");
        add_data_header("[position] lat lon");
        add_data_header("[position] deg deg");
        add_data_header("[depth] depth max\\_depth");
        add_data_header("[depth] m m");
        add_data_header("[height] height min\\_height");
        add_data_header("[height] m m");
        add_data_header("[voltage] voltage min\\_voltage max\\_voltage");
        add_data_header("[voltage] V V V");

        // Set the command handler's callback
        command_handler.set_callback(&SafetyNode::command_callback, this);
        packet_handler.set_callback(&SafetyNode::packet_callback, this);

        // Set up the service servers
        abort_safety_server = node_handle->advertiseService(
            "system/abort_safety_node",
            &SafetyNode::abort_srv_callback, this);
        reset_safety_server = node_handle->advertiseService(
            "system/reset_safety_node",
            &SafetyNode::reset_srv_callback, this);

        // Set up the service clients
        mission_mode_client = node_handle->serviceClient<MissionModeSrv>(
            "system/mission_mode");

        // Set up the publisher for actuator control messages as a latched
        // publisher and publish the initial message
        actuator_control_pub = node_handle->advertise<ActuatorControlMsg>(
            "device/actuator_control", 1, true);
        ActuatorControlMsg actuator_control_msg;
        actuator_control_msg.enable = true;
        actuator_control_pub.publish(actuator_control_msg);

        // Set up the safety status publisher as a latched publisher and
        // publish the initial message
        safety_status_pub = node_handle->advertise<SafetyStatusMsg>(
            "system/safety_status", 1, true);
        SafetyStatusMsg safety_status_msg;
        safety_status_msg.status = SAFETY_STATUS_READY;
        safety_status_msg.message = "READY";
        safety_status_pub.publish(safety_status_msg);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Configure the monitored subscriber for nav messages

        nav_sub.set_message_rate(get_param<double>("~nav/rate"));
        nav_sub.set_out_of_bounds_function(
            [this](NavigationMsg message)
            {

                // Check if any of the nav states are NaN
                std::vector<double> nav_states;
                nav_states.push_back(message.roll);
                nav_states.push_back(message.pitch);
                nav_states.push_back(message.yaw);
                nav_states.push_back(message.vn);
                nav_states.push_back(message.ve);
                nav_states.push_back(message.vd);
                nav_states.push_back(message.lat);
                nav_states.push_back(message.lon);
                nav_states.push_back(message.alt);
                if (avl::any_nan(nav_states))
                    return true;

                // Check if the attitudes are within the limits
                if ( message.roll  < avl::deg_to_rad(get_param<double>("~nav/min_roll"))  ||
                     message.roll  > avl::deg_to_rad(get_param<double>("~nav/max_roll"))  ||
                     message.pitch < avl::deg_to_rad(get_param<double>("~nav/min_pitch")) ||
                     message.pitch > avl::deg_to_rad(get_param<double>("~nav/max_pitch")) )
                    return true;

                // Check if the velocity is within the limits
                Eigen::Vector3d v_n = {message.vn, message.ve, message.vd};
                if (v_n.norm() > get_param<double>("~nav/max_vel"))
                    return true;

                // If there are geofence points, check if the position is in
                // the polygon formed by the geofence points
                if (geofence_lats.size() == geofence_lons.size() &&
                    geofence_lats.size() >= 3 &&
                    !avl::point_in_poly(avl::rad_to_deg(message.lat),   avl::rad_to_deg(message.lon),
                                        geofence_lats, geofence_lons))
                    return true;

                return false;

            });
        nav_sub.set_repeated_message_function(
            [this](NavigationMsg previous_message,
                   NavigationMsg message)
            {
                return  message.lat == previous_message.lat &&
                        message.lon == previous_message.lon;
            });
        nav_sub.enable_message_rate_check(get_param<bool>("~nav/enable_rate_check"));
        nav_sub.enable_out_of_bounds_check(get_param<bool>("~nav/enable_out_of_bounds_check"));
        nav_sub.enable_repitition_check(get_param<bool>("~nav/enable_reptition_check"));
        nav_sub.subscribe("nav/inertial_nav", 1,
            &SafetyNode::nav_msg_callback,
            &SafetyNode::fault_callback, this);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Configure the monitored subscriber for depth messages

        max_depth = get_param<double>("~depth/max");
        depth_sub.set_message_rate(get_param<double>("~depth/rate"));
        depth_sub.set_out_of_bounds_function(
            [this](std_msgs::Float64 message)
            {
                return message.data > max_depth;
            });
        depth_sub.set_repeated_message_function(
            [this](std_msgs::Float64 previous_message, std_msgs::Float64 message)
            {
                return  message.data == previous_message.data;
            });
        depth_sub.enable_message_rate_check(get_param<bool>("~depth/enable_rate_check"));
        depth_sub.enable_out_of_bounds_check(get_param<bool>("~depth/enable_out_of_bounds_check"));
        depth_sub.enable_repitition_check(get_param<bool>("~depth/enable_reptition_check"));
        depth_sub.subscribe("device/depth", 1,
            &SafetyNode::depth_msg_callback,
            &SafetyNode::fault_callback, this);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Configure the monitored subscriber for height messages

        height_sub.set_message_rate(get_param<double>("~height/rate"));
        height_sub.set_out_of_bounds_function(
            [this](std_msgs::Float64 message)
            {
                return  message.data < get_param<double>("~height/min");
            });
        height_sub.set_repeated_message_function(
            [this](std_msgs::Float64 previous_message, std_msgs::Float64 message)
            {
                return  message.data == previous_message.data;
            });
        height_sub.enable_message_rate_check(get_param<bool>("~height/enable_rate_check"));
        height_sub.enable_out_of_bounds_check(get_param<bool>("~height/enable_out_of_bounds_check"));
        height_sub.enable_repitition_check(get_param<bool>("~height/enable_reptition_check"));
        height_sub.subscribe("device/height", 1,
            &SafetyNode::height_msg_callback,
            &SafetyNode::fault_callback, this);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Configure the monitored subscriber for voltage messages

        voltage_sub.set_message_rate(get_param<double>("~voltage/rate"));
        voltage_sub.set_out_of_bounds_function(
            [this](std_msgs::Float64 message)
            {
                return  message.data < get_param<double>("~voltage/min") ||
                        message.data > get_param<double>("~voltage/max");
            });
        voltage_sub.set_repeated_message_function(
            [this](std_msgs::Float64 previous_message, std_msgs::Float64 message)
            {
                return  message.data == previous_message.data;
            });
        voltage_sub.enable_message_rate_check(get_param<bool>("~voltage/enable_rate_check"));
        voltage_sub.enable_out_of_bounds_check(get_param<bool>("~voltage/enable_out_of_bounds_check"));
        voltage_sub.enable_repitition_check(get_param<bool>("~voltage/enable_reptition_check"));
        voltage_sub.subscribe("device/voltage", 1,
            &SafetyNode::voltage_msg_callback,
            &SafetyNode::fault_callback, this);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Configure the monitored subscriber for DVL messages

        dvl_sub.set_message_rate(get_param<double>("~dvl/rate"));
        dvl_sub.enable_message_rate_check(get_param<bool>("~dvl/enable_rate_check"));
        dvl_sub.enable_out_of_bounds_check(get_param<bool>("~dvl/enable_out_of_bounds_check"));
        dvl_sub.enable_repitition_check(get_param<bool>("~dvl/enable_reptition_check"));
        dvl_sub.subscribe("device/dvl", 1,
            &SafetyNode::dvl_msg_callback,
            &SafetyNode::fault_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
  SafetyNode node(argc, argv);
  node.start();
  return 0;
}
