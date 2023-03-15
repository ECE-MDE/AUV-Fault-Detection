//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node that assembles status and sensord ata messages into
//              HEARTBEAT packet bytes and periodically publishes them.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  system/heartbeat (avl_comms/HeartbeatMsg)
//
// Subscribers: system/safety_status_sub (avl_msgs/SafetyStatusMsg)
//              nav/inertial_nav (avl_navigation/NavigationMsg)
//              device/depth (std_msgs/Float64)
//              device/height (std_msgs/Float64)
//              device/rpm (std_msgs/Float64)
//              device/voltage (std_msgs/Float64)
//              device/whoi/status (avl_devices/WhoiStatusMsg)
//              device/iridium/signal (avl_devices/IridiumSignalMsg)
//              system/mission_status (avl_msgs/MissionStatusMsg)
//              device/gps (avl_devices/GpsMsg)
//              device/imu (avl_msgs/ImuMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

// Monitored subscriber class
#include <avl_core/monitored_subscriber.h>

// Util functions
#include <avl_core/util/misc.h>
#include <avl_core/util/math.h>

// Comms functionality
#include <avl_comms/command_handler.h>
using namespace avl;

// ROS messages
#include <avl_msgs/HeartbeatMsg.h>
#include <avl_msgs/SafetyStatusMsg.h>
#include <avl_msgs/NavigationMsg.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <avl_msgs/WhoiStatusMsg.h>
#include <avl_msgs/IridiumSignalMsg.h>
#include <avl_msgs/MissionStatusMsg.h>
#include <avl_msgs/GpsMsg.h>
#include <avl_msgs/ImuMsg.h>
using namespace avl_msgs;

//==============================================================================
//                            HEARTBEAT RATE ENUM
//==============================================================================

enum HeartbeatRate
{
    RATE_DISABLE,
    RATE_001HZ,
    RATE_01HZ,
    RATE_1HZ,
    RATE_10HZ
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class HeartbeatNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        HeartbeatNode constructor
    //--------------------------------------------------------------------------
    HeartbeatNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Timer and publisher for publishing periodic heartbeat messages
    ros::Timer fsd_heartbeat_timer;
    ros::Timer bsd_heartbeat_timer;
    ros::Publisher fsd_heartbeat_pub;
    ros::Publisher bsd_heartbeat_pub;

    // Subscribers for status data
    ros::Subscriber safety_status_sub;
    ros::Subscriber mission_status_sub;
    ros::Subscriber whoi_status_sub;
    ros::Subscriber iridium_signal_sub;
    MonitoredSubscriber<NavigationMsg> nav_sub;
    ros::Subscriber nav_status_sub;
    MonitoredSubscriber<GpsMsg> gps_sub;
    MonitoredSubscriber<std_msgs::Float64> depth_sub;
    MonitoredSubscriber<std_msgs::Float64> height_sub;
    MonitoredSubscriber<std_msgs::Float64> voltage_sub;
    MonitoredSubscriber<std_msgs::Float64> rpm_sub;
    MonitoredSubscriber<ImuMsg> imu_sub;

    // Heartbeat struct to store data from subscribed messages
    Heartbeat heartbeat;

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

private:

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

        // Handle HEARTBEAT commands
        if (command_name == "HEARTBEAT")
        {

            // Get the rate enum from the command
            HeartbeatRate rate =
                params.get("RATE").to_enum<HeartbeatRate>();

            // Choose the timer to modify based on the interface
            ros::Timer* timer = (interface == INTERFACE_FSD) ?
                &fsd_heartbeat_timer : &bsd_heartbeat_timer;

            // If the rate is set to disable, stop the timer. Otherwise, set
            // the timer period based on the enum value and restart the timer
            if (rate == RATE_DISABLE)
            {
                timer->stop();
                log_info("disabled %s heartbeat",
                    avl::interface_to_string(interface).c_str());
            }
            else
            {
                double rate_hz = 1.0;
                if      (rate == RATE_001HZ) rate_hz = 0.01;
                else if (rate == RATE_01HZ)  rate_hz = 0.1;
                else if (rate == RATE_1HZ)   rate_hz = 1.0;
                else if (rate == RATE_10HZ)  rate_hz = 10.0;
                timer->setPeriod(ros::Duration(1.0/rate_hz), false);
                timer->start();
                log_info("set %s heartbeat rate to %.2f Hz",
                    avl::interface_to_string(interface).c_str(), rate_hz);
            }

            result = true;
            return true;

        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        fault_callback
    // Description: Called when a fault is detected by a monitored subscriber.
    // Arguments:   - fault: Fault event structure.
    //--------------------------------------------------------------------------
    void fault_callback(const avl::SubscriberFault& fault)
    {

        // Reset the status value to NAN depending on which message timed out,
        // and reset that subscriber so that it can time out again
        if (fault.topic == "device/gps")
        {
            heartbeat.gps_sats = 0;
            heartbeat.gps_lat = NAN;
            heartbeat.gps_lon = NAN;
            heartbeat.gps_alt = NAN;
            gps_sub.reset();
        }
        else if (fault.topic == "nav/inertial_nav")
        {
            heartbeat.roll = NAN;
            heartbeat.pitch = NAN;
            heartbeat.yaw = NAN;
            heartbeat.vn = NAN;
            heartbeat.ve = NAN;
            heartbeat.vd = NAN;
            heartbeat.nav_lat = NAN;
            heartbeat.nav_lon = NAN;
            heartbeat.nav_alt = NAN;
            nav_sub.reset();
        }
        else if (fault.topic == "device/depth")
        {
            heartbeat.depth = NAN;
            depth_sub.reset();
        }
        else if (fault.topic == "device/height")
        {
            heartbeat.height = NAN;
            height_sub.reset();
        }
        else if (fault.topic == "device/voltage")
        {
            heartbeat.voltage = NAN;
            voltage_sub.reset();
        }
        else if (fault.topic == "device/rpm")
        {
            heartbeat.rpm = NAN;
            rpm_sub.reset();
        }
        else if (fault.topic == "device/imu")
        {
            heartbeat.wx = NAN;
            heartbeat.wy = NAN;
            heartbeat.wz = NAN;
            heartbeat.ax = NAN;
            heartbeat.ay = NAN;
            heartbeat.az = NAN;
            imu_sub.reset();
        }

    }

    //--------------------------------------------------------------------------
    // Name:        safety_status_msg_callback
    // Description: Called when a safety status message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void safety_status_msg_callback(const SafetyStatusMsg& message)
    {
        heartbeat.status = static_cast<SafetyStatus>(message.status);
    }

    //--------------------------------------------------------------------------
    // Name:        mission_status_msg_callback
    // Description: Called when a mission status message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void mission_status_msg_callback(const MissionStatusMsg& message)
    {
        heartbeat.mission_mode = static_cast<MissionMode>(message.mode);
        heartbeat.fsd_mission_state =  static_cast<State>(message.fsd_mission_state);
        heartbeat.fsd_current_action = message.fsd_current_action;
        heartbeat.fsd_total_actions =  message.fsd_total_actions;
        heartbeat.fsd_action_percent = message.fsd_action_percent;
        heartbeat.bsd_mission_state =  static_cast<State>(message.bsd_mission_state);
        heartbeat.bsd_current_action = message.bsd_current_action;
        heartbeat.bsd_total_actions =  message.bsd_total_actions;
        heartbeat.bsd_action_percent = message.bsd_action_percent;
    }

    //--------------------------------------------------------------------------
    // Name:        whoi_status_msg_callback
    // Description: Called when a WHOI status message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void whoi_status_msg_callback(const WhoiStatusMsg& message)
    {
        if (message.synced_to_gps)
            heartbeat.umodem_sync = SYNC_SYNCED;
        else
            heartbeat.umodem_sync = SYNC_UNSYNCED;
    }

    //--------------------------------------------------------------------------
    // Name:        iridium_signal_msg_callback
    // Description: Called when an Iridium signal message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void iridium_signal_msg_callback(const IridiumSignalMsg& message)
    {
        heartbeat.iridium_str = message.signal_strength;
    }

    //--------------------------------------------------------------------------
    // Name:        gps_msg_callback
    // Description: Called when a GPS message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void gps_msg_callback(const GpsMsg& message)
    {
        heartbeat.gps_sats = message.num_sats;
        heartbeat.gps_lat = avl::rad_to_deg(message.lat);
        heartbeat.gps_lon = avl::rad_to_deg(message.lon);
        heartbeat.gps_alt = message.alt;
    }

    //--------------------------------------------------------------------------
    // Name:        nav_status_msg_callback
    // Description: Called when a nav status message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void nav_status_msg_callback(const std_msgs::Bool& message)
    {
        heartbeat.nav_initialized = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        nav_msg_callback
    // Description: Called when a nav message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void nav_msg_callback(const NavigationMsg& message)
    {
        heartbeat.roll =    avl::rad_to_deg(message.roll);
        heartbeat.pitch =   avl::rad_to_deg(message.pitch);
        heartbeat.yaw =     avl::rad_to_deg(message.yaw);
        heartbeat.vn =      message.vn;
        heartbeat.ve =      message.ve;
        heartbeat.vd =      message.vd;
        heartbeat.nav_lat = avl::rad_to_deg(message.lat);
        heartbeat.nav_lon = avl::rad_to_deg(message.lon);
        heartbeat.nav_alt = message.alt;
        heartbeat.nav_yaw_std = avl::rad_to_deg(message.yaw_std);
        heartbeat.nav_avg_pos_err = message.avg_pos_err;
    }

    //--------------------------------------------------------------------------
    // Name:        depth_msg_callback
    // Description: Called when a depth message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void depth_msg_callback(const std_msgs::Float64& message)
    {
        heartbeat.depth = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        height_msg_callback
    // Description: Called when an height message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void height_msg_callback(const std_msgs::Float64& message)
    {
        heartbeat.height = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        voltage_msg_callback
    // Description: Called when a battery voltage message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void voltage_msg_callback(const std_msgs::Float64& message)
    {
        heartbeat.voltage = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        rpm_msg_callback
    // Description: Called when an RPM message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void rpm_msg_callback(const std_msgs::Float64& message)
    {
        heartbeat.rpm = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        imu_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void imu_msg_callback(const ImuMsg& msg)
    {
        heartbeat.wx = avl::rad_to_deg(msg.angular_velocity.x);
        heartbeat.wy = avl::rad_to_deg(msg.angular_velocity.y);
        heartbeat.wz = avl::rad_to_deg(msg.angular_velocity.z);
        heartbeat.ax = msg.linear_acceleration.x;
        heartbeat.ay = msg.linear_acceleration.y;
        heartbeat.az = msg.linear_acceleration.z;
    }

    //--------------------------------------------------------------------------
    // Name:        heartbeat_timer_callback
    // Description: Called when the timer triggers.
    // Arguments:   - event: ROS timer event.
    //--------------------------------------------------------------------------
    void heartbeat_timer_callback(const ros::TimerEvent& event, bool bsd)
    {

        PacketHeader header;
        header.timestamp = avl::get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = avl::get_vehicle_id();
        header.destination_id = 0;

        HeartbeatPacket heartbeat_packet(header, heartbeat);

        HeartbeatMsg msg;
        msg.heartbeat_packet = heartbeat_packet.get_bytes();
        msg.micro_heartbeat_packet =
            MicroHeartbeatPacket::from_heartbeat_packet(heartbeat_packet);

        // Publish the message based on the interface
        if (bsd)
            bsd_heartbeat_pub.publish(msg);
        else
            fsd_heartbeat_pub.publish(msg);

    }

    //--------------------------------------------------------------------------
    // Name:        init_subscribers
    // Description: Initializes all heartbeat data subscribers.
    //--------------------------------------------------------------------------
    void init_subscribers()
    {

        safety_status_sub = node_handle->subscribe(
            "system/safety_status", 1,
            &HeartbeatNode::safety_status_msg_callback, this);

        mission_status_sub = node_handle->subscribe(
            "system/mission_status", 1,
            &HeartbeatNode::mission_status_msg_callback, this);

        whoi_status_sub = node_handle->subscribe(
            "device/whoi/status", 1,
            &HeartbeatNode::whoi_status_msg_callback, this);

        nav_status_sub = node_handle->subscribe(
            "nav/status", 1,
            &HeartbeatNode::nav_status_msg_callback, this);

        iridium_signal_sub = node_handle->subscribe(
            "device/iridium/signal", 1,
            &HeartbeatNode::iridium_signal_msg_callback, this);

        // Get the minimum message input rate from the config file
        double input_rate = get_param<double>("~status_input_rate");

        gps_sub.set_message_rate(input_rate);
        gps_sub.enable_message_rate_check(true);
        gps_sub.subscribe("device/gps", 1,
            &HeartbeatNode::gps_msg_callback,
            &HeartbeatNode::fault_callback, this);

        nav_sub.set_message_rate(input_rate);
        nav_sub.enable_message_rate_check(true);
        nav_sub.subscribe("nav/inertial_nav", 1,
            &HeartbeatNode::nav_msg_callback,
            &HeartbeatNode::fault_callback, this);

        depth_sub.set_message_rate(input_rate);
        depth_sub.enable_message_rate_check(true);
        depth_sub.subscribe("device/depth", 1,
            &HeartbeatNode::depth_msg_callback,
            &HeartbeatNode::fault_callback, this);

        height_sub.set_message_rate(input_rate);
        height_sub.enable_message_rate_check(true);
        height_sub.subscribe("device/height", 1,
            &HeartbeatNode::height_msg_callback,
            &HeartbeatNode::fault_callback, this);

        voltage_sub.set_message_rate(input_rate);
        voltage_sub.enable_message_rate_check(true);
        voltage_sub.subscribe("device/voltage", 1,
            &HeartbeatNode::voltage_msg_callback,
            &HeartbeatNode::fault_callback, this);

        rpm_sub.set_message_rate(input_rate);
        rpm_sub.enable_message_rate_check(true);
        rpm_sub.subscribe("device/rpm", 1,
            &HeartbeatNode::rpm_msg_callback,
            &HeartbeatNode::fault_callback, this);

        imu_sub.set_message_rate(input_rate);
        imu_sub.enable_message_rate_check(true);
        imu_sub.subscribe("device/imu", 1,
            &HeartbeatNode::imu_msg_callback,
            &HeartbeatNode::fault_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Set up the publishers for heartbeats
        fsd_heartbeat_pub = node_handle->advertise<HeartbeatMsg>(
            "comms/fsd_heartbeat", 1);
        bsd_heartbeat_pub = node_handle->advertise<HeartbeatMsg>(
            "comms/bsd_heartbeat", 1);

        // Bind the timer callbackd to specify FSD or BSD interface
        auto fsd_timer_cb = std::bind(&HeartbeatNode::heartbeat_timer_callback,
            this, std::placeholders::_1, false);
        auto bsd_timer_cb = std::bind(&HeartbeatNode::heartbeat_timer_callback,
            this, std::placeholders::_1, true);

        // Set up the heartbeat timers with the default ratefsd_timer_cb
        double default_heartbeat_rate = get_param<double>(
            "~default_heartbeat_rate");
        fsd_heartbeat_timer = node_handle->createTimer(
            ros::Duration(1.0/default_heartbeat_rate), fsd_timer_cb);
        bsd_heartbeat_timer = node_handle->createTimer(
            ros::Duration(1.0/default_heartbeat_rate), bsd_timer_cb);

        // Initialize subscribers
        init_subscribers();

        // Configure the command handler callback
        command_handler.set_callback(&HeartbeatNode::command_callback, this);

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
    HeartbeatNode node(argc, argv);
    node.start();
    return 0;
}
