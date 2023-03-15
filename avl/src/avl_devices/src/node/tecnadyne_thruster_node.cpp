//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to interface with the Tecnadyne Thruster actuator.
//
// Servers:     /dive/thruster_enable (avl_msgs/MotorSrv)
//
// Clients:     None
//
// Publishers:  device/rpm (std_msgs/Float64)
//
// Subscribers: device/actuator_control (avl_msgs/ActuatorControlMsg)
//              setpoint/rpm (avl_msgs/Float64SetpointMsg)
//==============================================================================

// Node base class
#include <avl_devices/device_node.h>

// TCP port class
#include <avl_asio/tcp_socket.h>

// Utility functions
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>
#include <avl_core/util/misc.h>
#include <avl_core/util/math.h>
#include <avl_core/monitored_subscriber.h>

// ROS message includes
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/ActuatorControlMsg.h>
#include <avl_msgs/MotorSrv.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// Thruster messaging protocol
#include <avl_devices/protocol/tecnadyne_thruster.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TecnadyneThrusterNode : public DeviceNode
{

public:
    //--------------------------------------------------------------------------
    // Name:        TecnadyneThrusterNode constructor
    //--------------------------------------------------------------------------
    TecnadyneThrusterNode(int argc, char **argv) : DeviceNode(argc, argv) {}

private:
    // TCP device instance
    TcpSocket tcp;

    // Subscribers
    MonitoredSubscriber<Float64SetpointMsg> rpm_cmd_sub;
    ros::Subscriber actuator_control_sub;

    // Publisher for measured rpm values
    ros::Publisher rpm_pub;

    // Services
    ros::ServiceServer motor_state_srv;

    // Timers
    ros::Duration feedback_duration;
    ros::Timer feedback_timer;
    ros::Timer tcp_timeout;
    ros::Timer state_rpm_timeout;

    // Thruster struct for current motor status
    Thruster thruster{};

    // Flags indicating whether motor control is enabled.
    // Safety is meant to prevent commands from reaching motor. State is meant
    // to prevent motor from accepting/reacting to commands
    bool motor_safety_enable = false;
    bool motor_state_enable = false;

    // Flag indicating a packet was terminated early by match function
    bool early_term = false;

    // Variable to store packet bytes as they are received
    std::vector<uint8_t> packet;

    //Variables for basic class
    double thruster_voltage_device;
    double thruster_current_device;
    bool thruster_fault_device = false;
    double thruster_RPM_device;
    bool response_wait{false};

private:
    //--------------------------------------------------------------------------
    // Name:        motor_state_srv_callback
    // Description: Called when the motor_state service is requested. Sets a
    //              requested rpm and either enables or disables the motor. If
    //              the motor is disabled, the rpm request is ignored and 0 rpm
    //              is commanded.
    // Arguments:   - request.state_enable: Request received on the service.
    //              - request.rpm: Response to the service request.
    //              - response.success: response indicating motor ack'ed the
    //                request.
    // Returns:     True if service completed, false if it failed to complete.
    //--------------------------------------------------------------------------
    bool motor_state_srv_callback(MotorSrv::Request &request,
                                  MotorSrv::Response &response)
    {
        log_info("Motor state %s request received", request.state_enable ? "enable" : "disable");

        motor_state_enable = request.state_enable;
        thruster.rpm.cmd = avl::clamp(request.rpm, (-thruster.settings.max_rpm),
                                      thruster.settings.max_rpm);
        bool oneshot = true;
        state_rpm_timeout = node_handle->createTimer(ros::Duration(
                                                         request.timeout),
                                                     &TecnadyneThrusterNode::disable, this, oneshot);
        response.success = true; //TODO update these to check for ACK
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        feedback_timer_callback
    // Description: Called when feedback timer is reached. Send the request
    //              packet.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void feedback_timer_callback(const ros::TimerEvent &event)
    {
        // Check if message has been received since last query, to prevent
        // serial collisions
        static size_t wait_count{0};
        if (response_wait)
        {
            // After consecutive skips, assume response was missed
            if (wait_count >= 5)
            {
                response_wait = false;
                wait_count = 0;
            }
            else
                wait_count++;
        }
        else
        {
            static size_t timer_count{0}; //count to iterate throgh all channels
            if (thruster_queries[timer_count] == 0x21)
            {
                //log_debug("RPM commanded: %.2f", thruster.rpm.cmd);
                tcp.write(TEC_WRAP(RPM_SETPOINT(&thruster),
                                   thruster.settings.address));
            }
            else if (thruster_queries[timer_count] == 0x41)
            {
                //log_debug("ENABLE: %u", motor_state_enable);
                if (motor_state_enable)
                {
                    //log_debug("RPM speed loop enable to : 0x%x", thruster.settings.address);
                    tcp.write(TEC_WRAP(speed_loop_enable,
                                       thruster.settings.address));
                }
                else
                {
                    //log_debug("RPM speed loop disable to : 0x%x", thruster.settings.address);
                    tcp.write(TEC_WRAP(speed_loop_disable,
                                       thruster.settings.address));
                }
            }
            else
            {
                //log_debug("Timer count %u", timer_count);
                tcp.write(TEC_CMD(thruster_queries[timer_count],
                                  thruster.settings.address));
            }
            timer_count++;
            if (timer_count >= thruster_queries.size())
                timer_count = 0;

            response_wait = true;
        }
    }

    //--------------------------------------------------------------------------
    // Name:        read_handler
    // Description: Handler for received motor messages
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void read_handler(std::vector<uint8_t> data)
    {
        response_wait = false;
        avl::append(packet, data);
        //remove header and footer
        std::vector<uint8_t> message{TEC_UNWRAP(packet, early_term)};

        //if TEC_UNWRAP exited early, need to find next DLE/ETX pair
        if (early_term)
        {
            early_term = false;
            tcp.set_match(Match({DLE, ETX}, &TecnadyneThrusterNode::read_handler,
                                this));
            return;
        }

        //retrieve motor's address, then remove from message vector to align
        //with command Reference iterator positions for payloads
        message.erase(message.begin());

        try
        {
            switch (message[0])
            {
            case 0x32:
                TEC_ANALOG_STATUS(message, &thruster);

                log_data("[THRUSTER] %.0f %.3f %.3f %.1f %.1f %.1f %u %u",
                    thruster.p, thruster.v, thruster.i,
                    thruster.temp.controller, thruster.temp.heatsink,
                    thruster.temp.winding, thruster.reset, thruster.fault);
                if (thruster.fault)
                    log_warning("Thruster motor drive fault state");
                thruster_voltage_device = thruster.v;
                thruster_current_device = thruster.i;

                // Reset tcp_timeout
                tcp_timeout.stop();
                tcp_timeout.start();
                break;

            case 0x35:
                TEC_FLASH_DATA(message, &thruster);
                log_data("[THRUSTER_SETTINGS] 0x%x %d %d 0x%x 0x%x %d",
                         thruster.settings.address, thruster.settings.tx_delay,
                         thruster.settings.watchdog, thruster.settings.slew,
                         thruster.settings.ACK, thruster.settings.freq);
                break;

            case 0x36:
                break;

            case 0x37:
            {
                TEC_EXT_STATUS(message, &thruster);
                log_data("[RPM] %.0f %.0f", thruster.rpm.measured,
                         thruster.rpm.cmd);

                std_msgs::Float64 rpm_msg;
                rpm_msg.data = thruster.rpm.measured;

                thruster_RPM_device = thruster.rpm.measured;

                rpm_pub.publish(rpm_msg);
                break;
            }

            case 0x70:
                TEC_FW(message, &thruster);
                log_data("[FW] " + thruster.settings.fw_ver + " " +
                         thruster.settings.fw_date);
                break;

            default:
                log_warning("Unrecognized message type: " +
                            std::to_string(message[0]));
                response_wait = true;
                break;
            }
        }
        catch (const std::exception &ex)
        {
            log_warning("Message parse error");
            response_wait = true;
        }

        packet.clear();
        tcp.set_match(Match(&DLE, &TecnadyneThrusterNode::next_byte_handler,
                            this));
    }

    //--------------------------------------------------------------------------
    // Name:        fault_callback
    // Description: Called when a fault is detected by a monitored subscriber.
    // Arguments:   - fault: Fault event structure.
    //--------------------------------------------------------------------------
    void fault_callback(const avl::SubscriberFault &fault)
    {
        std::string fault_type;
        if (fault.type == 0)
            fault_type = "TIMEOUT_FAULT";
        else if (fault.type == 1)
            fault_type = "OUT_OF_BOUNDS_FAULT";
        else
            fault_type = "REPEATED_MESSAGE_FAULT";
        log_warning("Fault detected: %s", fault_type.c_str());
        disable();
        rpm_cmd_sub.reset();
    }

    //--------------------------------------------------------------------------
    // Name:        rpm_setpoint_callback
    // Description: Called when an rpm message is received on the setpoint topic
    //              Sets motor rpm and waits for next feedback cycle to push
    //              update to the motor
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void rpm_setpoint_callback(const Float64SetpointMsg &message)
    {
        // Two enable booleans: safety and controller
        if (motor_safety_enable && message.enable)
        {
            if (!std::isnan(message.data))
                thruster.rpm.cmd = avl::clamp(message.data,
                                              (-thruster.settings.max_rpm), thruster.settings.max_rpm);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        actuator_control_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void actuator_control_msg_callback(const ActuatorControlMsg
                                           message)
    {
        log_info("Actuator %s request received", message.enable ? "enable" : "disable");
        motor_safety_enable = message.enable;
    }

    //--------------------------------------------------------------------------
    // Name:        tcp_interruption
    // Description: Handles writing a tcp message outside the feedback timer
    //              scope. Will pause feedback for ~2x feedback_duration
    // Arguments:   - message: formatted message ready for transmission
    //--------------------------------------------------------------------------
    void tcp_interruption(std::vector<uint8_t> message)
    {
        //stop the feedback timer to prevent serial line collisions
        feedback_timer.stop();
        feedback_duration.sleep();
        //process any data received on the serial line
        for (int i = 0; i < 100; i++)
        {
            tcp.spin_once();
            ros::spinOnce();
        }

        tcp.write(message);

        feedback_duration.sleep();
        //process any data received on the serial line
        for (int i = 0; i < 100; i++)
        {
            tcp.spin_once();
            ros::spinOnce();
        }
        feedback_timer.start();
    }

    //--------------------------------------------------------------------------
    // Name:        next_byte_handler
    // Description: Handles receipt of a header byte, indicative of a potential
    //              new packet.
    //--------------------------------------------------------------------------
    void next_byte_handler(std::vector<uint8_t> data)
    {
        avl::append(packet, data);

        if (packet.back() == DLE)
            tcp.set_match(Match(1, &TecnadyneThrusterNode::next_byte_handler,
                                this));
        //TODO add function to call to get one more byte (checksum) and do checksum
        //validation
        else if (packet.back() == STX)
            tcp.set_match(Match({DLE, ETX}, &TecnadyneThrusterNode::read_handler,
                                this));
        else if (packet.back() == ACK)
        {
            // Message received, so OK to send another command
            response_wait = false;
            log_data("[ACK] 1");
            packet.clear();
        }
        else if (packet.back() == NAK)
        {
            response_wait = false;
            log_data("[NAK] 0");
            packet.clear();
        }
        else
        {
            log_warning("Unrecognized message format");
            for (size_t i = 0; i < packet.size(); i++)
                log_debug("DB%d : 0x%x", i, packet[i]);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        disable
    // Description: Called when all setpoints in the control node are disabled.
    //              Logic on how to disable the controller, such as sending
    //              disabling outputs to any cascaded controllers, should be
    //              implemented here.
    //--------------------------------------------------------------------------
    void disable(const ros::TimerEvent &event = ros::TimerEvent())
    {
        log_info("Thruster 0 RPM Commanded");
        thruster.rpm.cmd = 0.0;
    }

    //--------------------------------------------------------------------------
    // Name:        tcp_connect
    // Description: Connect to the tcp device. Used for startup or loss of comms
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void tcp_connect(const ros::TimerEvent &event)
    {
        try
        {
            std::string tcp_address = get_param<std::string>("~tcp/address");
            int tcp_port = get_param<int>("~tcp/port");
            log_info("Attempting to connect to " + tcp_address + ":" + std::to_string(tcp_port));
            // Open the TCP port
            tcp.connect(tcp_address, tcp_port);

            // Set the match condition to a newline for data packets
            tcp.set_match(Match(&DLE, &TecnadyneThrusterNode::next_byte_handler,
                                this));
            feedback_timer.start();
            log_info("TCP Connection successful");
        }
        catch (const std::exception &ex)
        {
            log_warning("TCP Connection failed");
            feedback_timer.stop();
            actuator_control_sub.shutdown();
            rpm_cmd_sub.shutdown();
            rpm_pub.shutdown();
            return;
            //TODO probably try to close the port (if it's already open)
        }
        // Set up the measured rpm publisher
        rpm_pub = node_handle->advertise<std_msgs::Float64>("device/rpm", 1);

        // Set up the subscribers
        actuator_control_sub = node_handle->subscribe("device/actuator_control",
                                                      1, &TecnadyneThrusterNode::actuator_control_msg_callback, this);
        rpm_cmd_sub.set_message_rate(get_param<double>("~motor_input_rate"));
        rpm_cmd_sub.enable_message_rate_check(true);
        rpm_cmd_sub.enable_out_of_bounds_check(false);
        rpm_cmd_sub.enable_repitition_check(false);
        rpm_cmd_sub.subscribe("setpoint/rpm", 8,
                              &TecnadyneThrusterNode::rpm_setpoint_callback,
                              &TecnadyneThrusterNode::fault_callback, this);

        //Establish comms
        tcp_interruption(TEC_CMD(0x35, BROADCAST));                 //Request Flash Data
        tcp_interruption(TEC_CMD(0x70, thruster.settings.address)); //FW

    }

    //--------------------------------------------------------------------------
    // Name:        get_device_parameters
    // Description: Called when a DEVICE packet is requested from the node or
    //              when a DEVICE packet is needed to be published to the FSD
    //              or BSD interface. This function should be implemented by the
    //              device child class to create and return a parameter list.
    // Returns:     Parameter list containing parameters to be added to the
    //              device packet that will be transmitted.
    //--------------------------------------------------------------------------
    avl::ParameterList get_device_parameters()
    {

        avl::ParameterList params;
        params.add(Parameter("VOLT", thruster_voltage_device));
        params.add(Parameter("CURR", thruster_current_device));
        params.add(Parameter("FAULT",thruster_fault_device));
        params.add(Parameter("RPM",  thruster_RPM_device));

        return params;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        log_info("initializing Tecnadyne thruster...");

        log_data("[THRUSTER] power voltage current controller_temp heatsink_temp"
                 " motor_winding_temp reset fault");
        log_data("[THRUSTER] W V A degC degC degC bool bool");

        log_data("[THRUSTER_SETTING] address tx_delay watchdog slew ACk freq");
        log_data("[THRUSTER_SETTING] NA ms s ms boolean kHz");

        log_data("[RPM] measured cmd");
        log_data("[RPM] rpm rpm");

        log_data("[SETTING] fw_ver fw_date");
        log_data("[SETTING] string string");

        log_data("[ACK] Acknowledge");
        log_data("[ACK] boolean");

        // Configure feedback request timer
        feedback_duration = ros::Duration(get_param<float>("~feedback_duration") / thruster_queries.size());
        feedback_timer = node_handle->createTimer(feedback_duration,
                                                  &TecnadyneThrusterNode::feedback_timer_callback, this);
        // Don't start timer until tcp connection established
        feedback_timer.stop();

        // Set up services
        motor_state_srv = node_handle->advertiseService("dive/thruster_enable",
                                                        &TecnadyneThrusterNode::motor_state_srv_callback, this);

        //Set the pole pair count
        thruster.settings.pole_pair_count = get_param<int>("~motor/pole_pair_count");

        //Set the max motor speed
        thruster.settings.max_rpm = get_param<float>("~motor/max_rpm");

        // Initial attempt to connect to TCP port
        const ros::TimerEvent event_empty;
        tcp_connect(event_empty);

        // Timer to attempt reconnect if TCP connection is lost
        ros::Duration tcp_timeout_duration = ros::Duration(
            get_param<int>("~tcp/reconnect"));
        tcp_timeout = node_handle->createTimer(tcp_timeout_duration,
                                               &TecnadyneThrusterNode::tcp_connect, this);

        // Set device name and default DEVICE packet output rates
        set_device_name("THRUSTER");
        set_device_packet_output_rate(1.0, INTERFACE_FSD);
        set_device_packet_output_rate(1.0, INTERFACE_BSD);
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function when the node is started. Can
    //              be overriden by a derived node class.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        disable();
        motor_safety_enable = false;
        motor_state_enable = false;
    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::Rate spin_rate(1000);
        while (ros::ok())
        {
            tcp.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
        }
    }
};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    TecnadyneThrusterNode node(argc, argv);
    node.start();
    return 0;
}
