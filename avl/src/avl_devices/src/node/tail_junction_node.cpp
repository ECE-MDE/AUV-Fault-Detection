//==============================================================================
// Autonomous Vehicle Library
//
// Description: A ROS node to interface with the NAVO tail junction board.
//
// Servers:     device/reset_fins (std_srvs/Trigger)
//              device/enable_strobe (std_srvs/Trigger)
//              device/disable_strobe (std_srvs/Trigger)
//              device/set_sail_mode (std_srvs/Trigger)
//              device/zero_pressure (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  device/pressure (std_msgs/Float64)
//              device/rpm (std_msgs/Float64)
//              device/fin_measurement_request (std_msgs/Empty)
//              device/fin_calibration_status (std_msgs/UInt16)
//
// Subscribers: device/actuator_control (avl_msgs/ActuatorControlMsg)
//              device/fins (avl_msgs/FinsMsg)
//              device/motor (std_msgs/Float64)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Serial port class
#include <avl_asio/serial_port.h>

// AAF commands
#include <avl_devices/protocol/aaf/aaf.h>

// Command handler class
#include <avl_comms/command_handler.h>
using namespace avl;

// Util functions
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>
#include <avl_core/util/math.h>

// ROS message includes
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <avl_msgs/FinCalibrationMsg.h>
#include <avl_msgs/FinsMsg.h>
#include <std_srvs/Trigger.h>
#include <avl_msgs/SailModeSrv.h>
#include <avl_msgs/ActuatorControlMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TailJunctionNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        TailJunctionNode constructor
    //--------------------------------------------------------------------------
    TailJunctionNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Serial device instance
    SerialPort serial;

    // Buffer to hold bytes read from the device
    std::vector<uint8_t> read_buffer;

    // Timer fin resets and its corresponding timer duration
    ros::Timer fin_reset_timer;
    ros::Duration fin_reset_timer_duration;

    // Service servers
    ros::ServiceServer reset_fins_server;
    ros::ServiceServer enable_strobe_server;
    ros::ServiceServer disable_strobe_server;
    ros::ServiceServer set_sail_mode_server;
    ros::ServiceServer zero_pressure_server;

    // Publishers
    ros::Publisher pressure_pub;
    ros::Publisher rpm_pub;
    ros::Publisher fin_measurement_request_pub;
    ros::Publisher fin_calibration_status_pub;

    // Subscribers
    ros::Subscriber fins_sub;
    ros::Subscriber motor_sub;
    ros::Subscriber fin_calibration_sub;
    ros::Subscriber fin_measurement_sub;

    // Subscriber for actuator control messages and flag indicating whether
    // actuator control is enabled. If it is disabled, fins and motor messages
    // will be ignored
    ros::Subscriber actuator_control_sub;
    bool actuator_control_enabled = true;

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

    // Indicates whether fin calibration is currently active
    bool calibration_active = false;

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

        // Handle SONAR commands
        if (command_name == "TARE PRESSURE")
        {
            log_info("taring pressure sensor");
            AafPacket aaf_packet = AUV_COMMAND_PACKET();
            aaf_packet.add_field(AUV_COMMAND_TARE_PRESSURE());
            write_packet(aaf_packet);
            result = true;
            return true;
        }

        // Handle STROBE commands
        else if (command_name == "STROBE")
        {
            bool enable = params.get("ENABLE").to_bool();
            log_info("%s strobe", enable ? "enabling" : "disabling");
            AafPacket aaf_packet = AUV_COMMAND_PACKET();
            aaf_packet.add_field(AUV_COMMAND_STROBE(enable));
            write_packet(aaf_packet);
            result = true;
            return true;
        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        write_packet
    // Description: Writes a packet to the tail junction node over serial and
    //              logs the bytes that were written.
    // Arguments:   - packet: Packet to write.
    //--------------------------------------------------------------------------
    void write_packet(AafPacket packet)
    {
        serial.write(packet.get_bytes());
        log_data("[serial] tx " + packet.get_string());
    }

    //--------------------------------------------------------------------------
    // Name:        header_read_callback
    // Description: Handler for packet header.
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void header_read_callback(std::vector<uint8_t> data)
    {
        avl::append(read_buffer, AAF_PACKET_HEADER);
        serial.set_match(Match(2, &TailJunctionNode::descriptor_read_callback, this));
    }

    //--------------------------------------------------------------------------
    // Name:        descriptor_read_callback
    // Description: Handler for packet descriptor.
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void descriptor_read_callback(std::vector<uint8_t> data)
    {
        avl::append(read_buffer, data);
        serial.set_match(Match(data.at(1)+2, &TailJunctionNode::payload_read_callback, this));
    }

    //--------------------------------------------------------------------------
    // Name:        payload_read_callback
    // Description: Handler for packet payload.
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void payload_read_callback(std::vector<uint8_t> data)
    {

        avl::append(read_buffer, data);
        log_data("[serial] rx " + avl::byte_to_hex(read_buffer));
        AafPacket packet(read_buffer);

        // Handle SENSOR_DATA packets
        if (packet.get_descriptor() == SENSOR_DATA_DESC)
        {
            if (packet.has_field(SENSOR_DATA_PRESSURE_DESC))
            {
                std::vector<uint8_t> field_data = packet.get_field(SENSOR_DATA_PRESSURE_DESC).get_data();
                float pressure_bar = avl::from_bytes<float>(field_data, true);
                double pressure_psi = 14.5038 * static_cast<double>(pressure_bar);
                std_msgs::Float64 pressure_msg;
                pressure_msg.data = pressure_psi;
                pressure_pub.publish(pressure_msg);
                log_data("[pressure] %f", pressure_psi);
            }
            if (packet.has_field(SENSOR_DATA_TEMPERATURE_DESC))
            {
                std::vector<uint8_t> field_data = packet.get_field(SENSOR_DATA_TEMPERATURE_DESC).get_data();
                float temperature_c = avl::from_bytes<float>(field_data, true);
                log_data("[temperature] %f", temperature_c);
            }
            if (packet.has_field(SENSOR_DATA_BATTERY_DESC))
            {
                std::vector<uint8_t> field_data = packet.get_field(SENSOR_DATA_BATTERY_DESC).get_data();
                float voltage = avl::from_bytes<float>(avl::subvector(field_data, 0, 4), true);
                float current = avl::from_bytes<float>(avl::subvector(field_data, 4, 4), true);
                log_data("[battery] %f %f", voltage, current);
            }
            if (packet.has_field(SENSOR_DATA_MOTOR_SPEED_DESC))
            {
                std::vector<uint8_t> field_data = packet.get_field(SENSOR_DATA_MOTOR_SPEED_DESC).get_data();
                float motor_speed_rpm = avl::from_bytes<float>(field_data, true) * 60.0;
                std_msgs::Float64 rpm_msg;
                rpm_msg.data = motor_speed_rpm;
                rpm_pub.publish(rpm_msg);
                log_data("[rpm] %.0f", motor_speed_rpm);
            }
        }

        // Handle AUV_COMMAND packets
        if (packet.get_descriptor() == AUV_COMMAND_DESC)
        {
            if (packet.has_field(AUV_COMMAND_FIN_REQUEST_MEASUREMENT_DESC))
            {
                log_data("[fin_measurement_request]");
                std_msgs::Empty fin_measurement_request_msg;
                fin_measurement_request_pub.publish(fin_measurement_request_msg);
            }
        }

        // Handle AUV_DATA packets
        if (packet.get_descriptor() == AUV_DATA_DESC)
        {

            // Handle status field
            if (packet.has_field(AUV_DATA_FIN_CALIBRATION_STATUS_DESC))
            {

                // States from Brian's documentation:
                // 0: Init
                // 1: Run
                // 2: Cal
                // 3: Measure
                // 4: Cal done

                uint16_t state = avl::from_bytes<uint16_t>(
                    packet.get_field(AUV_DATA_FIN_CALIBRATION_STATUS_DESC).get_data(), true);
                std_msgs::UInt16 fin_calibration_status_msg;
                fin_calibration_status_msg.data = state;
                fin_calibration_status_pub.publish(fin_calibration_status_msg);
                log_data("[fin_calibration_status] %d", state);

                // Calibration is active in states 2 and 3
                calibration_active = (state == 2) || (state == 3);

            }

        }

        read_buffer.clear();
        serial.set_match(Match(AAF_PACKET_HEADER, &TailJunctionNode::header_read_callback, this));

    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //                           SERVICE CALLBACKS
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //--------------------------------------------------------------------------
    // Name:        reset_fins_srv_callback
    // Description: Called when the reset_fins service is requested.  Sets fins
    //              to their home positions given in the config file.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool reset_fins_srv_callback(std_srvs::Trigger::Request& request,
                                 std_srvs::Trigger::Response& response)
    {
        AafPacket packet = AUV_COMMAND_PACKET();
        packet.add_field(
            AUV_COMMAND_FIN_POSITIONS(
                get_param<double>("~top/home_angle"),
                get_param<double>("~starboard/home_angle"),
                get_param<double>("~bottom/home_angle"),
                get_param<double>("~port/home_angle")));
        write_packet(packet);
        response.success = true;
        response.message = "[tail_junction_node] success (fins reset)";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        actuator_control_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void actuator_control_msg_callback(const ActuatorControlMsg message)
    {
        log_debug("actuator control message received");
        if (message.enable)
        {
            actuator_control_enabled = true;
            log_debug("enabled actuators");
        }
        else
        {
            actuator_control_enabled = false;
            log_debug("disabled actuators");
        }
    }

    //--------------------------------------------------------------------------
    // Name:        enable_actuators_srv_callback
    // Description: Called when the enable_actuators service is requested.
    //              (Re)enables control of the fins and motor.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool enable_actuators_srv_callback(std_srvs::Trigger::Request& request,
                                       std_srvs::Trigger::Response& response)
    {
        log_debug("enable_actuators_srv_callback called");
        actuator_control_enabled = true;
        response.success = true;
        response.message = "[tail_junction_node] success (actuator control enabled)";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        disable_actuators_srv_callback
    // Description: Called when the disable_actuators service is requested.
    //              Disables control of the fins and motor, resetting fin
    //              position and turning the motor off.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool disable_actuators_srv_callback(std_srvs::Trigger::Request& request,
                                        std_srvs::Trigger::Response& response)
    {
        log_debug("disable_actuators_srv_callback called");
        actuator_control_enabled = false;
        response.success = true;
        response.message = "[tail_junction_node] success (actuator control disabled)";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        enable_strobe_srv_callback
    // Description: Called when the enable_strobe service is requested.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool enable_strobe_srv_callback(std_srvs::Trigger::Request& request,
                                    std_srvs::Trigger::Response& response)
    {
        log_debug("enable_strobe_srv_callback called");
        AafPacket strobe_on_packet = AUV_COMMAND_PACKET();
        strobe_on_packet.add_field(AUV_COMMAND_STROBE(1));
        write_packet(strobe_on_packet);
        response.success = true;
        response.message = "[tail_junction_node] success (strobe enabled)";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        disable_strobe_srv_callback
    // Description: Called when the disable_strobe service is requested.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool disable_strobe_srv_callback(std_srvs::Trigger::Request& request,
                                     std_srvs::Trigger::Response& response)
    {
        log_debug("disable_strobe_srv_callback called");
        AafPacket strobe_off_packet = AUV_COMMAND_PACKET();
        strobe_off_packet.add_field(AUV_COMMAND_STROBE(0));
        write_packet(strobe_off_packet);
        response.success = true;
        response.message = "[tail_junction_node] success (strobe disabled)";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        set_sail_mode_srv_callback
    // Description: Called when the set_sail_mode service is requested.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool set_sail_mode_srv_callback(SailModeSrv::Request& request,
                                    SailModeSrv::Response& response)
    {

        log_debug("set_sail_mode_srv_callback called");

        int mode = 0;
        if (request.mode == "INITIALIZE")
        {
            mode = 0;
            response.success = true;
            response.message = "[tail_junction_node] success (sail set to mode INITIALIZE)";
        }
        else if (request.mode == "IDLE")
        {
            mode = 1;
            response.success = true;
            response.message = "[tail_junction_node] success (sail set to mode IDLE)";
        }

        else if (request.mode == "RUN")
        {
            mode = 2;
            response.success = true;
            response.message = "[tail_junction_node] success (sail set to mode RUN)";
        }
        else
        {
            response.success = false;
            response.message = "[tail_junction_node] success (unrecognized sail mode)";
        }

        AafPacket sail_mode_packet = AUV_COMMAND_PACKET();
        sail_mode_packet.add_field(AUV_COMMAND_MAST_STATUS(mode));
        write_packet(sail_mode_packet);
        log_debug("[tx] %s", sail_mode_packet.get_string().c_str());

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        zero_pressure_srv_callback
    // Description: Called when a zero_pressure service is requested.
    //              Sets an average of the current pressure to zero pressure.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool zero_pressure_srv_callback(std_srvs::Trigger::Request& request,
                                    std_srvs::Trigger::Response& response)
    {
        AafPacket tare_pressure_packet = AUV_COMMAND_PACKET();
        tare_pressure_packet.add_field(AUV_COMMAND_TARE_PRESSURE());
        write_packet(tare_pressure_packet);
        response.success = true;
        response.message = "success [tail_junction_node] (pressure tare complete)";
        return true;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //                          SUBSCRIPTION CALLBACKS
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //--------------------------------------------------------------------------
    // Name:        fins_msg_callback
    // Description: Called when a fins message is received on the fins topic.
    //              Sets fin angle and publishes the corresponding PWM message
    //              if any fin angles have changed.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void fins_msg_callback(const FinsMsg& message)
    {

        // Don't move fins if calibration is active
        if (calibration_active)
        {
            log_warning("fin calibration active, ignoring fins command");
            return;
        }

        // Don't move fins if actuator control is disabled
        if (actuator_control_enabled)
        {

            // Clamp the fin angles to their limits
            double top_angle = avl::clamp(
                avl::rad_to_deg(message.top),
                get_param<double>("~top/min_angle"),
                get_param<double>("~top/max_angle"));

            double starboard_angle = avl::clamp(
                avl::rad_to_deg(message.starboard),
                get_param<double>("~starboard/min_angle"),
                get_param<double>("~starboard/max_angle"));

            double bottom_angle = avl::clamp(
                avl::rad_to_deg(message.bottom),
                get_param<double>("~bottom/min_angle"),
                get_param<double>("~bottom/max_angle"));

            double port_angle = avl::clamp(
                avl::rad_to_deg(message.port),
                get_param<double>("~port/min_angle"),
                get_param<double>("~port/max_angle"));

            // Construct and write the fin position packet
            AafPacket fin_packet = AUV_COMMAND_PACKET();
            fin_packet.add_field(
                AUV_COMMAND_FIN_POSITIONS(
                    top_angle,
                    starboard_angle,
                    bottom_angle,
                    port_angle));
            write_packet(fin_packet);

            // Reset the fin reset timer to prevent fins from being reset
            fin_reset_timer.setPeriod(fin_reset_timer_duration);

            log_data("[fins] %f %f %f %f",
                message.top,
                message.starboard,
                message.bottom,
                message.port);

        }

    }

    //--------------------------------------------------------------------------
    // Name:        motor_msg_callback
    // Description: Called when a motor message is received on the motor topic.
    //              Sets motor percent and publishes the corresponding PWM
    //              message if the motor percent has changed.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void motor_msg_callback(const std_msgs::Float64& message)
    {
        if (actuator_control_enabled)
        {
            AafPacket packet = AUV_COMMAND_PACKET();
            double motor_percent = avl::clamp(message.data, -100.0, 100.0);
            double motor_value = avl::linear_scale(motor_percent, -100.0, 100.0, -1.0, 1.0);
            packet.add_field(AUV_COMMAND_MOTOR(motor_value));
            write_packet(packet);
            log_data("[motor] %f %f", motor_percent, motor_value);
        }
        else
            log_warning("actuator control disabled");
    }

    //--------------------------------------------------------------------------
    // Name:        fin_calibration_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void fin_calibration_msg_callback(const FinCalibrationMsg& message)
    {

        AafPacket packet = AUV_COMMAND_PACKET();
        packet.add_field(AUV_COMMAND_FIN_START_CALIBRATION(
            message.fin_id,
            message.min_angle,
            message.max_angle,
            message.home_angle));
        write_packet(packet);

        log_info("[fin_calibration] %d %f %f %f",
            message.fin_id,
            message.min_angle,
            message.max_angle,
            message.home_angle);

    }

    //--------------------------------------------------------------------------
    // Name:        fin_measurement_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void fin_measurement_msg_callback(const std_msgs::Float64& message)
    {
        AafPacket packet = AUV_DATA_PACKET();
        packet.add_field(AUV_DATA_FIN_ANGLE_MEASUREMENT(message.data));
        write_packet(packet);
    }

    //--------------------------------------------------------------------------
    // Name:        fin_reset_timer_callback
    // Description: Called when the fin reset timer expires. Sets all fins to
    //              their home positions.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void fin_reset_timer_callback(const ros::TimerEvent& event)
    {

        // Don't reset fins if calibration is active
        if (calibration_active) return;

        AafPacket packet = AUV_COMMAND_PACKET();
        packet.add_field(
            AUV_COMMAND_FIN_POSITIONS(
                get_param<double>("~top/home_angle"),
                get_param<double>("~starboard/home_angle"),
                get_param<double>("~bottom/home_angle"),
                get_param<double>("~port/home_angle")));
        write_packet(packet);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log data headers
        add_data_header("[motor] percent value");
        add_data_header("[motor] %% NA");
        add_data_header("[pressure] pressure");
        add_data_header("[pressure] PSI");
        add_data_header("[temperature] temperature");
        add_data_header("[temperature] C");
        add_data_header("[battery] voltage current");
        add_data_header("[battery] V A");
        add_data_header("[rpm] rpm");
        add_data_header("[rpm] RPM");
        add_data_header("[fins] top starboard bottom port");
        add_data_header("[fins] rad rad rad rad");

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));
        serial.set_match(Match(AAF_PACKET_HEADER,
            &TailJunctionNode::header_read_callback, this));

        // Configure and fin reset timer
        fin_reset_timer_duration = ros::Duration(get_param<float>("~fin_reset_duration"));
        fin_reset_timer = node_handle->createTimer(fin_reset_timer_duration,
            &TailJunctionNode::fin_reset_timer_callback, this);

        // Set up the service servers
        reset_fins_server = node_handle->advertiseService(
            "device/reset_fins",
            &TailJunctionNode::reset_fins_srv_callback, this);
        enable_strobe_server = node_handle->advertiseService(
            "device/enable_strobe",
            &TailJunctionNode::enable_strobe_srv_callback, this);
        disable_strobe_server = node_handle->advertiseService(
            "device/disable_strobe",
            &TailJunctionNode::disable_strobe_srv_callback, this);
        set_sail_mode_server = node_handle->advertiseService(
            "device/set_sail_mode",
            &TailJunctionNode::set_sail_mode_srv_callback, this);
        zero_pressure_server = node_handle->advertiseService(
            "device/zero_pressure",
            &TailJunctionNode::zero_pressure_srv_callback, this);

        // Set up the publishers
        pressure_pub = node_handle->advertise<std_msgs::Float64>(
            "device/pressure", 1);
        rpm_pub = node_handle->advertise<std_msgs::Float64>("device/rpm", 1);
        fin_measurement_request_pub = node_handle->advertise<std_msgs::Empty>(
            "device/fin_measurement_request", 1);
        fin_calibration_status_pub = node_handle->advertise<std_msgs::UInt16>(
            "device/fin_calibration_status", 1);

        // Set up the subscribers
        fins_sub = node_handle->subscribe("device/fins", 1,
            &TailJunctionNode::fins_msg_callback, this);
        motor_sub = node_handle->subscribe("device/motor", 1,
            &TailJunctionNode::motor_msg_callback, this);
        fin_calibration_sub = node_handle->subscribe("device/fin_calibration",
            1, &TailJunctionNode::fin_calibration_msg_callback, this);
        fin_measurement_sub = node_handle->subscribe("device/fin_measurement",
            1, &TailJunctionNode::fin_measurement_msg_callback, this);
        actuator_control_sub = node_handle->subscribe("device/actuator_control",
            1, &TailJunctionNode::actuator_control_msg_callback, this);

        log_debug("sending sail idle");
        AafPacket sail_idle_packet = AUV_COMMAND_PACKET();
        sail_idle_packet.add_field(AUV_COMMAND_MAST_STATUS(1));
        write_packet(sail_idle_packet);

        // Set the command handler's callback
        command_handler.set_callback(&TailJunctionNode::command_callback, this);

        // Enable the strobe
        log_debug("enabling strobe");
        AafPacket strobe_on_packet = AUV_COMMAND_PACKET();
        strobe_on_packet.add_field(AUV_COMMAND_STROBE(1));
        write_packet(strobe_on_packet);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::Rate spin_rate(1000);
        while(ros::ok())
        {
            serial.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function when the node is started.
    //--------------------------------------------------------------------------
    void shutdown()
    {

        log_debug("sending sail initialize");
        AafPacket sail_initialize_packet = AUV_COMMAND_PACKET();
        sail_initialize_packet.add_field(AUV_COMMAND_MAST_STATUS(0));
        write_packet(sail_initialize_packet);

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    TailJunctionNode node(argc, argv);
    node.start();
    return 0;
}
