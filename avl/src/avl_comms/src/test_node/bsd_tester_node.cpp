//==============================================================================
// Autonomous Vehicle Library
//
// Description: A test node for sending packets over UDP and receiving
//              responses.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Utility functions
#include <avl_core/util/misc.h>

// UDP socket class for sending packets
#include <avl_asio/udp_socket.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

// Communications enums
#include <avl_comms/comms.h>

// ROS messages
#include <std_srvs/Trigger.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class BsdTesterNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        BsdTesterNode constructor
    //--------------------------------------------------------------------------
    BsdTesterNode(int argc, char **argv) : Node(argc, argv) { }

private:

    // UDP socket used to send and receive mesasges and the multicast address
    // and port that they are passed through
    UdpSocket socket;
    std::string multicast_address;
    int multicast_tx_port;
    int multicast_rx_port;

    // Timer for periodically sending packets
    ros::Timer timer;

    // Vehicle ID that passthrough packets are targeted for
    uint8_t passthrough_target_id;
    ros::ServiceServer passthrough_server;

    // Controls for printing of packet information to the console, from config
    // file
    bool print_heartbeat_packets;
    bool print_response_packets;
    bool print_device_packets;
    bool print_passthrough_packets;

    // Previously received heartbeat for mode comparison to detect when the
    // mode switches
    Heartbeat prev_heartbeat;

    // Yaw value to send for navigating in a square. Incremented by 90 after
    // every action
    double yaw = 0.0;

private:

    //--------------------------------------------------------------------------
    // Name:        timer_callback
    // Description: Called when the read match condition is met.
    //--------------------------------------------------------------------------
    void timer_callback(const ros::TimerEvent& event)
    {

        yaw += 90.0;

        PacketHeader header;
        header.timestamp = avl::get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        Action action1;
        action1.name = "PRIMITIVE";
        action1.mode = ACTION_MODE_SET;
        action1.parameters.add(Parameter("DURATION", 300.0));
        action1.parameters.add(Parameter("YAW", yaw));
        action1.parameters.add(Parameter("RPM", 1500.0));
        action1.parameters.add(Parameter("DEPTH", 3.0));

        ActionPacket packet1(header, action1);

        socket.send_to(multicast_address, multicast_tx_port, packet1.get_bytes());

    }

    //--------------------------------------------------------------------------
    // Name:        passthrough_srv_callback
    // Description: Called when the passthrough service is called.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service call completed, false otherwise.
    //--------------------------------------------------------------------------
    bool passthrough_srv_callback(std_srvs::Trigger::Request& request,
                                  std_srvs::Trigger::Response& response)
    {

        PacketHeader passthrough_dest_header;
        passthrough_dest_header.timestamp = avl::get_epoch_time_nanoseconds();
        passthrough_dest_header.timeout = 0;
        passthrough_dest_header.source_id = 0;
        passthrough_dest_header.destination_id = 0;

        PacketHeader packet_dest_header;
        packet_dest_header.timestamp = avl::get_epoch_time_nanoseconds();
        packet_dest_header.timeout = 0;
        packet_dest_header.source_id = 0;
        packet_dest_header.destination_id = 0;

        Command command;
        command.name = "PING";
        CommandPacket command_packet(passthrough_dest_header, command);

        PassthroughMessage message;
        message.target_id = passthrough_target_id;
        message.channel = CHANNEL_ACOUSTIC;
        message.interface = INTERFACE_BSD;
        message.data = {0xAA, 0xBB, 0xCC, 0xDD};

        PassthroughPacket packet(packet_dest_header, message);

        socket.send_to(multicast_address, multicast_tx_port, packet.get_bytes());
        log_debug("[tx] %s", packet.get_string().c_str());
        response.success = true;
        response.message = "success";
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        packet_callback
    // Description: Called when the read match condition is met.
    // Arguments:   - data: data read by match condition
    //--------------------------------------------------------------------------
    void packet_callback(Packet packet)
    {

        log_packet(packet);

        switch (packet.get_descriptor())
        {

            case HEARTBEAT_PACKET:
            {

                HeartbeatPacket heartbeat_packet(packet.get_bytes());
                Heartbeat heartbeat = heartbeat_packet.get_heartbeat();

                if (heartbeat.mission_mode == MISSION_MODE_BSD &&
                    prev_heartbeat.mission_mode != MISSION_MODE_BSD)
                {
                    log_warning("SWITCHED INTO BSD MODE!");
                }
                else if (heartbeat.mission_mode != MISSION_MODE_BSD &&
                         prev_heartbeat.mission_mode == MISSION_MODE_BSD)
                {
                    log_warning("SWITCHED OUT OF BSD MODE!");
                }

                prev_heartbeat = heartbeat;
                break;

            } // case HEARTBEAT_PACKET

            default:
            {

            }

        } // switch

    }

    //--------------------------------------------------------------------------
    // Name:        log_packet
    // Description: Logs information about a packet.
    // Arguments:   - packet: Packet to be logged.
    //--------------------------------------------------------------------------
    void log_packet(Packet packet)
    {

        PacketHeader header = packet.get_header();

        switch (packet.get_descriptor())
        {

            case RESPONSE_PACKET:
            {
                if (print_response_packets)
                {
                    ResponsePacket response_packet(packet.get_bytes());
                    Response response = response_packet.get_response();
                    std::string message(response.data.begin(), response.data.end());
                    log_info("================================================================================");
                    log_info("Recevied RESPONSE packet");
                    log_info("timestamp:      %ju", header.timestamp);
                    log_info("timeout:        %ju", header.timeout);
                    log_info("source_id:      %u",  header.source_id);
                    log_info("destination_id: %u",  header.destination_id);
                    log_info("--------------------------------------------------------------------------------");
                    log_info("SOURCE:         %ju", response.source);
                    log_info("RESULT:         %s",  response.result ? "true" : "false");
                    log_info("DATA:           %s",  message.c_str());
                    log_info("================================================================================");
                }
                break;
            }

            case HEARTBEAT_PACKET:
            {
                if (print_heartbeat_packets)
                {
                    HeartbeatPacket heartbeat_packet(packet.get_bytes());
                    Heartbeat heartbeat = heartbeat_packet.get_heartbeat();
                    log_info("================================================================================");
                    log_info("Recevied HEARTBEAT packet");
                    log_info("timestamp:      %ju",  header.timestamp);
                    log_info("timeout:        %ju",  header.timeout);
                    log_info("source_id:      %u",   header.source_id);
                    log_info("destination_id: %u",   header.destination_id);
                    log_info("--------------------------------------------------------------------------------");

                    log_info("STATUS:               %d",   heartbeat.status);
                    log_info("ROLL:                 %.2f", heartbeat.roll);
                    log_info("PITCH:                %.2f", heartbeat.pitch);
                    log_info("YAW:                  %.2f", heartbeat.yaw);
                    log_info("VN:                   %.2f", heartbeat.vn);
                    log_info("VE:                   %.2f", heartbeat.ve);
                    log_info("VD:                   %.2f", heartbeat.vd);
                    log_info("DEPTH:                %.2f", heartbeat.depth);
                    log_info("HEIGHT:               %.2f", heartbeat.height);
                    log_info("RPM:                  %.2f", heartbeat.rpm);
                    log_info("VOLTAGE:              %.2f", heartbeat.voltage);
                    log_info("UMODEM_SYNC:          %d",   heartbeat.umodem_sync);
                    log_info("IRIDIUM_STR:          %d",   heartbeat.iridium_str);

                    log_info("MISSION_MODE:         %d",   heartbeat.mission_mode);
                    log_info("FSD_MISSION_STATE:    %d",   heartbeat.fsd_mission_state);
                    log_info("FSD_CURRENT_ACTION:   %d",   heartbeat.fsd_current_action);
                    log_info("FSD_TOTAL_ACTIONS:    %d",   heartbeat.fsd_total_actions);
                    log_info("FSD_ACTION_PERCENT:   %.2f", heartbeat.fsd_action_percent);
                    log_info("BSD_MISSION_STATE:    %d",   heartbeat.bsd_mission_state);
                    log_info("BSD_CURRENT_ACTION:   %d",   heartbeat.bsd_current_action);
                    log_info("BSD_TOTAL_ACTIONS:    %d",   heartbeat.bsd_total_actions);
                    log_info("BSD_ACTION_PERCENT:   %.2f", heartbeat.bsd_action_percent);

                    log_info("GPS_SATS:             %d",   heartbeat.gps_sats);
                    log_info("GPS_LAT:              %.2f", heartbeat.gps_lat);
                    log_info("GPS_LON:              %.2f", heartbeat.gps_lon);
                    log_info("GPS_ALT:              %.2f", heartbeat.gps_alt);

                    log_info("NAV_LAT:              %.2f", heartbeat.nav_lat);
                    log_info("NAV_LON:              %.2f", heartbeat.nav_lon);
                    log_info("NAV_ALT:              %.2f", heartbeat.nav_alt);
                    log_info("================================================================================");
                }
                break;
            }

            case DEVICE_PACKET:
            {
                if (print_device_packets)
                {
                    DevicePacket device_packet(packet);
                    DeviceInfo device_info = device_packet.get_device_info();
                    log_info("================================================================================");
                    log_info("Recevied DEVICE packet");
                    log_info("timestamp:      %ju",  header.timestamp);
                    log_info("timeout:        %ju",  header.timeout);
                    log_info("source_id:      %u",   header.source_id);
                    log_info("destination_id: %u",   header.destination_id);
                    log_info("--------------------------------------------------------------------------------");
                    log_info("DEVICE NAME: %s", device_info.name.c_str());
                    log_info("PARAMETERS:");
                    for (const Parameter& param : device_info.parameters)
                        log_info("name: %-12s   type: %d   value: %s",
                            param.name.c_str(),
                            param.type,
                            avl::byte_to_hex(param.value).c_str());
                    log_info("================================================================================");
                }
                break;
            }

            case PASSTHROUGH_PACKET:
            {
                if (print_passthrough_packets)
                {
                    PassthroughPacket passthrough_packet(packet.get_bytes());
                    PacketHeader header = packet.get_header();
                    PassthroughMessage message = passthrough_packet.get_passthrough_message();
                    log_info("================================================================================");
                    log_info("Recevied PASSTHROUGH packet");
                    log_info("timestamp:      %ju", header.timestamp);
                    log_info("timeout:        %u",  header.timeout);
                    log_info("source_id:      %u",  header.source_id);
                    log_info("destination_id: %u",  header.destination_id);
                    log_info("--------------------------------------------------------------------------------");
                    log_info("TARGET_ID: %u",  message.target_id);
                    log_info("CHANNEL:   %s",  channel_to_string(static_cast<CommsChannel>(message.channel)).c_str());
                    log_info("INTERFACE: %s",  interface_to_string(static_cast<CommsInterface>(message.interface)).c_str());
                    log_info("DATA:      %s",  avl::byte_to_hex(message.data).c_str());
                    log_info("================================================================================");
                }
                break;
            }

            default:
            {
                // TODO: Other packet types if needed
            }

        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get the config file settings
        multicast_address = avl::get_param<std::string>("~multicast_address");
        multicast_tx_port = avl::get_param<int>("~multicast_tx_port");
        multicast_rx_port = avl::get_param<int>("~multicast_rx_port");
        passthrough_target_id = get_param<int>("~passthrough_target_id");

        // Open the UDP socket
        socket.set_read_timeout(0);
        socket.set_packet_callback(&BsdTesterNode::packet_callback, this);
        socket.open(multicast_rx_port, multicast_address);

        // Configure the timer
        ros::Duration timer_duration(30.0);
        timer = node_handle->createTimer(timer_duration,
            &BsdTesterNode::timer_callback, this);

        passthrough_server = node_handle->advertiseService(
            "/bsd_tester/passthrough",
            &BsdTesterNode::passthrough_srv_callback, this);

        ros::Duration(3.0).sleep();

        // Add some actions to the mission queue
        PacketHeader header;
        header.timestamp = avl::get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        Action action1;
        action1.name = "PRIMITIVE";
        action1.mode = ACTION_MODE_APPEND;
        action1.parameters.add(Parameter("DURATION", 300.0));
        action1.parameters.add(Parameter("YAW", 0.0));
        action1.parameters.add(Parameter("RPM", 1500.0));
        action1.parameters.add(Parameter("DEPTH", 3.0));

        ActionPacket packet1(header, action1);

        socket.send_to(multicast_address, multicast_tx_port, packet1.get_bytes());

        Command command;
        command.name = "MISSION";
        command.parameters.add(Parameter("COMMAND", 0));

        CommandPacket command_packet(header, command);

        socket.send_to(multicast_address, multicast_tx_port, command_packet.get_bytes());

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
            socket.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function or when an exception is thrown
    //              in the init or run functions.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        socket.close();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    BsdTesterNode node(argc, argv);
    node.start();
    return 0;
}
