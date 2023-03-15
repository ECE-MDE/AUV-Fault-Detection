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

class FsdTesterNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        FsdTesterNode constructor
    //--------------------------------------------------------------------------
    FsdTesterNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // UDP socket used to send and receive mesasges and the multicast address
    // and port that they are passed through
    UdpSocket socket;
    std::string multicast_address;
    int multicast_port;

    // UDP multicast socket for receiving acoustic packets
    UdpSocket acoustic_socket;
    std::string acoustic_multicast_address;
    int acoustic_multicast_port;

    // UDP multicast socket for receiving Iridium packets
    UdpSocket iridium_socket;
    std::string iridium_multicast_address;
    int iridium_multicast_port;

    // Vehicle ID number that the node will pretend to have
    uint8_t vehicle_id;

    // Vehicle ID to send packets to
    uint8_t packet_destination_id;

    // Vehicle ID that passthrough packets are targeted for
    uint8_t passthrough_target_id;

    // Timer for periodically sending packets
    ros::Timer timer;

    // Service server for sending COMMAND packets with a MODE field
    ros::ServiceServer mode_server;
    ros::ServiceServer passthrough_server;

private:

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
        passthrough_dest_header.source_id = vehicle_id;
        passthrough_dest_header.destination_id = passthrough_target_id;

        PacketHeader packet_dest_header;
        packet_dest_header.timestamp = avl::get_epoch_time_nanoseconds();
        packet_dest_header.timeout = 0;
        packet_dest_header.source_id = vehicle_id;
        packet_dest_header.destination_id = packet_destination_id;

        Command command;
        command.name = "PING";
        CommandPacket command_packet(passthrough_dest_header, command);

        PassthroughMessage message;
        message.target_id = passthrough_target_id;
        message.channel = CHANNEL_ACOUSTIC;
        message.interface = INTERFACE_FSD;
        message.data = command_packet.get_bytes();

        PassthroughPacket packet(packet_dest_header, message);

        socket.send_to(multicast_address, multicast_port, packet.get_bytes());
        log_debug("[tx] %s", packet.get_string().c_str());
        response.success = true;
        response.message = "success";
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        mode_srv_callback
    // Description: Called when the mode service is called.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service call completed, false otherwise.
    //--------------------------------------------------------------------------
    bool mode_srv_callback(std_srvs::Trigger::Request& request,
                           std_srvs::Trigger::Response& response)
    {
        PacketHeader header;
        header.timestamp = avl::get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = avl::get_vehicle_id();
        header.destination_id = packet_destination_id;

        Command command;
        command.name = "BSD MODE";
        command.parameters.add(Parameter("DURATION", 5));
        CommandPacket packet(header, command);
        socket.send_to(multicast_address, multicast_port, packet.get_bytes());
        log_debug("[tx] %s", packet.get_string().c_str());
        response.success = true;
        response.message = "success";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        timer_callback
    // Description: Called when the read match condition is met.
    //--------------------------------------------------------------------------
    void timer_callback(const ros::TimerEvent& event)
    {
        PacketHeader header;
        header.timestamp = avl::get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = avl::get_vehicle_id();
        header.destination_id = packet_destination_id;
        Command command;
        command.name = "PING";
        CommandPacket packet(header, command);
        socket.send_to(multicast_address, multicast_port, packet.get_bytes());
        log_debug("[tx] %s", packet.get_string().c_str());
    }

    //--------------------------------------------------------------------------
    // Name:        packet_callback
    // Description: Called when the read match condition is met.
    // Arguments:   - data: data read by match condition
    //--------------------------------------------------------------------------
    void packet_callback(Packet packet)
    {
        log_data("[ethernet_rx] %s", packet.get_string().c_str());
        handle_packet(packet, CHANNEL_ETHERNET);
    }

    //--------------------------------------------------------------------------
    // Name:        acoustic_packet_callback
    // Description: Called when a packet is received.
    // Arguments:   - packet: Received packet.
    //--------------------------------------------------------------------------
    void acoustic_packet_callback(Packet packet)
    {
        log_data("[acoustic_rx] %s", packet.get_string().c_str());
        handle_packet(packet, CHANNEL_ACOUSTIC);
    }

    //--------------------------------------------------------------------------
    // Name:        iridium_packet_callback
    // Description: Called when a packet is received.
    // Arguments:   - packet: Received packet.
    //--------------------------------------------------------------------------
    void iridium_packet_callback(Packet packet)
    {
        log_data("[iridium_rx] %s", packet.get_string().c_str());
        handle_packet(packet, CHANNEL_IRIDIUM);
    }

    //--------------------------------------------------------------------------
    // Name:        handle_packet
    // Description: Handles a received packet
    // Arguments:   - packet: Packet to be handled.
    // Arguments:   - channel: Channel the packet was received through.
    //--------------------------------------------------------------------------
    void handle_packet(Packet packet, CommsChannel channel)
    {

        PacketHeader header = packet.get_header();

        // Do not handle packets that are not destined for this vehicle or all
        // vehicles
        if (header.destination_id != vehicle_id &&
            header.destination_id != 0)
            return;

        // Handle the packet based on its type
        switch (packet.get_descriptor())
        {

            case RESPONSE_PACKET:
            {
                if (get_param<bool>("~log_response_packets"))
                {
                    ResponsePacket response_packet(packet.get_bytes());
                    Response response = response_packet.get_response();
                    std::string message(response.data.begin(), response.data.end());
                    log_info("================================================================================");
                    log_info("Recevied RESPONSE packet through %s channel", channel_to_string(channel).c_str());
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
                if (get_param<bool>("~log_heartbeat_packets"))
                {
                    HeartbeatPacket heartbeat_packet(packet.get_bytes());
                    Heartbeat heartbeat = heartbeat_packet.get_heartbeat();
                    log_info("================================================================================");
                    log_info("Recevied HEARTBEAT packet through %s channel", channel_to_string(channel).c_str());
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
        vehicle_id = avl::get_param<int>("~vehicle_id");
        packet_destination_id = avl::get_param<int>("~packet_destination_id");
        passthrough_target_id = avl::get_param<int>("~passthrough_target_id");
        multicast_address = avl::get_param<std::string>("~multicast_address");
        multicast_port = avl::get_param<int>("~multicast_port");

        // Open the UDP socket
        socket.set_read_timeout(0);
        socket.set_packet_callback(&FsdTesterNode::packet_callback, this);
        socket.open(multicast_port, multicast_address);

        // Open the acoustic UDP multicast socket and set the packet callback
        acoustic_multicast_address =
            get_param<std::string>("~acoustic_multicast_address");
        acoustic_multicast_port = get_param<int>("~acoustic_multicast_port");
        acoustic_socket.set_read_timeout(0);
        acoustic_socket.set_packet_callback(
            &FsdTesterNode::acoustic_packet_callback, this);
        acoustic_socket.open(acoustic_multicast_port,
                             acoustic_multicast_address);

        // Open the Iridium UDP multicast socket and set the packet callback
        iridium_multicast_address =
            get_param<std::string>("~iridium_multicast_address");
        iridium_multicast_port = get_param<int>("~iridium_multicast_port");
        iridium_socket.set_read_timeout(0);
        iridium_socket.set_packet_callback(
            &FsdTesterNode::iridium_packet_callback, this);
        iridium_socket.open(iridium_multicast_port,
                          iridium_multicast_address);

        // // Configure the timer
        // ros::Duration timer_duration(3.0);
        // timer = node_handle->createTimer(timer_duration,
        //     &FsdTesterNode::timer_callback, this);

        // Set up the mode service
        mode_server = node_handle->advertiseService(
            "/fsd_tester/mode",
            &FsdTesterNode::mode_srv_callback, this);

        passthrough_server = node_handle->advertiseService(
            "/fsd_tester/passthrough",
            &FsdTesterNode::passthrough_srv_callback, this);

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
            acoustic_socket.spin_once();
            iridium_socket.spin_once();
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
    FsdTesterNode node(argc, argv);
    node.start();
    return 0;
}
