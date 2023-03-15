//==============================================================================
// Autonomous Vehicle Library
//
// Description:
//
// Servers:     /comms/ethernet_tx (avl_msgs/DataTxSrv)
//
// Clients:     None
//
// Publishers:  /comms/ethernet_rx (avl_msgs/DataRxMsg)
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// UDP socket and TCP server classes
#include <avl_asio/udp_socket.h>
#include <avl_asio/tcp_server.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

// Comms architecture enums
#include <avl_comms/comms.h>

// Util functions
#include <avl_core/util/byte.h>

// Included for std::find
#include <algorithm>

// ROS message includes
#include <avl_msgs/DataTxSrv.h>
#include <avl_msgs/DataRxMsg.h>
#include <avl_msgs/HeartbeatMsg.h>
#include <avl_msgs/MissionStatusMsg.h>
#include <avl_msgs/MissionModeSrv.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class EthernetChannelNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        EthernetChannelNode constructor
    //--------------------------------------------------------------------------
    EthernetChannelNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // UDP socket, address, and port mappings for FSD and BSD interfaces
    std::map<CommsInterface, std::unique_ptr<UdpSocket>> udp_socket;
    std::map<CommsInterface, std::string> udp_address;
    std::map<CommsInterface, int> udp_tx_port;
    std::map<CommsInterface, int> udp_rx_port;

    // TCP server and port mappings for FSD and BSD interfaces
    std::map<CommsInterface, std::unique_ptr<TcpServer>> tcp_server;
    std::map<CommsInterface, int> tcp_port;
    std::map<CommsInterface, bool> tcp_connected;

    // Ethernet channel TX server and RX publisher
    ros::ServiceServer ethernet_tx_server;
    ros::Publisher ethernet_rx_pub;

    // Subscriber for heartbeat messages
    ros::Subscriber fsd_heartbeat_sub;
    ros::Subscriber bsd_heartbeat_sub;
    bool broadcast_heartbeats;

    // BSD watchdog timer
    ros::Timer bsd_timer;

    // Mode subscriber
    ros::Subscriber mission_status_sub;
    MissionMode mission_mode = MISSION_MODE_FSD;

    // Service client to request change to FSD mode
    ros::ServiceClient mission_mode_client;

private:

    //--------------------------------------------------------------------------
    // Name:        transmit_bytes
    // Description: Transmits bytes to an interface over TCP.
    // Arguments:   - bytes: Bytes to be transmitted.
    //              - interface: Interface to transmit bytes to.
    //--------------------------------------------------------------------------
    void transmit_bytes_tcp(std::vector<uint8_t> bytes, CommsInterface interface)
    {
        if (tcp_connected[interface])
        {
            try
            {

                double t_start = avl::get_epoch_time();

                tcp_server[interface]->write(bytes);

                double t_end = avl::get_epoch_time();
                double t_transmit = t_end - t_start;

                log_data("[tcp_tx] %s %.3e %s",
                    interface_to_string(interface).c_str(),
                    t_transmit,
                    avl::byte_to_hex(bytes, false).c_str());

            }
            catch (const ConnectionClosedException& ex)
            {
                tcp_connected[interface] = false;
            }
            catch (const std::exception& ex)
            {
                log_error("unable to transmit packet (%s)", ex.what());
                tcp_connected[interface] = false;
            }
        }
    }

    //--------------------------------------------------------------------------
    // Name:        transmit_bytes_udp
    // Description: Transmits bytes to an interface over UDP.
    // Arguments:   - bytes: Bytes to be transmitted.
    //              - interface: Interface to transmit bytes to.
    //--------------------------------------------------------------------------
    void transmit_bytes_udp(std::vector<uint8_t> bytes, CommsInterface interface)
    {
        try
        {

            double t_start = avl::get_epoch_time();

            udp_socket[interface]->send_to(udp_address[interface],
                udp_tx_port[interface], bytes);

            double t_end = avl::get_epoch_time();
            double t_transmit = t_end - t_start;

            log_data("[udp_tx] %s %.3e %s",
                interface_to_string(interface).c_str(),
                t_transmit,
                avl::byte_to_hex(bytes, false).c_str());

        }
        catch (const std::exception& ex)
        {
            log_error("unable to transmit packet (%s)", ex.what());
        }
    }

    //--------------------------------------------------------------------------
    // Name:        ethernet_tx_srv_callback
    // Description: Called when the ethernet channel transmit service is called.
    // Arguments:   - request: Request received on the service.
    //              - response: Response to the service request.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool ethernet_tx_srv_callback(DataTxSrv::Request& request,
                                  DataTxSrv::Response& response)
    {
        auto interface = static_cast<CommsInterface>(request.interface);
        transmit_bytes_tcp(request.data, interface);
        transmit_bytes_udp(request.data, interface);
        response.success = true;
        response.message = "success";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        fsd_packet_callback
    // Description: Called when a packet is received.
    // Arguments:   - packet: Received packet.
    //--------------------------------------------------------------------------
    void packet_callback(Packet packet, CommsInterface interface, bool tcp)
    {

        // Log the received packet
        if (tcp)
            log_data("[tcp_rx] %s %s",
                interface_to_string(interface).c_str(),
                packet.get_string(false).c_str());
        else
            log_data("[udp_rx] %s %s",
                interface_to_string(interface).c_str(),
                packet.get_string(false).c_str());

        // If the packet is from the BSD interface and the mission mode is BSD,
        // reset the BSD watchdog timer
        if (interface == INTERFACE_BSD && mission_mode == MISSION_MODE_BSD)
        {
            double duration = get_param<double>("~bsd/timer_duration");
            if (duration > 0)
            {
                bsd_timer.setPeriod(ros::Duration(duration));
                bsd_timer.start();
            }
            else
            {
                bsd_timer.stop();
            }
        }

        // Publish the received packet
        DataRxMsg msg;
        msg.interface = interface;
        msg.data = packet.get_bytes();
        ethernet_rx_pub.publish(msg);

    }

    //--------------------------------------------------------------------------
    // Name:        heartbeat_msg_callback
    // Description: Called when a heartbeat message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void heartbeat_msg_callback(const HeartbeatMsg& message,
        CommsInterface interface)
    {
        if (broadcast_heartbeats)
        {
            transmit_bytes_udp(message.heartbeat_packet, interface);

            if (interface != INTERFACE_FSD)
                transmit_bytes_tcp(message.heartbeat_packet, interface);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        mission_status_msg_callback
    // Description: Called when a mission status message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void mission_status_msg_callback(const MissionStatusMsg& message)
    {
        mission_mode = static_cast<MissionMode>(message.mode);
        if (mission_mode == MISSION_MODE_BSD)
        {
            double duration = get_param<double>("~bsd/timer_duration");
            if (duration > 0)
            {
                bsd_timer.setPeriod(ros::Duration(duration));
                bsd_timer.start();
            }
            else
            {
                bsd_timer.stop();
            }
        }
        else
        {
            bsd_timer.stop();
        }
    }

    //--------------------------------------------------------------------------
    // Name:        bsd_timer_callback
    // Description: Called when the bsd timer activates
    // Arguments:   - event: timer event
    //--------------------------------------------------------------------------
    void bsd_timer_callback(const ros::TimerEvent& event)
    {

        // The timer is only active if the mission is in BSD mode. If the timer
        // times out in this period, switch back to FSD mode

        MissionModeSrv srv;
        srv.request.mode = MISSION_MODE_FSD;
        mission_mode_client.call(srv);

        log_error("BSD failed to communicate, switching to FSD mode (%s)",
            srv.response.message.c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        session_callback
    // Description: Called when a new TCP client connection is accepted.
    // Arguments:   - socket: Reference to TCP socket to use to communicate with
    //                client
    //--------------------------------------------------------------------------
    void session_callback(CommsInterface interface)
    {

        log_debug("client session started on interface %s",
                    interface_to_string(interface).c_str());
        tcp_connected[interface] = true;

        // Configure the read timeout
        tcp_server[interface]->set_read_timeout(0.0);

        // Bind the packet callbacks for the corresponding interface
        auto packet_cb = std::bind(&EthernetChannelNode::packet_callback,
            this, std::placeholders::_1, interface, true);
        tcp_server[interface]->set_packet_callback(packet_cb);

        // Spin while the client remains connected
        try
        {
            ros::Rate spin_rate(10000);
            while (tcp_connected[interface] &&
                   tcp_server[interface]->is_connected() && ros::ok())
            {
                tcp_server[INTERFACE_FSD]->spin_once();
                udp_socket[INTERFACE_FSD]->spin_once();
                tcp_server[INTERFACE_BSD]->spin_once();
                udp_socket[INTERFACE_BSD]->spin_once();
                ros::spinOnce();
                spin_rate.sleep();
            }
        }
        catch (const TimeoutException&)
        {
            log_debug("read timed out");
        }
        catch (const ConnectionClosedException&)
        {
            log_debug("client closed connection on interface %s",
                        interface_to_string(interface).c_str());
        }

        tcp_connected[interface] = false;
        log_debug("client session ended");

    }

    //--------------------------------------------------------------------------
    // Name:        configure_interface
    // Description: Configures the interface from the config file.
    // Arguments:   - inter: Interface to configure.
    //--------------------------------------------------------------------------
    void configure_interface(CommsInterface inter)
    {

        // Get the interface name for config settngs
        std::string name = inter == INTERFACE_FSD ? "fsd" : "bsd";

        // Get config file settings
        udp_address[inter] = get_param<std::string>("~"+name+"/udp_address");
        udp_tx_port[inter] = get_param<int>("~"+name+"/udp_tx_port");
        udp_rx_port[inter] = get_param<int>("~"+name+"/udp_rx_port");
        tcp_port[inter] = get_param<int>("~"+name+"/tcp_port");

        // Configure and open the TCP server
        tcp_server[inter] = std::unique_ptr<TcpServer>( new TcpServer());
        auto session_cb = std::bind(&EthernetChannelNode::session_callback,
            this, inter);
        tcp_server[inter]->set_session_callback(session_cb);
        tcp_server[inter]->open(tcp_port[inter]);

        // Configure and open the UDP socket
        udp_socket[inter] = std::unique_ptr<UdpSocket>( new UdpSocket());
        udp_socket[inter]->set_read_timeout(0);
        auto udp_packet_cb = std::bind(&EthernetChannelNode::packet_callback,
            this, std::placeholders::_1, inter, false);
        udp_socket[inter]->set_packet_callback(udp_packet_cb);
        udp_socket[inter]->open(udp_rx_port[inter], udp_address[inter]);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Configure the interfaces
        configure_interface(INTERFACE_FSD);
        configure_interface(INTERFACE_BSD);

        // Set up the ethernet channel TX server and RX publisher.
        ethernet_tx_server = node_handle->advertiseService("comms/ethernet_tx",
            &EthernetChannelNode::ethernet_tx_srv_callback, this);
        ethernet_rx_pub = node_handle->advertise<DataRxMsg>
            ("comms/ethernet_rx", 32);

        // Bind the subscriber callbacks to specify FSD or BSD interface
        boost::function<void (const HeartbeatMsg&)> fsd_cb =
            boost::bind(&EthernetChannelNode::heartbeat_msg_callback,
                this, _1, INTERFACE_FSD);

        boost::function<void (const HeartbeatMsg&)> bsd_cb =
            boost::bind(&EthernetChannelNode::heartbeat_msg_callback,
                this, _1, INTERFACE_BSD);

        // Set up the subscriber for heartbeat messages that will be broadcast
        fsd_heartbeat_sub = node_handle->subscribe<HeartbeatMsg>(
            "comms/fsd_heartbeat", 1, fsd_cb);
        bsd_heartbeat_sub = node_handle->subscribe<HeartbeatMsg>(
            "comms/bsd_heartbeat", 1, bsd_cb);
        broadcast_heartbeats = get_param<bool>("~broadcast_heartbeats");

        // Set up the BSD watchdog timer
        ros::Duration bsd_timer_duration(
            get_param<double>("~bsd/timer_duration"));
        bsd_timer = node_handle->createTimer(bsd_timer_duration,
            &EthernetChannelNode::bsd_timer_callback, this, true);
        bsd_timer.stop();

        // Track the current mode FSD or BSD
        mission_status_sub = node_handle->subscribe( "system/mission_status",
            1, &EthernetChannelNode::mission_status_msg_callback, this);

        // Set up the mission mode service client
        mission_mode_client =
            node_handle->serviceClient<MissionModeSrv>("system/mission_mode");

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::Rate spin_rate(10000);
        while(ros::ok())
        {
            tcp_server[INTERFACE_FSD]->spin_once();
            udp_socket[INTERFACE_FSD]->spin_once();
            tcp_server[INTERFACE_BSD]->spin_once();
            udp_socket[INTERFACE_BSD]->spin_once();
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
        tcp_server[INTERFACE_FSD]->close();
        udp_socket[INTERFACE_FSD]->close();
        tcp_server[INTERFACE_BSD]->close();
        udp_socket[INTERFACE_BSD]->close();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    EthernetChannelNode node(argc, argv);
    node.start();
    return 0;
}
