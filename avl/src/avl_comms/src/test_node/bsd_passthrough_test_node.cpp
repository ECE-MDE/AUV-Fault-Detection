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

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

// Communications enums
#include <avl_comms/comms.h>

// TCP socket class for sending packets
#include <avl_asio/tcp_socket.h>

#include <avl_msgs/DataRxMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class BsdPassthroughTestNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        BsdPassthroughTestNode constructor
    //--------------------------------------------------------------------------
    BsdPassthroughTestNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // TCP socket for FSD interface
    TcpSocket fsd_socket;
    std::string fsd_address = "127.0.0.1";
    int fsd_port = 1338;

    // TCP socket for BSD interface
    TcpSocket bsd_socket;
    std::string bsd_address = "127.0.0.1";
    int bsd_port = 2020;

    // Timer for periodically sending packets
    ros::Timer timer;

    ros::Publisher ethernet_rx_pub;

private:

    //--------------------------------------------------------------------------
    // Name:        fsd_packet_callback
    // Description: Called when a packet is received on the FSD interface.
    // Arguments:   - packet: Received packet.
    //--------------------------------------------------------------------------
    void fsd_packet_callback(Packet packet)
    {

        PacketHeader header = packet.get_header();
        uint8_t desc = packet.get_descriptor();
        log_debug("received FSD packet of type %d", desc);

        if (desc == PASSTHROUGH_PACKET)
        {

            PassthroughPacket passthrough_packet(packet);
            PassthroughMessage message = passthrough_packet.get_passthrough_message();

            log_info("================================================================================");
            log_info("FSD received PASSTHROUGH packet");
            log_info("timestamp:      %ju", header.timestamp);
            log_info("timeout:        %u",  header.timeout);
            log_info("source_id:      %u",  header.source_id);
            log_info("destination_id: %u",  header.destination_id);
            log_info("--------------------------------------------------------------------------------");
            if (passthrough_packet.has_origin_id())
                log_info("ORIGIN_ID: %u",  passthrough_packet.get_origin_id());
            log_info("TARGET_ID: %u",  message.target_id);
            log_info("CHANNEL:   %s",  channel_to_string(static_cast<CommsChannel>(message.channel)).c_str());
            log_info("INTERFACE: %s",  interface_to_string(static_cast<CommsInterface>(message.interface)).c_str());
            log_info("DATA:      %s",  avl::byte_to_hex(message.data).c_str());
            log_info("================================================================================");

        }

    }

    //--------------------------------------------------------------------------
    // Name:        bsd_packet_callback
    // Description: Called when a packet is received on the BSD interface.
    // Arguments:   - packet: Received packet.
    //--------------------------------------------------------------------------
    void bsd_packet_callback(Packet packet)
    {

        PacketHeader header = packet.get_header();
        uint8_t desc = packet.get_descriptor();
        log_debug("received BSD packet of type %d", desc);

        if (desc == PASSTHROUGH_PACKET)
        {

            PassthroughPacket passthrough_packet(packet);
            PassthroughMessage message = passthrough_packet.get_passthrough_message();

            log_info("================================================================================");
            log_info("BSD received PASSTHROUGH packet");
            log_info("timestamp:      %ju", header.timestamp);
            log_info("timeout:        %u",  header.timeout);
            log_info("source_id:      %u",  header.source_id);
            log_info("destination_id: %u",  header.destination_id);
            log_info("--------------------------------------------------------------------------------");
            if (passthrough_packet.has_origin_id())
                log_info("ORIGIN_ID: %u",  passthrough_packet.get_origin_id());
            log_info("TARGET_ID: %u",  message.target_id);
            log_info("CHANNEL:   %s",  channel_to_string(static_cast<CommsChannel>(message.channel)).c_str());
            log_info("INTERFACE: %s",  interface_to_string(static_cast<CommsInterface>(message.interface)).c_str());
            log_info("DATA:      %s",  avl::byte_to_hex(message.data).c_str());
            log_info("================================================================================");

        }

    }

    //--------------------------------------------------------------------------
    // Name:        timer_callback
    // Description: Called when the read match condition is met.
    //--------------------------------------------------------------------------
    void timer_callback(const ros::TimerEvent& event)
    {
        log_debug("timer_callback");

        // Send a micro passthrough packet to the FSD as if it had received it
        // from another vehicle (IDs 0x01 -> 0x02 -> 0x03)

        // PacketHeader header;
        // header.timestamp = avl::get_epoch_time_nanoseconds();
        // header.timeout = 0;
        // header.source_id = 0x02;
        // header.destination_id = 0x03;
        //
        // PassthroughMessage msg;
        // msg.target_id = 0x03;
        // msg.channel = CHANNEL_ETHERNET;
        // msg.interface = INTERFACE_BSD;
        // msg.data = {0xAA, 0xBB, 0xCC};
        //
        // PassthroughPacket packet(header, msg);
        // packet.add_field(PASSTHROUGH_ORIGIN_ID, {0x02});
        //
        // std::vector<uint8_t> bytes = MicroPassthroughPacket::from_passthrough_packet(packet);
        // // fsd_socket.write(bytes);
        //
        // DataRxMsg rx_msg;
        // rx_msg.interface = INTERFACE_FSD;
        // rx_msg.data = bytes;
        // ethernet_rx_pub.publish(rx_msg);

        PacketHeader header;
        header.timestamp = 0;
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        PassthroughMessage msg;
        msg.target_id = 0x02;
        msg.channel = CHANNEL_ETHERNET;
        msg.interface = INTERFACE_BSD;
        msg.data = {0xAA, 0xBB, 0xCC};

        PassthroughPacket packet(header, msg);
        // packet.add_field(PASSTHROUGH_ORIGIN_ID, {0x02});

        bsd_socket.write(packet.get_bytes());

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // // This block shows that passthrough -> micro passthrough and the
        // // reverse works as expected
        //
        // PacketHeader header;
        // header.timestamp = avl::get_epoch_time_nanoseconds();
        // header.timeout = 0;
        // header.source_id = 0x01;
        // header.destination_id = 0x02;
        //
        // PassthroughMessage msg;
        // msg.target_id = 0x03;
        // msg.channel = CHANNEL_ETHERNET;
        // msg.interface = INTERFACE_BSD;
        // msg.data = {0xAA, 0xBB, 0xCC};
        //
        // PassthroughPacket packet(header, msg);
        //
        // std::vector<uint8_t> bytes = MicroPassthroughPacket::from_passthrough_packet(packet);
        // log_info("%s", avl::byte_to_hex(bytes).c_str());
        //
        // PassthroughPacket test = MicroPassthroughPacket::to_passthrough_packet(bytes);
        // PacketHeader test_header = test.get_header();
        // PassthroughMessage test_msg = test.get_passthrough_message();
        // log_info("=============================");
        // log_info("timestamp:      %ju", test_header.timestamp);
        // log_info("timeout:        %ju", test_header.timeout);
        // log_info("source_id:      0x%02X",  test_header.source_id);
        // log_info("destination_id: 0x%02X",  test_header.destination_id);
        // log_info("=============================");
        // log_info("target_id: 0x%02X", test_msg.target_id);
        // log_info("channel:   0x%02X", test_msg.channel);
        // log_info("interface: 0x%02X", test_msg.interface);
        // log_info("data:      %s",   avl::byte_to_hex(test_msg.data).c_str());
        // log_info("=============================");
        // // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // Open the FSD TCP socket
        log_info("configuring FSD socket...");
        fsd_socket.set_read_timeout(0);
        fsd_socket.set_packet_callback(
            &BsdPassthroughTestNode::fsd_packet_callback, this);
        while (!fsd_socket.is_connected() && ros::ok())
        {
            try
            {
                log_info("connecting to FSD...");
                fsd_socket.connect(fsd_address, fsd_port);
                log_info("connected to FSD");
            }
            catch (const std::exception& ex)
            {
                log_warning("FSD connect failed, retrying...");
                ros::Duration(1.0).sleep();
            }
        }

        // Open the BSD TCP socket
        log_info("configuring BSD socket...");
        bsd_socket.set_read_timeout(0);
        bsd_socket.set_packet_callback(
            &BsdPassthroughTestNode::bsd_packet_callback, this);
        while (!bsd_socket.is_connected() && ros::ok())
        {
            try
            {
                log_info("connecting to BSD...");
                bsd_socket.connect(bsd_address, bsd_port);
                log_info("connected to BSD");
            }
            catch (const std::exception& ex)
            {
                log_warning("BSD connect failed, retrying...");
                ros::Duration(1.0).sleep();
            }
        }

        // Configure the timer
        ros::Duration timer_duration(5.0);
        timer = node_handle->createTimer(timer_duration,
            &BsdPassthroughTestNode::timer_callback, this);

        ethernet_rx_pub = node_handle->advertise<DataRxMsg>
            ("comms/ethernet_rx", 32);

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
            fsd_socket.spin_once();
            bsd_socket.spin_once();
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

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    BsdPassthroughTestNode node(argc, argv);
    node.start();
    return 0;
}
