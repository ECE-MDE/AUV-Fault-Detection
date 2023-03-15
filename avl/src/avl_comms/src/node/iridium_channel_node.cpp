//==============================================================================
// Autonomous Vehicle Library
//
// Description: Handles interfacing with an Iridium SBD modem node to provide
//              read and write functionality over the Iridium network. Contains
//              a transmit queue and will attempt to transmit messages from it
//              while Iridium service is available.
//
// Servers:     /comms/iridium_tx (avl_msgs/DataTxSrv)
//
// Clients:     device/iridium/sbd_rx (avl_msgs/ByteArraySrv)
//
// Publishers:  /comms/iridium_rx (avl_msgs/DataRxMsg)
//
// Subscribers: device/iridium/sbd_rx (avl_msgs/ByteArrayMsg)
//              device/iridium/signal (avl_msgs/IridiumSignalMsg)
//              system/heartbeat (avl_msgs/HeartbeatMsg)
//==============================================================================

// Base node class
#include <avl_core/node.h>

// ROS messages
#include <avl_msgs/DataTxSrv.h>
#include <avl_msgs/ByteArraySrv.h>
#include <avl_msgs/DataRxMsg.h>
#include <avl_msgs/ByteArrayMsg.h>
#include <avl_msgs/IridiumSignalMsg.h>
#include <avl_msgs/HeartbeatMsg.h>
using namespace avl_msgs;

// Transmit queue class
#include <avl_comms/transmit_queue.h>

// Comms architecture enums
#include <avl_comms/comms.h>

// Util functions
#include <avl_core/util/byte.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class IridiumChannelNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        IridiumChannelNode constructor
    //--------------------------------------------------------------------------
    IridiumChannelNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Iridium channel TX server and RX publisher
    ros::ServiceServer iridium_tx_server;
    ros::Publisher iridium_rx_pub;

    // Clients and subscriber for interfacing with the Iridium modem
    ros::ServiceClient modem_tx_client;
    ros::Subscriber modem_rx_sub;
    ros::Subscriber modem_signal_sub;

    // Variables for heartbeat packet broadcasting
    ros::Subscriber heartbeat_sub;
    HeartbeatMsg heartbeat_msg;
    ros::Timer heartbeat_timer;

    // Config file settings
    bool broadcast_heartbeats;
    int min_signal_strengh;

    // TX queue to hold messages to be transmitted when the modem achieves a
    // connection to the Iridium network
    TransmitQueue tx_queue;

    // Flag indicating whether iridium signal is sufficiently strong to attempt
    // to transmit
    bool signal_sufficient = false;

private:

    //--------------------------------------------------------------------------
    // Name:        iridium_tx_srv_callback
    // Description: Service callback called when the iridium interface transmit
    //              service is requested by a client.
    // Arguments:   - request: Request message received from the client.
    //              - response: Response message to be returned to the client.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool iridium_tx_srv_callback(DataTxSrv::Request& request,
                                 DataTxSrv::Response& response)
    {
        tx_queue.push(request.interface, request.data);
            log_info("added message to queue with priority %d: %s",
        request.interface, avl::byte_to_hex(request.data).c_str());
        response.success = true;
        response.message = "success";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        response_timer_callback
    // Description: Called when the response timer expires.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void heartbeat_timer_callback(const ros::TimerEvent& event)
    {
        if (broadcast_heartbeats)
            tx_queue.set_heartbeat(heartbeat_msg.micro_heartbeat_packet);
    }

    //--------------------------------------------------------------------------
    // Name:        heartbeat_msg_callback
    // Description: Called when a heartbeat message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void heartbeat_msg_callback(const HeartbeatMsg& message)
    {
        heartbeat_msg = message;
    }

    //--------------------------------------------------------------------------
    // Name:        signal_msg_callback
    // Description: Called when an Iridium signal message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void signal_msg_callback(const IridiumSignalMsg& message)
    {
        signal_sufficient = message.service_available &&
                            message.signal_strength > min_signal_strengh;
    }

    //--------------------------------------------------------------------------
    // Name:        modem_rx_msg_callback
    // Description: Called when a modem data message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void modem_rx_msg_callback(const ByteArrayMsg& message)
    {

        // Create and publish a data receive message
        std::vector<uint8_t> bytes = message.data;
        DataRxMsg iridium_rx_msg;
        iridium_rx_msg.interface = INTERFACE_FSD;
        iridium_rx_msg.data = bytes;
        iridium_rx_pub.publish(iridium_rx_msg);

        // Log the received data
        log_data("[rx] %s", avl::byte_to_hex(bytes).c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get the config file settings
        min_signal_strengh = get_param<int>("~min_signal_strengh");
        broadcast_heartbeats = get_param<bool>("~broadcast_heartbeats");

        // Set up the Iridium channel TX server and RX publisher.
        iridium_tx_server = node_handle->advertiseService("comms/iridium_tx",
            &IridiumChannelNode::iridium_tx_srv_callback, this);
        iridium_rx_pub = node_handle->advertise<DataRxMsg>
            ("comms/iridium_rx", 32);

        // Set up the modem interface client and subscriber
        modem_signal_sub = node_handle->subscribe("device/iridium/signal", 1,
            &IridiumChannelNode::signal_msg_callback, this);
        modem_rx_sub = node_handle->subscribe("device/iridium/sbd_rx", 1,
            &IridiumChannelNode::modem_rx_msg_callback, this);
        modem_tx_client = node_handle->serviceClient<ByteArraySrv>(
            "device/iridium/sbd_tx");

        // Set up the subscriber and timer for heartbeat messages
        heartbeat_sub = node_handle->subscribe("comms/fsd_heartbeat", 1,
            &IridiumChannelNode::heartbeat_msg_callback, this);
        ros::Duration heartbeat_timer_duration(
            1.0/get_param<double>("~heartbeat_broadcast_rate"));
        heartbeat_timer = node_handle->createTimer(heartbeat_timer_duration,
            &IridiumChannelNode::heartbeat_timer_callback, this);

        // Wait for the Iridium modem node to start up
        modem_tx_client.waitForExistence();

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

            if (!tx_queue.empty() && signal_sufficient)
            {

                ByteArraySrv modem_tx_srv;
                modem_tx_srv.request.data = tx_queue.top();
                modem_tx_client.call(modem_tx_srv);

                if (modem_tx_srv.response.success)
                {
                    log_debug("iridium transmit successful");
                    tx_queue.pop();
                }
                else
                {
                    log_debug("iridium transmit failed: %s",
                        modem_tx_srv.response.message);
                }

            }

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
    IridiumChannelNode node(argc, argv);
    node.start();
    return 0;
}
