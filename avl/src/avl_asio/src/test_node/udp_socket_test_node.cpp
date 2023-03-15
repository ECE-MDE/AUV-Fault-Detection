//==============================================================================
// Autonomous Vehicle Library
//
// Description: A test node for the UdpSocket class.
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

// UDP socket class
#include <avl_asio/udp_socket.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class UdpSocketTestNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        UdpSocketTestNode constructor
    //--------------------------------------------------------------------------
    UdpSocketTestNode(int argc, char **argv) : Node(argc, argv) {}

private:

    // UDP multicast socket instance
    UdpSocket socket;

    // Timer for periodically sending messages
    ros::Timer timer;

    // Message to broadcast
    std::string message = "Hello world!";

private:

    //--------------------------------------------------------------------------
    // Name:        timer_callback
    // Description: Called when the read match condition is met.
    //--------------------------------------------------------------------------
    void timer_callback(const ros::TimerEvent&)
    {
        std::string send_address = get_param<std::string>("~send_address");
        int send_port = get_param<int>("~send_port");
        socket.send_to(send_address, send_port, message+"\n");
        log_data("sent: %s", message.c_str());
    }

    //--------------------------------------------------------------------------
    // Name:        read_callback
    // Description: Called when the read match condition is met.
    // Arguments:   - data: data read by match condition
    //--------------------------------------------------------------------------
    void read_callback(std::vector<uint8_t> data)
    {

        // Format the received message as a string. Remove the trailing newline.
        std::string msg = std::string(data.begin(), data.end()-1);

        // Log the received message
        log_data("received: %s", msg.c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Open the UDP socket
        socket.set_read_timeout(0);
        socket.set_match(Match("\n", &UdpSocketTestNode::read_callback, this));
        socket.open(get_param<int>("~listen_port"));

        // Configure the timer
        ros::Duration timer_duration(get_param<float>("~period"));
        timer = node_handle->createTimer(timer_duration,
            &UdpSocketTestNode::timer_callback, this);

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
    UdpSocketTestNode node(argc, argv);
    node.start();
    return 0;
}
