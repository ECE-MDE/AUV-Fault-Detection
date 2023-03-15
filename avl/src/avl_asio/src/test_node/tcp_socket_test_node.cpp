//==============================================================================
// Autonomous Vehicle Library
//
// Description: Simple test and example usage for the tcp_socket class. Opens a
//              TCP socket conenction to a TCP server and periodically writes a
//              message. Prints any response from the server.
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

// TCP socket
#include <avl_asio/tcp_socket.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TcpSocketTestNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        TcpSocketTestNode constructor
    //--------------------------------------------------------------------------
    TcpSocketTestNode(int argc, char **argv) : Node(argc, argv) {}

private:

    // TCP socket instance
    TcpSocket client;

    // Timer for periodically sending messages
    ros::Timer timer;

    // Message to send to server
    std::string message = "Hello world!";

private:

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
        log_info("received: %s", msg.c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        timer_callback
    // Description: Called when the read match condition is met.
    //--------------------------------------------------------------------------
    void timer_callback(const ros::TimerEvent&)
    {
        log_info("sending: %s", message.c_str());
        client.write(message+"\n", 100);
        log_info("sent: %s", message.c_str());
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Configure the TCP client and connect to the server
        client.set_read_timeout(get_param<int>("~read_timeout"));
        client.set_match(Match("\n", &TcpSocketTestNode::read_callback, this));
        client.connect(get_param<std::string>("~address"),
                       get_param<int>("~port"));

        // Configure the timer
        timer = node_handle->createTimer(
            ros::Duration(get_param<float>("~period")),
            &TcpSocketTestNode::timer_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {

        try
        {
            ros::Rate spin_rate(1000);
            while(ros::ok())
            {
                client.spin_once();
                ros::spinOnce();
                spin_rate.sleep();
            }
        }
        catch(const TimeoutException& ex)
        {
            log_error("%s", ex.what());
        }
        catch(const ConnectionClosedException& ex)
        {
            log_error("server closed connection");
        }

    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function or when an exception is thrown
    //              in the init or run functions.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        client.close();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    TcpSocketTestNode node(argc, argv);
    node.start();
    return 0;
}
