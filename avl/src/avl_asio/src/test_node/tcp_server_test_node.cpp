//==============================================================================
// Autonomous Vehicle Library
//
// Description: Simple test and example usage for the TcpServer class. Opens a
//              TCP server and accepts an incoming connection. Echoes any
//              messages sent by a connected client back to the client.
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

// TCP server
#include <avl_asio/tcp_server.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TcpServerTestNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        TcpServerTestNode constructor
    //--------------------------------------------------------------------------
    TcpServerTestNode(int argc, char **argv) : Node(argc, argv) {}

private:

    // TCP server instance
    TcpServer server;

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
        log_warning("read: %s", msg.c_str());

        // Echo the message back to the client
        log_warning("writing: %s", msg.c_str());
        server.write(msg+"\n");

        // Log the received message
        log_warning("wrote: %s", msg.c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        session_callback
    // Description: Called when a new TCP client connection is accepted.
    // Arguments:   - socket: Reference to TCP socket to use to communicate with
    //                client
    //--------------------------------------------------------------------------
    void session_callback()
    {

        log_info("client connected");

        // Configure the session
        server.set_read_timeout(get_param<int>("~read_timeout"));
        server.set_match(Match("\n", &TcpServerTestNode::read_callback, this));

        // Spin while the client remains connected
        try
        {
            ros::Rate spin_rate(1000);
            while(server.is_connected() && ros::ok())
            {
                server.spin_once();
                ros::spinOnce();
                spin_rate.sleep();
            }
        }
        catch (const TimeoutException& ex)
        {
            log_error("%s", ex.what());
        }
        catch (const ConnectionClosedException& ex)
        {
            log_error("client closed connection");
        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Configure the server and open it
        server.set_session_callback(&TcpServerTestNode::session_callback, this);
        server.open(get_param<int>("~port"));

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
            server.spin_once();
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
        server.close();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    TcpServerTestNode node(argc, argv);
    node.start();
    return 0;
}
