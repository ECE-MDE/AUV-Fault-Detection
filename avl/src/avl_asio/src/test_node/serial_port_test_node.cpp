//==============================================================================
// Autonomous Vehicle Library
//
// Description: Simple test and example usage for the SerialPort class. Opens a
//          serial port, writes an example message, and prints all received
//          messages. Serial port settings can be configured in the config file.
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

// Serial port
#include <avl_asio/serial_port.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class SerialPortTestNode : public Node
{

private:

    // Serial port instance
    SerialPort serial;

public:

    //--------------------------------------------------------------------------
    // Name:        SerialPortTestNode constructor
    //--------------------------------------------------------------------------
    SerialPortTestNode(int argc, char **argv) : Node(argc, argv) {}

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
        log_data("received: %s", msg.c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Configure and open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.set_match(Match("\n", &SerialPortTestNode::read_callback, this));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Write a configuration command
        serial.write("$CCCFQ,SRC\r\n");

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
                serial.spin_once();
            }
        }
        catch(const TimeoutException& ex)
        {
            log_error("read timed out");
        }

    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function or when an exception is thrown
    //              in the init or run functions.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        serial.close();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    SerialPortTestNode node(argc, argv);
    node.start();
    return 0;
}
