//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to interface with the ASCL power distribution board.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/current (std_msgs/Float64)
//              device/voltage (std_msgs/Float64)
//              device/power (std_msgs/Float64)
//
// Subscribers: None
//==============================================================================

// Core utility
#include <avl_core/node.h>
#include <avl_asio/serial_port.h>
#include <avl_core/util/string.h>

// ROS message includes
#include <std_msgs/Float64.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class AsclPowerNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        AsclPowerNode constructor
    //--------------------------------------------------------------------------
    AsclPowerNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Serial device instance
    SerialPort serial;

    // Publishers for power data
    ros::Publisher voltage_pub;
    ros::Publisher current_pub;
    ros::Publisher power_pub;

private:

    //--------------------------------------------------------------------------
    // Name:        read_handler
    // Description: Handler for NMEA GPS messages
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void read_handler(std::vector<uint8_t> data)
    {

        // Format the data as a string, and remove any \r\n characters
        std::string line = std::string(data.begin(), data.end());
        avl::strip(line, '\r');
        avl::strip(line, '\n');
        log_data("[rx] "+ line);

        try
        {

            // The board's output is formatted as <current_mA>,<voltage_mV>\r\n

            // Split the comma delimited message
            std::vector<std::string> split_line = avl::split(line, ",");
            double current_mA = std::stod(split_line.at(0));
            double voltage_mV = std::stod(split_line.at(1));
            double current = current_mA / 1000.0;
            double voltage = voltage_mV / 1000.0;
            double power = voltage * current;

            std_msgs::Float64 msg;
            msg.data = current;
            current_pub.publish(msg);
            msg.data = voltage;
            voltage_pub.publish(msg);
            msg.data = power;
            power_pub.publish(msg);

            log_data("[power] %.3f %.3f %.3f", voltage, current, power);

        }
        catch (const std::exception& ex)
        {
            log_warning("ignoring invalid message (%s)", line.c_str());
        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log data headers
        add_data_header("[power] voltage current power");
        add_data_header("[power] V A W");

        // Set up the publishers for power data
        voltage_pub = node_handle->advertise<std_msgs::Float64>("device/voltage", 1);
        current_pub = node_handle->advertise<std_msgs::Float64>("device/current", 1);
        power_pub = node_handle->advertise<std_msgs::Float64>("device/power", 1);

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set the match condition to a newline for the board's power messages
        serial.set_match(Match("\n", &AsclPowerNode::read_handler, this));

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

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    AsclPowerNode node(argc, argv);
    node.start();
    return 0;
}
