//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node for pressure and sound velocity measurements from a
//              Valeport miniSVS sound velocity sensor.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/pressure (std_msgs/Float64)
//              device/sound_velocity (std_msgs/Float64)
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_devices/device_node.h>

// Serial port class
#include <avl_asio/serial_port.h>

// Utility functions
#include <avl_core/util/string.h>

// ROS message includes
#include <std_msgs/Float64.h>

// Valeport command protocol
#include <avl_devices/protocol/valeport.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ValeportMiniSVSNode : public DeviceNode
{

public:
    //--------------------------------------------------------------------------
    // Name:        ValeportMiniSVSNode constructor
    //--------------------------------------------------------------------------
    ValeportMiniSVSNode(int argc, char **argv) : DeviceNode(argc, argv)
    {
    }

private:
    // Serial device instance
    SerialPort serial;

    // Publisher for temperature, conductivity, pressure, salinity and sound
    // velocity data messages
    ros::Publisher pressure_pub;
    ros::Publisher sound_velocity_pub;

    //Variables for basic class
    double pressure_device;
    double sound_velocity_device;

private:
    //--------------------------------------------------------------------------
    // Name:        read_handler
    // Description: Handler for MiniSVS messages
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void read_handler(std::vector<uint8_t> data)
    {
        // Format the data as a string, and remove any \r\n characters
        std::string line = std::string(data.begin(), data.end());
        avl::strip(line, '\r');
        avl::strip(line, '\n');

        try
        {
          // Split the comma delimited message
          std::vector<std::string> split_line = avl::split(line, ",");
        
          //double temperature = std::stod(split_line[0]);
          //double conductivity = std::stod(split_line[1]);
          //double salinity = std::stod(split_line[3]);
          double pressure = std::stod(split_line.at(2));
          std_msgs::Float64 pressure_msg;
          pressure_msg.data = pressure;
          pressure_pub.publish(pressure_msg);

          double sound_velocity = std::stod(split_line.at(4));
          std_msgs::Float64 sound_velocity_msg;
          sound_velocity_msg.data = sound_velocity;
          sound_velocity_pub.publish(sound_velocity_msg);

          // Log data
          log_data("[SV+P] %f %f", sound_velocity, pressure);

          sound_velocity_device = sound_velocity;
          pressure_device = pressure;
        }
        catch (const std::exception &ex)
        {
            log_warning("ignoring invalid line: " + line);
        }
    }

    //--------------------------------------------------------------------------
    // Name:        get_device_parameters
    // Description: Called when a DEVICE packet is requested from the node or
    //              when a DEVICE packet is needed to be published to the FSD
    //              or BSD interface. This function should be implemented by the
    //              device child class to create and return a parameter list.
    // Returns:     Parameter list containing parameters to be added to the
    //              device packet that will be transmitted.
    //--------------------------------------------------------------------------
    avl::ParameterList get_device_parameters()
    {

        avl::ParameterList params;
        params.add(Parameter("PRESSURE", pressure_device));
        params.add(Parameter("SOUND VELOCITY", sound_velocity_device));
        return params;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        log_info("initializing Valeport MiniSVS...");

        // Log data headers
        log_data("[SV+P] sound_velocity pressure");
        log_data("[SV+P] m/s dBar");

        // Set up the publisher for MiniSVS data
        pressure_pub = node_handle->advertise<std_msgs::Float64>("device/pressure", 1);
        sound_velocity_pub = node_handle->advertise<std_msgs::Float64>("device/sound_velocity", 1);

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set the match condition to a newline
        serial.set_match(Match("\n", &ValeportMiniSVSNode::read_handler, this));

        // Configure the sensor
        //serial.write(VALEPORT_SET_FORMAT("SEABIRD"));
        ros::Duration(2.0).sleep();
        //serial.write(VALEPORT_SET_OUTPUT_RATE(get_param<int>("~output_rate")));

        // Set device name and default DEVICE packet output rates
        set_device_name("MINISVS");
        set_device_packet_output_rate(1.0, INTERFACE_FSD);
        set_device_packet_output_rate(1.0, INTERFACE_BSD);
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
    ValeportMiniSVSNode node(argc, argv);
    node.start();
    return 0;
}
