//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node for temperature and conductivity measurements from a
//              Valeport miniCT Probe temperature and conductivity sensor.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/conductivity (std_msgs/Float64)
//              device/density (std_msgs/Float64)
//              device/salinity (std_msgs/Float64)
//              device/temperature (std_msgs/Float64)
//
// Subscribers: device/pressure (std_msgs/Float64)
//==============================================================================

// Device node base class
#include <avl_devices/device_node.h>

// Serial port class
#include <avl_asio/serial_port.h>

// Utility functions
#include <avl_core/util/string.h>
#include <avl_devices/algorithm/density_calculator.h>

// ROS message includes
#include <std_msgs/Float64.h>

// Valeport command protocol
#include <avl_devices/protocol/valeport.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ValeportMinictNode : public DeviceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        ValeportMinictNode constructor
    //--------------------------------------------------------------------------
    ValeportMinictNode(int argc, char **argv) : DeviceNode(argc, argv)
    {

    }

private:

    // Serial port instance
    SerialPort serial;

    // Publishers and Subscribers
    ros::Publisher conductivity_pub;
    ros::Publisher density_pub;
    ros::Publisher salinity_pub;
    ros::Publisher temperature_pub;
    ros::Subscriber pressure_sub;

    // Global variables
    double temperature_device;
    double conductivity_device;
    double density_device;
    double salinity_device;
    double pressure;

private:

    //--------------------------------------------------------------------------
    // Name:        pressure_callback 
    // Description: Process messages on the pressure topic
    // Arguments:   - msg: data from ROS topic
    //--------------------------------------------------------------------------
    void pressure_callback(const std_msgs::Float64& msg)
    {
        pressure = msg.data;
    }

    //--------------------------------------------------------------------------
    // Name:        read_handler
    // Description: Handler for MiniCT messages
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
            // Split the comma delimited message and extract the measurements
            std::vector<std::string> split_line = avl::split(line, ",");

            double temperature = std::stod(split_line.at(0));
            std_msgs::Float64 temperature_msg;
            temperature_msg.data = temperature;
            temperature_pub.publish(temperature_msg);

            double conductivity = std::stod(split_line.at(1));
            std_msgs::Float64 conductivity_msg;
            conductivity_msg.data = conductivity;
            conductivity_pub.publish(conductivity_msg);
   
            double salinity = SALINITY_CALCULATOR(conductivity, temperature,
                pressure);
            std_msgs::Float64 salinity_msg;
            salinity_msg.data = salinity;
            salinity_pub.publish(salinity_msg);
            salinity_device = salinity; 
  
            double density = std::nan("");
            if ( VALIDITY_CHECK(conductivity, temperature, salinity) )
            {
                density = CALCULATE_DENSITY(conductivity, temperature,
                    pressure); 
                std_msgs::Float64 density_msg;
                density_msg.data = density;
                density_pub.publish(density_msg);
            }
            density_device = density;

            // Log data
            log_data("[CTD_+_S] %f %f %f %f", conductivity, temperature,
                density, salinity);

            conductivity_device = conductivity;
            temperature_device = temperature;

        }
        catch (const std::exception& ex)
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
        params.add(Parameter("TEMP", temperature_device));
        params.add(Parameter("COND",  conductivity_device));
        params.add(Parameter("DENSITY",  density_device));
        params.add(Parameter("SALINITY",  salinity_device));
        return params;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {
        // Log data headers
        log_data("[CTD_+_S] conductivity temperature density salinity");
        log_data("[CTD_+_S] degC mS/cm kg/m^3 PSU");

        // Set up the publisher for GPS data
        conductivity_pub = node_handle->advertise<std_msgs::Float64>
            ("device/conductivity", 1);
        density_pub = node_handle->advertise<std_msgs::Float64>
            ("device/density", 1);
        salinity_pub = node_handle->advertise<std_msgs::Float64>
            ("device/salinity", 1);
        temperature_pub = node_handle->advertise<std_msgs::Float64>
            ("device/temperature", 1);
        pressure_sub = node_handle->subscribe("device/pressure", 1, 
            &ValeportMinictNode::pressure_callback, this);

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
            get_param<int>("~serial/baudrate"));

        // Set the match condition to a newline for NMEA strings
        serial.set_match(Match("\n", &ValeportMinictNode::read_handler, this));

        // Configure the sensor
        serial.write(VALEPORT_SET_FORMAT("SB"));
        ros::Duration(2.0).sleep();
        serial.write(VALEPORT_SET_OUTPUT_RATE(get_param<int>("~output_rate")));

        // Set device name and default DEVICE packet output rates
        set_device_name("MINICT");
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
    ValeportMinictNode node(argc, argv);
    node.start();
    return 0;
}
