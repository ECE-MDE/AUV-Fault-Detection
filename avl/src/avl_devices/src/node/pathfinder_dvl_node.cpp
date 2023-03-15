//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to interface with the Teledyne Pathfinder DVL.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/dvl (avl_msgs/PathfinderDvlMsg)
//              device/velocity (geometry_msgs/Vector3)
//              device/height (std_msgs/Float64)
//
// Subscribers: None
//==============================================================================

// Core utility
#include <avl_asio/serial_port.h>
#include <avl_core/util/string.h>
#include <avl_core/util/matrix.h>

// Device node base class
#include <avl_devices/device_node.h>

// ROS message includes
#include <avl_msgs/PathfinderDvlMsg.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

// Alias for double vector
typedef std::vector<double> doubles_t;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class PathfinderDvlNode : public DeviceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        PathfinderDvlNode constructor
    //--------------------------------------------------------------------------
    PathfinderDvlNode(int argc, char **argv) : DeviceNode(argc, argv)
    {

    }

private:

    // Publishers for DVL, velocity, and height messages
    ros::Publisher dvl_pub;
    ros::Publisher velocity_pub;
    ros::Publisher height_pub;

    // Serial port instance
    SerialPort serial;

    // Most recent serial message received from the DVL
    std::string message;

    // Vehicle state variables
    Matrix3d R_s_b;
    Vector3d v_eb_s;
    Vector3d v_eb_b;
    double err_cutoff;

    // Pathfinder DVL has 4 beams from which we estimate the height
    VectorXd h_vec;

    // Janus angle of the dvl beams
    double janus_angle;

    // Other values measured by the DVL
    double salinity;    // ppt
    double temperature; // C
    double depth;       // m
    double sound_speed; // m/s

private:

    //--------------------------------------------------------------------------
    // Name:        read_handler
    // Description: Called when a DVL message is read.
    // Arguments:   - data: DVL message bytes
    //--------------------------------------------------------------------------
    void read_handler(std::vector<uint8_t> data)
    {

        // Form a string from the vector of bytes
        message = std::string(data.begin(), data.end());
        avl::strip(message, '\r');
        avl::strip(message, '\n');
        log_data("[raw] %s", message.c_str());

        try
        {

            // Data: BOTTOM-TRACK, INSTRUMENT-REFERENCED VELOCITY DATA
            if (avl::starts_with(message, ":BI"))
            {

                // Parse the message. units are mm/s. Convert to m/s
                std::vector<std::string> split_line = avl::split(message, ",");
                v_eb_s(0) = std::stod(split_line.at(1)) / 1000.0;
                v_eb_s(1) = std::stod(split_line.at(2)) / 1000.0;
                v_eb_s(2) = std::stod(split_line.at(3)) / 1000.0;
                double ve = std::stod(split_line.at(4)) / 1000.0;
                bool data_good = (split_line.at(5) == "A") &&
                                 (abs(ve) < err_cutoff);

                // Rotate sensor frame velocity to body frame
                v_eb_b = R_s_b * v_eb_s;

                // Log data
                log_data("[v_eb_s] %f %f %f %f %d",
                    v_eb_s(0), v_eb_s(1), v_eb_s(2), ve, data_good);
                log_data("[v_eb_b] %f %f %f %f %d",
                    v_eb_b(0), v_eb_b(1), v_eb_b(2), ve, data_good);

                // Publish the velocity data, NAN if no lock
                geometry_msgs::Vector3 velocity_msg;
                velocity_msg.x = data_good ? v_eb_b(0) : NAN;
                velocity_msg.y = data_good ? v_eb_b(1) : NAN;
                velocity_msg.z = data_good ? v_eb_b(2) : NAN;
                velocity_pub.publish(velocity_msg);

                avl_msgs::PathfinderDvlMsg dvl_msg;
                dvl_msg.vx = v_eb_b(0);
                dvl_msg.vy = v_eb_b(1);
                dvl_msg.vz = v_eb_b(2);
                dvl_msg.ve = ve;
                dvl_msg.valid = data_good;
                dvl_pub.publish(dvl_msg);

            }

            // Data: PRESSURE AND RANGE TO BOTTOM DATA
            if (avl::starts_with(message, ":RA"))
            {

                // Parse the message
                std::vector<std::string> split_line = avl::split(message, ",");
                double pressure = std::stod(split_line.at(1));
                double h1 = std::stod(split_line.at(2)) / 10.0;
                double h2 = std::stod(split_line.at(3)) / 10.0;
                double h3 = std::stod(split_line.at(4)) / 10.0;
                double h4 = std::stod(split_line.at(5)) / 10.0;

                // Calculate altitude from the 4 slant ranges per page 35 of the
                // Pathfinder DVL Operation Manual
                double h = (h1*h2) / (h1+h2) + (h3*h4) / (h3+h4);

                // Gather the height values for device packet updates
                h_vec(0) = h1;
                h_vec(1) = h2;
                h_vec(2) = h3;
                h_vec(3) = h4;
                h_vec(4) = h;

                // Create and publish a height message
                std_msgs::Float64 height_msg;
                height_msg.data = h;
                height_pub.publish(height_msg);

                log_data("[pressure] %f", pressure);
                log_data("[height] %f %f %f %f %f", h, h1, h2, h3, h4);

            }

            // Data: TIMING AND SCALING DATA
            if (avl::starts_with(message, ":TS"))
            {

                // Parse the message
                std::vector<std::string> split_line = avl::split(message, ",");
                salinity =       std::stod(split_line.at(2)); // ppt
                temperature =    std::stod(split_line.at(3)); // C
                depth =          std::stod(split_line.at(4)); // m
                sound_speed =    std::stod(split_line.at(5)); // m/s

                log_data("[water] %f %f %f %f",
                    salinity, temperature, depth, sound_speed);

            }

        }
        catch (const std::exception& ex)
        {
            log_warning("failed to parse message (%s) (%s)",
                ex.what(), message.c_str());
        }

    }

    //--------------------------------------------------------------------------
    // Name:        send_command
    // Description: Sends a command string to the DVL. Automatically appends
    //              a carraige return and line break. Waits for the DVL to
    //              respond to the command.
    // Arguments:   - command: command string to send
    //--------------------------------------------------------------------------
    void send_command(std::string command)
    {

        // Write the command
        serial.write(command + "\r");

        // Spin until the response is read
        ros::Rate spin_rate(1000);
        while (ros::ok() && !avl::contains(message, command))
        {
            serial.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
        }

    }

    //--------------------------------------------------------------------------
    // Name:        send_break
    // Description: Sends a break command string to the DVL, which stops
    //              pinging and allows the DVL to be configured. Waits for the
    //              DVL to respond to the command.
    //--------------------------------------------------------------------------
    void send_break()
    {

        // Write the break command
        serial.write("===\r\n");

        // Spin until the text block response to the break is read. The last
        // line contains "Current time is:"
        ros::Rate spin_rate(1000);
        while (ros::ok() && !avl::contains(message, "Current time"))
        {
            serial.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
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
        params.add(Parameter("Vx", v_eb_b(0)));
        params.add(Parameter("Vy", v_eb_b(1)));
        params.add(Parameter("Vz", v_eb_b(2)));
        params.add(Parameter("H1", h_vec(0)));
        params.add(Parameter("H2", h_vec(1)));
        params.add(Parameter("H3", h_vec(2)));
        params.add(Parameter("H4", h_vec(3)));
        params.add(Parameter("H", h_vec(4)));
        params.add(Parameter("SALINITY", salinity));
        params.add(Parameter("TEMPERATURE", temperature));
        params.add(Parameter("DEPTH", depth));
        params.add(Parameter("SOUND_SPEED", sound_speed));
        return params;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log the data headers
        add_data_header("[velocity] v_x v_y v_z data\\_good");
        add_data_header("[velocity] m/s m/s m/s bool");
        add_data_header("[velocity_inst] v_x v_y v_z v_err data\\_good");
        add_data_header("[velocity_inst] m/s m/s m/s m/s bool");
        add_data_header("[height] h h_0 h_1 h_2 h_3");
        add_data_header("[height] m m m m m");
        add_data_header("[pressure] p");
        add_data_header("[pressure] kPa");
        add_data_header("[water] salinity temp depth sound\\_speed");
        add_data_header("[water] ppt C m m/s");

        // Set up the publishers
        dvl_pub = node_handle->advertise<PathfinderDvlMsg>("device/dvl", 1);
        velocity_pub = node_handle->advertise<Vector3>("device/velocity", 1);
        height_pub = node_handle->advertise<Float64>("device/height", 1);

        // Get sensor frame to body frame rotation matrix from config file
        VectorXd R_vec = avl::from_std_vector(get_param<doubles_t>("~R_s_b"));
        R_s_b << R_vec;
        R_s_b.transposeInPlace();
        std::cout << R_s_b << std::endl;

        // Get error cutoff from config file
        err_cutoff = get_param<double>("~err_cutoff");

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));
        serial.set_match(Match("\r\n", &PathfinderDvlNode::read_handler, this));

        // Configure the Pathfinder DVL according to Table 14
        // (Recommended Commands) in the Pathfinder Operation Manual
        send_break();

        send_command("PS0");            // Query system parameters
        send_command("CR1");            // Set factory defaults
        send_command("CB811");          // Set 115200 baud
        send_command("CF11110");        // Flow control
        send_command("BP1");            // Bottom track pings once per ensemble
        send_command("BX01100");        // Maximum tracking depth to 110m
        send_command("EA+04500");       // DVL heading alignment
        send_command("ED0000");         // Fallback transducer depth

        send_command("EX10111");        // Set coordinate system
        send_command("TE00:00:00.00");  // Ping as fast as possible
        send_command("TP00:00.00");     // Ping as fast as possible

        // Set sound speed, depth, salinity, temperature sensor sources via the
        // EZ command with the following fields
        // --------------------------------------------------------
        // |Sensor      | Value = 0 | Value = 1       | Value = 2 |
        // |------------|-----------|------------------------------
        // |Sound Speed | Manual    | Calculated      | External  |
        // |Depth       | Manual    | Internal Sensor | External  |
        // |Heading     | Manual    | Internal AHRS   | External  |
        // |Pitch       | Manual    | Internal AHRS   | External  |
        // |Roll        | Manual    | N/A             | External  |
        // |Salinity    | Manual    | N/A             | External  |
        // |Temperature | Manual    | Internal Sensor | External  |
        // |Up/Down     | Manual    | N/A             | External  |
        // --------------------------------------------------------
        send_command("EZ11110010");

        // Set salinity from config file
        send_command("ES" + std::to_string(get_param<int>("~salinity")));

        send_command("WP0");            // Water profiling off
        send_command("WS0200");         // Depth cell size default
        send_command("WN030");          // Number of depth cells default

        send_command("BK0");            // Disable water-mass layer
        send_command("#BL80,160,240");  // Water-mass layer boundaries default
        send_command("#BJ100 000 000"); // Bottom data types default
        send_command("CT0");            // Turnkey off
        send_command("#EE0000010");     // Specialized environment data default
        send_command("EV00000");        // Heading variation in degrees
        send_command("#PD13");          // Set data stream structure

        send_command("CK");             // Save parameters as default
        send_command("CS");             // Start pinging

        // Set the size of the height vector to match the number of beams
        // on the pathfinder DVL + 1 for the height estimate
        h_vec.resize(5);

        // Set device name and default DEVICE packet output rates
        set_device_name("DVL");
        set_device_packet_output_rate(0, INTERFACE_FSD);
        set_device_packet_output_rate(0, INTERFACE_BSD);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    //
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
    PathfinderDvlNode node(argc, argv);
    node.start();
    return 0;
}
