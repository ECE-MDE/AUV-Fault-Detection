//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to interface with the LinkQuest NavQuest 600 Micro DVL.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/dvl (avl_msgs/LinkquestDvlMsg)
//              device/velocity (geometry_msgs/Vector3)
//              device/height (std_msgs/Float64)
//
// Subscribers: None
//==============================================================================

// Core utility
#include <avl_core/node.h>
#include <avl_asio/serial_port.h>
#include <avl_core/util/string.h>

// ROS message includes
#include <avl_msgs/LinkquestDvlMsg.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// Linkquest DVL command protocol
#include <avl_devices/protocol/linkquest_dvl.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class LinkquestDvlNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        LinkquestDvlNode constructor
    //--------------------------------------------------------------------------
    LinkquestDvlNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publishers for DVL, velocity, and height messages
    ros::Publisher dvl_pub;
    ros::Publisher velocity_pub;
    ros::Publisher height_pub;

    // Serial port instance
    SerialPort serial;

private:

    void write_command(std::string command)
    {
        serial.write(command);
        ros::Duration(1.0).sleep();
        serial.spin_once();
        ros::spinOnce();
    }

    //--------------------------------------------------------------------------
    // Name:        read_handler
    // Description: Called when a DVL message is read.
    // Arguments:   - data: DVL message bytes
    //--------------------------------------------------------------------------
    void read_handler(std::vector<uint8_t> data)
    {

        // Form a string from the vector of bytes
        std::string message(data.begin(), data.end());
        avl::strip(message, '\r');
        avl::strip(message, '\n');
        log_data("[raw] %s", message.c_str());

        // If the line is an NQ1 DVL message
        if(avl::starts_with(message, "$#NQ.RES"))
        {

            // Parse the NQ1 message into a struct
            DvlNQ1Data nq1 = parse_nq1_message(message);

            // Create and publish the DVL message from the NQ1 struct
            LinkquestDvlMsg dvl_msg;
            dvl_msg.linear_velocity.x = nq1.velo_instrument[0];
            dvl_msg.linear_velocity.y = nq1.velo_instrument[1];
            dvl_msg.linear_velocity.z = nq1.velo_instrument[2];
            dvl_msg.water_current_velocity.x = nq1.water_velo_instrument[0];
            dvl_msg.water_current_velocity.y = nq1.water_velo_instrument[1];
            dvl_msg.water_current_velocity.z = nq1.water_velo_instrument[2];
            dvl_msg.beam_altitude.push_back(nq1.v_altitude[0]);
            dvl_msg.beam_altitude.push_back(nq1.v_altitude[1]);
            dvl_msg.beam_altitude.push_back(nq1.v_altitude[2]);
            dvl_msg.beam_altitude.push_back(nq1.v_altitude[3]);
            dvl_msg.linear_velocity_flag = nq1.velo_instrument_flag;
            dvl_msg.water_current_velocity_flag = nq1.water_velo_instrument_flag;
            dvl_msg.beam_flag.push_back(nq1.good_or_bad[0]);
            dvl_msg.beam_flag.push_back(nq1.good_or_bad[1]);
            dvl_msg.beam_flag.push_back(nq1.good_or_bad[2]);
            dvl_msg.beam_flag.push_back(nq1.good_or_bad[3]);

            // Only publish the DVL measurement if it is a valid bottom lock
            if (nq1.velo_instrument_flag == 1)
                dvl_pub.publish(dvl_msg);

            // Create and publish a velocity message. Units must be converted
            // from the DVL's mm/s to m/s
            geometry_msgs::Vector3 velocity_msg;
            velocity_msg.x = nq1.velo_instrument[0] / 1000.0;
            velocity_msg.y = nq1.velo_instrument[1] / 1000.0;
            velocity_msg.z = nq1.velo_instrument[2] / 1000.0;
            velocity_pub.publish(velocity_msg);
            log_data("[velocity] %.2f %.2f %.2f",
                velocity_msg.x,
                velocity_msg.y,
                velocity_msg.z);

            // Create and publish a height message
            std_msgs::Float64 height_msg;
            height_msg.data = nq1.altitude_estimate;
            height_pub.publish(height_msg);
            log_data("[height] %f %f %f %f %f",
                nq1.altitude_estimate,
                nq1.v_altitude[0],
                nq1.v_altitude[1],
                nq1.v_altitude[2],
                nq1.v_altitude[3]);

        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    //
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Read te DVL settings from the config file so that they can be
        // verified
        double output_period      = get_param<double>("~output_period");
        int speed_of_sound        = get_param<int>("~speed_of_sound");
        bool enable_trigger_out   = get_param<bool>("~enable_trigger_out");
        int working_mode          = get_param<int>("~working_mode");

        // Verify the DVL parameters

        // DVL trigger out and trigger in cannot be enabled at the same time
        if(enable_trigger_out && (working_mode == 2))
            throw std::runtime_error("trigger out and trigger in cannot both be enabled");

        // The output period must be greater than 0.01 seconds
        if(output_period < 0.01)
            throw std::runtime_error("output period must be greater than 0.01 seconds");

        // Set up the publishers
        dvl_pub = node_handle->advertise<LinkquestDvlMsg>("device/dvl", 1);
        velocity_pub = node_handle->advertise<geometry_msgs::Vector3>("device/velocity", 1);
        height_pub = node_handle->advertise<std_msgs::Float64>("device/height", 1);

        // Log the data headers
        log_data("[velocity] v_x v_y v_z");
        log_data("[velocity] m/s m/s m/s");
        log_data("[altitude] h h_0 h_1 h_2 h_3");
        log_data("[altitude] m m m m m");
        log_data("[raw] NQ1");
        log_data("[raw] N/A");

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set the match condition to \r\n for NQ1 message strings
        serial.set_match(Match("\r\n", &LinkquestDvlNode::read_handler, this));

        // Configure the DVL
        log_info("configuring the LinkQuest DVL...");

        // Put the DVL in configuration mode
        log_info("putting DVL in configuration mode");
        write_command(DVL_NQ_STOP());

        // Pause to allow the DVL to enter configuration mode
        ros::Duration(3.0).sleep();

        // Set the working mode
        log_info("setting working mode to %d", working_mode);
        write_command(DVL_SET_WORKING_MODE(working_mode));

        // Set the output period
        log_info("setting output period to %.2f", output_period);
        write_command(DVL_SET_OUTPUT_PERIOD(output_period));

        // Set the trigger out option
        log_info("setting trigger out to %d", (int)enable_trigger_out);
        write_command(DVL_SET_TRIGGER_OUT(enable_trigger_out));

        // Disable all output formats
        log_info("disabling output formats");
        write_command(DVL_RESET_OUTPUT_FORMAT(0));

        // enable only the NQ1 output format
        log_info("enabling NQ1 output format");
        write_command(DVL_SET_OUTPUT_FORMAT(1));

        // Set the speed of sound
        log_info("setting speed of sound to %d", speed_of_sound);
        write_command(DVL_SET_SPEED_OF_SOUND(speed_of_sound));

        // Enable the user-provided speed of sound
        log_info("setting DVL to use user-provided speed of sound");
        write_command(DVL_SET_ENABLE_USER_SOUND_VELOCITY(true));

        // Put the DVL back into measurement mode
        log_info("putting DVL in measurement mode");
        write_command(DVL_NQ_START());

        log_info("configuration complete");

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

    //--------------------------------------------------------------------------
    // Name:        shutdown
    //
    // Description: Called after the run function when the node is started.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        // Stop DVL output stream on shutdown
        write_command(DVL_NQ_STOP());
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    LinkquestDvlNode node(argc, argv);
    node.start();
    return 0;
}
