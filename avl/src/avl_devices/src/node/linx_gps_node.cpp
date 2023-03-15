//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to publish GPS data from a Linx GPS device that
//              provides GPS data in the NMEA format.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/gps (avl_msgs/GpsMsg)
//
// Subscribers: None
//==============================================================================

// Core utility
#include <avl_core/node.h>
#include <avl_asio/serial_port.h>
#include <avl_core/util/string.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>
#include <avl_core/util/geo.h>

// ROS message includes
#include <avl_msgs/GpsMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class LinxGpsNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        LinxGpsNode constructor
    //--------------------------------------------------------------------------
    LinxGpsNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Serial device instance
    SerialPort serial;

    // Publishers for GPS messages
    ros::Publisher gps_pub;
    GpsMsg gps_msg;

    // Timer used to wait for an acknowledge response to commands
    ros::Timer response_timer;
    ros::Duration response_timer_duration;
    std::string latest_message;

    // Mapping for GPS receiver states
    std::vector<std::string> state_map = {
        "unknown",
        "start-up",
        "notification for host supporting EPO",
        "successful transition to normal operation"};

    // Mapping for command responses
    std::vector<std::string> flag_map = {
        "invalid command",
        "unsupported command",
        "valid command but action failed",
        "valid command and action succeeded"};

private:

    //--------------------------------------------------------------------------
    // Name:        nmea_read_handler
    // Description: Handler for NMEA GPS messages
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void nmea_read_handler(std::vector<uint8_t> data)
    {

        // If it doesn't start with a $ it's not a valid NMEA message
        if (data.at(0) != '$')
            return;

        // Format the data as a string, and remove any \r\n characters
        std::string line = std::string(data.begin(), data.end());
        avl::strip(line, '\r');
        avl::strip(line, '\n');
        latest_message = line;

        // Log the received message
        log_data("[serial] rx " + line);

        // Parse the NMEA message
        try
        {

            // Check the NMEA message checksum
            size_t checksum_pos = line.find("*");
            std::string msg = line.substr(0, checksum_pos);
            std::string checksum = line.substr(checksum_pos);
            std::string expected_checksum = avl::get_nmea_checksum(msg);
            if (checksum != expected_checksum)
            {
                log_warning("ignoring NMEA message (%s) with invalid checksum"
                    " (got %s expected %s)",
                    line.c_str(), checksum.c_str(), expected_checksum.c_str());
                return;
            }

            // Split the NMEA message into its components
            std::vector<std::string> split_line = avl::split(line, ",");

            // If the NMEA message is the start-up response
            if(avl::starts_with(line, "$PMTK010"))
            {
                int state = std::stoi(split_line.at(1));
                log_info("received receiver state message: %s",
                    state_map.at(state).c_str());
            }

            // Handle NMEA GGA messages
            if(avl::starts_with(line, "$GPGGA") ||
               avl::starts_with(line, "$GNGGA"))
            {

                // Parse the GGA message data
                std::string msg_id =           split_line.at(0);
                std::string utc_time =         split_line.at(1);
                std::string latitude =         split_line.at(2);
                std::string ns_ind =           split_line.at(3);
                std::string longitude =        split_line.at(4);
                std::string ew_ind =           split_line.at(5);
                std::string fix_quality =      split_line.at(6);
                std::string num_sats =         split_line.at(7);
                std::string hdop =             split_line.at(8);
                std::string msl_alt =          split_line.at(9);
                std::string msl_units =        split_line.at(10);
                std::string geoid_sep =        split_line.at(11);
                std::string geoid_sep_units =  split_line.at(12);
                std::string diff_corr_age =    split_line.at(13);
                std::string diff_ref_station = split_line.at(14);

                // Parse the data into the GPS meassage. If the strings are
                // empty, there is no lock
                gps_msg.alt =         msl_alt.empty() ? NAN : std::stoi(msl_alt);
                gps_msg.hdop =        hdop.empty() ?    0   : std::stoi(hdop);
                gps_msg.fix_quality = std::stoi(fix_quality);
                gps_msg.num_sats =    std::stoi(num_sats);

            }

            // Handle NMEA RMC messages
            if(avl::starts_with(line, "$GPRMC") ||
               avl::starts_with(line, "$GNRMC"))
            {

                // Parse the RMC message data
                std::string msg_id =       split_line.at(0);
                std::string utc_time =     split_line.at(1);
                std::string status =       split_line.at(2);
                std::string latitude =     split_line.at(3);
                std::string ns_ind =       split_line.at(4);
                std::string longitude =    split_line.at(5);
                std::string ew_ind =       split_line.at(6);
                std::string ground_speed = split_line.at(7);  // kts
                std::string track_angle =  split_line.at(8);  // deg
                std::string date =         split_line.at(9);
                std::string mag_var =      split_line.at(10); // deg
                std::string mag_var_dir =  split_line.at(11);

                // Parse the data into the GPS meassage. If the strings are
                // empty, there is no lock
                gps_msg.lat =  latitude.empty() ?     NAN : avl::dms_to_rad(latitude, ns_ind);
                gps_msg.lon =  latitude.empty() ?     NAN : avl::dms_to_rad(longitude, ew_ind);
                gps_msg.ground_speed = ground_speed.empty() ? NAN : 0.5144444 * std::stod(ground_speed);
                gps_msg.track_angle = track_angle.empty() ?  NAN : avl::deg_to_rad(std::stod(track_angle));

                // Publish and log the GPS message
                gps_pub.publish(gps_msg);

                log_data("[gps] %.9f %.9f %.1f %.4f %.4f %d %d %d",
                    gps_msg.lat,
                    gps_msg.lon,
                    gps_msg.alt,
                    gps_msg.ground_speed,
                    gps_msg.track_angle,
                    gps_msg.fix_quality,
                    gps_msg.num_sats,
                    gps_msg.hdop);

            }

        }
        catch (const std::exception& ex)
        {
            log_warning("ignoring invalid NMEA line (%s) (%s)",
                line.c_str(), ex.what());
        }

    }

    //--------------------------------------------------------------------------
    // Name:        write_nmea_command
    // Description: Writes an NMEA formatted command to the GPS receiver after
    //              appending a checksum.
    // Arguments:   - command: NMEA command to be written
    //--------------------------------------------------------------------------
    void write_nmea_command(std::string command)
    {

        // Format and write the command
        std::string nmea_msg;
        nmea_msg = command + avl::get_nmea_checksum(command) + "\r\n";
        serial.write(nmea_msg);
        avl::strip(nmea_msg, '\r');
        avl::strip(nmea_msg, '\n');

        log_data("[serial] tx %s", nmea_msg.c_str());

        // Wait for the acknowledgement response
        std::string response = wait_for_ack(command);

        // Split the comma delimited NMEA message
        std::vector<std::string> split_line = avl::split(response, ",");
        int cmd_id = std::stoi(split_line.at(1));
        int flag = std::stoi(split_line.at(2));

        if (flag != 3)
            log_error("received acknowledge to command %d: %s",
                cmd_id, flag_map.at(flag).c_str());
        else
            log_info("received acknowledge to command %d: %s",
                cmd_id, flag_map.at(flag).c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        wait_for_ack
    // Description: Waits until an acknowledge is received for a specified
    //              command.
    // Arguments:   - command: Command to wait for acknowledgement of.
    // Returns:     Acknowledgement string.
    //--------------------------------------------------------------------------
    std::string wait_for_ack(std::string command)
    {

        std::string cmd_id = command.substr(5,3);
        std::string ack_str = "$PMTK001," + cmd_id;

        // Start the response timer
        response_timer.start();

        //Spin wile waiting for the string to enter the rx buffer
        while (ros::ok() && !avl::starts_with(latest_message, ack_str))
        {
            serial.spin_once();
            ros::spinOnce();
        };

        // If we have left the loop, the string was received

        // Reset the timer by setting a new duration
        response_timer.setPeriod(response_timer_duration, true);
        response_timer.stop();

        return latest_message;

    }

    //--------------------------------------------------------------------------
    // Name:        response_timer_callback
    // Description: Called when the response timer expires.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void response_timer_callback(const ros::TimerEvent& event)
    {
        throw std::runtime_error("timed out waiting for receiver response");
    }

    //--------------------------------------------------------------------------
    // Name:        send_config
    // Description: Configures the GPS module by sending the configuration
    //              commands.
    //--------------------------------------------------------------------------
    void send_config()
    {

        log_info("configuring GPS receiver...");

        // Write an initial command to wake up the receiver. This seems to be
        // required when the receiver is power cycled...
        serial.write("$PMTK353,1,0,0,0,0*2A\r\n");
        ros::Duration(1.0).sleep();

        // Configure GNSS systems. If GLONASS is used, RMC message will be
        // labeled GNRMC instead of GPRMC
        log_info("configuring GNSS systems...");
        write_nmea_command("$PMTK353,1,0,0,0,0");

        // Configure message rates by setting the number of position fixes
        // between each message (see position fix interval). Order is:
        //     GLL, RMC, VTG, GGA, GSA, GSV, 0,0,0,0,0,0,0,0,0,0,0, ZDA, 0
        log_info("configuring message rates...");
        std::ostringstream ss;
        ss << "$PMTK314,"
           << get_param<int>("~gll_div") << "," // GLL
           << get_param<int>("~rmc_div") << "," // RMC
           << get_param<int>("~vtg_div") << "," // VTG
           << get_param<int>("~gga_div") << "," // GGA
           << get_param<int>("~gsa_div") << "," // GSA
           << get_param<int>("~gsv_div") << "," // GSV
           << "0,0,0,0,0,0,0,0,0,0,0,"          // Reserved
           << get_param<int>("~zda_div") << "," // ZDA
           << "0";                              // Reserved
        write_nmea_command(ss.str());

        // Set the position fix interval
        log_info("setting position fix interval...");
        write_nmea_command("$PMTK220," +
            std::to_string(get_param<int>("~fix_interval")));

        // Configure 1PPS to always output with 100ms width
        log_info("configuring 1PPS output...");
        write_nmea_command("$PMTK285,4,100");

        // Enable 1PPS and NMEA message sync
        log_info("enabling 1PPS and NMEA message sync...");
        write_nmea_command("$PMTK255,1");

        // Disable static navigation threshold to allow a proper position
        // report when moving slowly
        log_info("disabling static navigation...");
        write_nmea_command("$PMTK386,0");

        // Enable SBAS
        log_info("enabling SBAS...");
        write_nmea_command("$PMTK313,1");

        log_info("GPS receiver configuration complete");

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log data headers
        add_data_header("[serial] dir msg");
        add_data_header("[serial] dir msg");
        add_data_header("[gps] lat lon alt ground\\_speed track\\_angle fix\\_quality num\\_sats hdop");
        add_data_header("[gps] rad rad m m/s rad NA num NA");

        // Set up the response timer as a one-shot timer. Creating the timer
        // also starts it, so stop it.
        response_timer_duration = ros::Duration(
            get_param<double>("~serial/read_timeout"));
        response_timer = node_handle->createTimer(response_timer_duration,
            &LinxGpsNode::response_timer_callback, this, true);
        response_timer.stop();

        // Set up the publisher for GPS data
        gps_pub = node_handle->advertise<GpsMsg>("device/gps", 1);

        // Open the serial port
        // serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set the match condition to a newline for NMEA strings
        serial.set_match(Match("\r\n", &LinxGpsNode::nmea_read_handler, this));

        // Send the GPS module configuration commands
        send_config();

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
    LinxGpsNode node(argc, argv);
    node.start();
    return 0;
}
