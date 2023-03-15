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
#include <avl_core/util/geo.h>

// ROS message includes
#include <avl_msgs/GpsMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TrimbleGpsNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        TrimbleGpsNode constructor
    //--------------------------------------------------------------------------
    TrimbleGpsNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Serial device instance
    SerialPort serial;

    // Publishers for GPS messages
    ros::Publisher gps_pub;
    GpsMsg gps_msg;

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
                    " (expected %s)", line.c_str(), checksum.c_str());
                return;
            }

            // Split the NMEA message into its components
            std::vector<std::string> split_line = avl::split(msg, ",");

            // Handle NMEA GGA messages
            if(avl::starts_with(line, "$GPGGA"))
            {

                // Parse the GGA message data
                // std::string msg_id =           split_line.at(0);
                // std::string utc_time =         split_line.at(1);
                // std::string latitude =         split_line.at(2);
                // std::string ns_ind =           split_line.at(3);
                // std::string longitude =        split_line.at(4);
                // std::string ew_ind =           split_line.at(5);
                std::string fix_quality =      split_line.at(6);
                std::string num_sats =         split_line.at(7);
                std::string hdop =             split_line.at(8);
                std::string msl_alt =          split_line.at(9);
                // std::string msl_units =        split_line.at(10);
                // std::string geoid_sep =        split_line.at(11);
                // std::string geoid_sep_units =  split_line.at(12);
                // std::string diff_corr_age =    split_line.at(13);
                // std::string diff_ref_station = split_line.at(14);

                // Parse the data into the GPS meassage. If the strings are
                // empty, there is no lock
                gps_msg.alt =         msl_alt.empty() ? NAN   : std::stod(msl_alt);
                gps_msg.hdop =        hdop.empty() ?    0     : std::stoi(hdop);
                gps_msg.fix_quality = fix_quality.empty() ? 0 : std::stoi(fix_quality);
                gps_msg.num_sats =    num_sats.empty() ?    0 : std::stoi(num_sats);


            }

            // Handle NMEA GGA messages
            if(avl::starts_with(line, "$GPGST"))
            {

                // Parse the GGA message data
                // std::string msg_id =           split_line.at(0);
                // std::string utc_time =         split_line.at(1);
                // std::string pseudorange_rms =  split_line.at(2);
                // std::string err_ell_maj_axis = split_line.at(3);
                // std::string err_ell_min_axis = split_line.at(4);
                // std::string err_ell_heading =  split_line.at(5);
                std::string stddev_lat_str = split_line.at(6);
                std::string stddev_lon_str = split_line.at(7);
                std::string stddev_alt_str = split_line.at(8);
                gps_msg.stddev_lat = stddev_lat_str.empty()? NAN : std::stod(stddev_lat_str);
                gps_msg.stddev_lon = stddev_lon_str.empty()? NAN : std::stod(stddev_lon_str);
                gps_msg.stddev_alt = stddev_alt_str.empty()? NAN : std::stod(stddev_alt_str);

                log_data("[error] %.4f %.4f %.4f",
                    gps_msg.stddev_lat,
                    gps_msg.stddev_lon,
                    gps_msg.stddev_alt);

            }

            // Handle NMEA RMC messages
            if(avl::starts_with(line, "$GPRMC"))
            {

                // Parse the RMC message data
                // std::string msg_id =       split_line.at(0);
                // std::string utc_time =     split_line.at(1);
                std::string status =       split_line.at(2);
                std::string latitude =     split_line.at(3);
                std::string ns_ind =       split_line.at(4);
                std::string longitude =    split_line.at(5);
                std::string ew_ind =       split_line.at(6);
                std::string ground_speed = split_line.at(7);  // kts
                std::string track_angle =  split_line.at(8);  // deg
                // std::string date =         split_line.at(9);
                // std::string mag_var =      split_line.at(10); // deg
                // std::string mag_var_dir =  split_line.at(11);

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
        log_data("[error] err\\_ell\\_maj\\_axis err\\_ell\\_min\\_axis err\\_ell\\_heading err\\_lat err\\_lon err\\_alt");
        log_data("[error] m m deg m m m");

        // Set up the publisher for GPS data
        gps_pub = node_handle->advertise<GpsMsg>("device/gps", 1);

        // Open the serial port
        // serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set the match condition to a newline for NMEA strings
        serial.set_match(Match("\r\n", &TrimbleGpsNode::nmea_read_handler, this));

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
    TrimbleGpsNode node(argc, argv);
    node.start();
    return 0;
}
