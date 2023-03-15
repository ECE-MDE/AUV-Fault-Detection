//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to publish GPS data from a serial GPS device that
//              provides GPS data in the NMEA format. If the GPS module has no
//              lock, the GPS message is published with zeros in every field.
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

class GpsNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        GpsNode constructor
    //--------------------------------------------------------------------------
    GpsNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Serial device instance
    SerialPort serial;

    // Publishers for GPS messages
    ros::Publisher gps_pub;

private:

    //--------------------------------------------------------------------------
    // Name:        write_nmea_command
    // Description: Writes an NMEA formatted command to the GPS receiver after
    //              appending a checksum.
    // Arguments:   - command: NMEA command to be written
    //--------------------------------------------------------------------------
    void write_nmea_command(std::string command)
    {
        std::string nmea_msg;
        nmea_msg = command + avl::get_nmea_checksum(command) + "\r\n";
        serial.write(nmea_msg);
        avl::strip(nmea_msg, '\r');
        avl::strip(nmea_msg, '\n');
        log_data("[tx] %s", nmea_msg.c_str());
    }

    //--------------------------------------------------------------------------
    // Name:        write_sirf_command
    // Description: Sends a SiRF command constructed from the given payload.
    //--------------------------------------------------------------------------
    void write_sirf_command(std::vector<uint8_t> payload)
    {

        // Start the SiRF message with the start sequence bytes
        std::vector<uint8_t> sirf_message = {0xA0, 0xA2};

        // Add the SiRF payload length bytes
        uint16_t payload_length = payload.size() & 0x7FFF;
        uint8_t payload_length_lsb = payload_length & 0xFF;
        uint8_t payload_length_msb = (payload_length >> 8) & 0xFF;
        sirf_message.push_back(payload_length_msb);
        sirf_message.push_back(payload_length_lsb);

        // Add the payload bytes
        avl::append(sirf_message, payload);

        // Add the SiRF checksum bytes
        uint16_t checksum = 0x00;
        for (uint8_t byte : payload)
        {
            checksum = checksum + byte;
            checksum = checksum & 0x7FFF;
        }
        uint8_t checksum_lsb = checksum & 0xFF;
        uint8_t checksum_msb = (checksum >> 8) & 0xFF;
        sirf_message.push_back(checksum_msb);
        sirf_message.push_back(checksum_lsb);

        // Add the stop sequence bytes
        avl::append(sirf_message, {0xB0, 0xB3});

        // Write the message
        serial.write(sirf_message);
        log_data("[tx] %s", avl::byte_to_hex(sirf_message).c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        sirf_read_handler
    // Description: Handler for SiRF GPS messages
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void sirf_read_handler(std::vector<uint8_t> data)
    {
        log_data("[sirf] " + avl::byte_to_hex(data));
    }

    //--------------------------------------------------------------------------
    // Name:        nmea_read_handler
    // Description: Handler for NMEA GPS messages
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void nmea_read_handler(std::vector<uint8_t> data)
    {

        // Format the data as a string, and remove any \r\n characters
        std::string line = std::string(data.begin(), data.end());
        avl::strip(line, '\r');
        avl::strip(line, '\n');
        log_data("[nmea] "+ line);

        try
        {

            // If the NMEA message is the GGA type, parse the fix data. For now
            // we aren't interested in the other message types
            if(avl::starts_with(line, "$GPGGA"))
            {

                // Split the comma delimited NMEA message
                std::vector<std::string> split_line = avl::split(line, ",");

                // GPS message to be published
                GpsMsg gps_msg;

                // Parse the NMEA message data and form the GPS data message

                // Latitude is strings 2 and 3 given as DMS and N or S
                // If they are empty, there is no lock and should be NaN
                if (!split_line[2].empty() && !split_line[3].empty())
                    gps_msg.lat = avl::dms_to_rad(split_line[2], split_line[3]);
                else
                    gps_msg.lat = NAN;

                // Longitude is strings 4 and 5 given as DMS and E or W
                // If they are empty, there is no lock and should be NaN
                if (!split_line[4].empty() && !split_line[5].empty())
                    gps_msg.lon = avl::dms_to_rad(split_line[4], split_line[5]);
                else
                    gps_msg.lon = NAN;

                // Fix quality is string 6
                gps_msg.fix_quality = std::stoi(split_line[6]);

                // Number of satellites in use is string 7
                gps_msg.num_sats = std::stoi(split_line[7]);

                // Altitude is string 9 and given as meters
                // If it is empty, there is no lock and should be NaN
                if (!split_line[9].empty())
                    gps_msg.alt = std::stoi(split_line[9]);
                else
                    gps_msg.alt = NAN;

                // Publish the GPS message
                gps_pub.publish(gps_msg);

                log_data("[gps] %.9f %.9f %.1f %d %d",
                    gps_msg.lat,
                    gps_msg.lon,
                    gps_msg.alt,
                    static_cast<int>(gps_msg.fix_quality),
                    static_cast<int>(gps_msg.num_sats));

            }

        }
        catch (const std::exception& ex)
        {
            log_warning("ignoring invalid NMEA line (%s)", line.c_str());
        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log data headers
        log_data("[gps] lat lon alt quality sats");
        log_data("[gps] deg deg m type num");

        // Set up the publisher for GPS data
        gps_pub = node_handle->advertise<GpsMsg>("device/gps", 1);

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set the match condition to a newline for NMEA strings
        serial.set_match(Match({0xB0, 0xB3}, &GpsNode::sirf_read_handler, this));

        // Configure the GPS receiver

        // Switch to SiRF mode
        log_info("switching to SiRF mode");
        write_nmea_command("$PSRF100,0,9600,8,1,0");

        ros::Duration(2.0).sleep();

        // Disable static navigation
        write_sirf_command({0x8F, 0x00});

        ros::Duration(1.0).sleep();

        // Set mode
        write_sirf_command({0x88, // Message ID
                            0x00, 0x00, // Reserved
                            0x04, // Degraded mode disable
                            0x00, 0x00, // Reserved
                            0x00, 0x00, // User-specified altitude
                            0x02, // Altitude hold mode disabled
                            0x00, // Altitude hold source last computed
                            0x00, // Reserved
                            0x00, // Degreded time out disable
                            0x00, // Dead reckoning time out disable
                            0x00  // Track smoothing disable
                           });

       // Switch back to NMEA mode
       log_info("switching to NMEA mode");
       ros::Duration(1.0).sleep();
       write_sirf_command({0x81, // Message ID
                           0x00, // Mode
                           0x01, // GGA Message
                           0x01, // Checksum
                           0x00, // GLL Message
                           0x01, // Checksum
                           0x00, // GSA Message
                           0x01, // Checksum
                           0x00, // GSV Message
                           0x01, // Checksum
                           0x00, // RMC Message
                           0x01, // Checksum
                           0x00, // VTG Message
                           0x01, // Checksum
                           0x00, // MSS Message
                           0x01, // Checksum
                           0x00, // Unused Field
                           0x00, // ZDA Message
                           0x00, // Unused Field
                           0x00, // Unused Field
                           0x00, // Unused Field
                           0x00, // Unused Field
                           0x25, 0x80 // 9600 baud
                          });
        ros::Duration(1.0).sleep();

        // Set the match condition to a newline for NMEA strings
        serial.set_match(Match("\n", &GpsNode::nmea_read_handler, this));

        // Set message rates
        write_nmea_command("$PSRF105,1");
        write_nmea_command("$PSRF103,00,00,01,01"); // GGA
        write_nmea_command("$PSRF103,01,00,00,01"); // GLL
        write_nmea_command("$PSRF103,02,00,00,01"); // GSA
        write_nmea_command("$PSRF103,03,00,00,01"); // GSV
        write_nmea_command("$PSRF103,04,00,05,01"); // RMC
        write_nmea_command("$PSRF103,05,00,00,01"); // VTG
        write_nmea_command("$PSRF103,06,00,00,01"); // MSS
        write_nmea_command("$PSRF103,07,00,00,01"); // Not defined
        write_nmea_command("$PSRF103,08,00,05,01"); // ZDA
        write_nmea_command("$PSRF103,09,00,00,01"); // Not defined

        // Enable the rate divider, dividing message rates by 5
        write_nmea_command("$PSRF103,0,6,1,0");

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
    GpsNode node(argc, argv);
    node.start();
    return 0;
}
