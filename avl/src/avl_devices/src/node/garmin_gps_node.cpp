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
//		device/nmea (std_msgs/String)
//
// Subscribers: None
//==============================================================================

// Core utility

#include <avl_asio/udp_socket.h>
#include <avl_core/util/string.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>
#include <avl_core/util/geo.h>

// Node base class
#include <avl_devices/device_node.h>

// ROS message includes
#include <avl_msgs/GpsMsg.h>
#include <std_msgs/String.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class GarminGpsNode : public DeviceNode
{

public:
    //--------------------------------------------------------------------------
    // Name:        GarminGpsNode constructor
    //--------------------------------------------------------------------------
    GarminGpsNode(int argc, char **argv) : DeviceNode(argc, argv) {}

private:
    // Serial device instance
    UdpSocket udp;

    // Publishers for GPS messages
    ros::Publisher gps_pub;
    ros::Publisher nmea_pub;

    double gps_age_device;
    double gps_lon_device;
    double gps_lat_device;

private:

    //--------------------------------------------------------------------------
    // Name:        nmea_read_handler
    // Description: Handler for NMEA GPS messages
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void nmea_read_handler(std::vector<uint8_t> data)
    {
        // Format the data as a string, and remove any \r\n characters
        std::string line = std::string(data.begin(), data.end());

        //publish the string to the nmea topic
        if ((data[0] == '$') && (data[1] == 'G'))
        {
            std_msgs::String msg;
            msg.data = line;
            // Publish the nmea message
            nmea_pub.publish(msg);
        }

        avl::strip(line, '\r');
        avl::strip(line, '\n');

        try
        {

            // If the NMEA message is the GGA type, parse the fix data. For now
            // we aren't interested in the other message types
            if (avl::contains(line, "GGA"))
            {

                // Split the comma delimited NMEA message
                std::vector<std::string> split_line = avl::split(line, ",");

                // GPS message to be published
                GpsMsg gps_msg;

                // Parse the NMEA message data and form the GPS data message

                // Latitude is strings 2 and 3 given as DMS and N or S
                // If they are empty, there is no lock and should be NaN
                if (!split_line[2].empty() && !split_line[3].empty())
                    gps_msg.lat = avl::dms_to_deg(split_line[2], split_line[3]);
                else
                    gps_msg.lat = NAN;

                // Longitude is strings 4 and 5 given as DMS and E or W
                // If they are empty, there is no lock and should be NaN
                if (!split_line[4].empty() && !split_line[5].empty())
                    gps_msg.lon = avl::dms_to_deg(split_line[4], split_line[5]);
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

                gps_lat_device = gps_msg.lat;
                gps_lon_device = gps_msg.lon;
                gps_age_device = 0;
            }
        }
        catch (const std::exception &ex)
        {
            log_warning("ignoring invalid NMEA line (%s)", line.c_str());
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
        params.add(Parameter("LAT", gps_lat_device));
        params.add(Parameter("LONG", gps_lon_device));
        params.add(Parameter("AGE", gps_age_device));
        gps_age_device += 1;
        return params;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        try
        {
            udp.set_read_timeout(get_param<int>("~udp/read_timeout"));
            int udp_port = get_param<int>("~udp/port");
            log_info("Attempting to connect to UDP port:" + std::to_string(udp_port));
            // Open the UDP port
            udp.open(udp_port);

            // Set the match condition to a newline
            udp.set_match(Match("\n", &GarminGpsNode::nmea_read_handler, this));

            log_info("UDP Connection successful");
        }
        catch (const std::exception &ex)
        {
            log_warning("UDP Connection failed");
            return;
            //TODO probably try to close the port (if it's already open)
        }

        // Log data headers
        log_data("[gps] lat lon alt quality sats");
        log_data("[gps] deg deg m type num");

        // Set up the publisher for GPS data
        gps_pub = node_handle->advertise<GpsMsg>("device/gps", 1);
        nmea_pub = node_handle->advertise<std_msgs::String>("device/nmea", 1);

        // Set device name and default DEVICE packet output rates
        set_device_name("TOPSIDE_GPS");
        set_device_packet_output_rate(1.0, INTERFACE_FSD);
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
            udp.spin_once();
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
    GarminGpsNode node(argc, argv);
    node.start();
    return 0;
}
