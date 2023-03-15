//==============================================================================
// Autonomous Vehicle Library
//
// Description: Interfaces with a computer running PingDSP's 3DSS control
//              software for control of a PingDSP 3D sidescan sonar. Provides
//              services to enable/disable the sonar and start/stop recoring.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================

// PCL is a really good library with a lot of great tools available. We should
// probably try to use it as effectively as possible here.

// Node base class
#include <avl_core/node.h>

// TCP socket class
#include <avl_asio/tcp_socket.h>

// Util functions
#include <avl_core/util/string.h>
#include <avl_core/util/geo.h>
#include <avl_core/util/matrix.h>

// PingDSP sonar command protocol
#include <avl_devices/protocol/pingdsp_sonar.h>
#include <avl_devices/protocol/pingdsp-3dss.h>
using namespace softsonar;

// ROS messages

#include <pcl_ros/point_cloud.h>

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;


//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TempSonarNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        TempSonarNode constructor
    //--------------------------------------------------------------------------
    TempSonarNode(int argc, char **argv) : Node(argc, argv), socket(16777216)
    {

    }

private:

    // Pointcloud publisher
    ros::Publisher pcl_pub;

    // Reference point for ned conversions
    double lat0, lon0, alt0;
    bool first = true;

    // TCP socket for connection to computer running 3DSS sonar control software
    TcpSocket socket;

    // PingDSP data structure to represent all data from sonar
    dx::DxData* ping_data;

    // Vehicle state associated with the most recent ping
    double roll; // radians
    double pitch; // radians
    double yaw; // radians
    double lat; // radians
    double lon; // radians
    double alt; // meters from mean sea level

    // Sonar settings information from the most recent ping
    double port_transducer_angle; // radians (positive downward)
    double stbd_transducer_angle; // radians (positive downward)

    // Port sonar data in geo coordinates from the most recent ping
    std::vector<GeoSonarPoint> port_sonar_data;

    // Starboatd sonar data in geo coordinates from the most recent ping
    std::vector<GeoSonarPoint> stbd_sonar_data;

private:

    //--------------------------------------------------------------------------
    // Name:        tcp_data_header_callback
    // Description: Called when a TCP message is read.
    // Arguments:   - data: message bytes
    //--------------------------------------------------------------------------
    void tcp_data_header_callback(std::vector<uint8_t> data)
    {

        // Set the match for the uint32 data size indicator bytes
        socket.set_match(Match(4, &TempSonarNode::tcp_data_size_callback, this));

    }

    //--------------------------------------------------------------------------
    // Name:        tcp_data_size_callback
    // Description: Called when a TCP message is read.
    // Arguments:   - data: message bytes
    //--------------------------------------------------------------------------
    void tcp_data_size_callback(std::vector<uint8_t> data)
    {

        int data_packet_size = from_bytes<uint32_t>(data, true);
        socket.set_match(Match(data_packet_size, &TempSonarNode::tcp_data_callback, this));

    }

    //--------------------------------------------------------------------------
    // Name:        tcp_data_callback
    // Description: Called when a TCP message is read.
    // Arguments:   - data: message bytes
    //--------------------------------------------------------------------------
    void tcp_data_callback(std::vector<uint8_t> data)
    {

        // Cast the data to the PingDSP defined object
        ping_data = reinterpret_cast<dx::DxData*>(data.data());

        // Get sonar settings for this ping
        port_transducer_angle = avl::deg_to_rad(ping_data->system_info.port_transducer_angle);
        stbd_transducer_angle = avl::deg_to_rad(ping_data->system_info.starboard_transducer_angle);

        // Get the ascii sentences from the data to find vehicle nav info
        std::vector<common::AsciiSentence> sentences = ping_data->get_ascii_sentences();

        // Iterate over ascii sentences to find the correct one
        for (std::vector<common::AsciiSentence>::iterator it = sentences.begin(); it != sentences.end(); ++it)
        {

            // Get the ascii string as a string
            char* char_str = reinterpret_cast<char*> (it->sentence);
            std::string full_ascii_sentence(char_str);

            // Get the ascii sentence with the navigation data
            if (avl::starts_with(full_ascii_sentence, "$NAV"))
            {

                // Remove the checksum from the NMEA sentence
                avl::strip(full_ascii_sentence, '\r');
                avl::strip(full_ascii_sentence, '\n');
                size_t checksum_pos = full_ascii_sentence.find("*");
                std::string ascii_sentence = full_ascii_sentence.substr(0, checksum_pos);

                // Split the comma delimited NMEA message
                std::vector<std::string> split_line = avl::split(ascii_sentence, ",");

                // Get the navigation values
                roll = stod(split_line.at(2)); // radians
                pitch = stod(split_line.at(3)); // radians
                yaw = stod(split_line.at(4)); // radians
                lat = stod(split_line.at(5)); // radians
                lon = stod(split_line.at(6)); // radians
                alt = stod(split_line.at(7)); // meters from mean sea level

                // Get the first point for reference
                if(first)
                {
                    lat0 = lat;
                    lon0 = lon;
                    alt0 = alt;
                    first = false;
                }

                break;
            }
        }

        // Clear out the data from the previous ping
        port_sonar_data.clear();
        stbd_sonar_data.clear();

        // Retreive the 3D sonar data from the current ping
        std::vector<common::Sidescan3DPoint> port_data= ping_data->get_port_sidescan3d_points();
        std::vector<common::Sidescan3DPoint> stbd_data = ping_data->get_starboard_sidescan3d_points();

        // Initialize the point cloud
        pcl::PointCloud<pcl::PointXYZI> point_cloud;
        point_cloud.header.frame_id = "map";

        // Transform the port sonar data into geo coordinates
        double sonar_roll;
        Vector3d sonar_coordinates(0.0, 1.0, 0.0); // Unit vector along the body y axis
        Vector3d body_coordinates;
        Vector3d ned_coordinates;
        double point_lat;
        double point_lon;
        double point_alt;
        Matrix3d R_s_b;
        Matrix3d R_b_n = avl::euler_to_matrix<double>(roll, pitch, yaw);
        GeoSonarPoint point;
        double n, e, d;
        for (std::vector<common::Sidescan3DPoint>::iterator it = port_data.begin(); it != port_data.end(); ++it)
        {
            sonar_roll = M_PI + it->angle - port_transducer_angle;
            R_s_b = avl::euler_to_matrix<double>(sonar_roll, 0.0, 0.0);
            body_coordinates = R_s_b*sonar_coordinates*it->range;
            ned_coordinates = R_b_n*body_coordinates;
            ned_to_geo(lat, lon, alt, ned_coordinates(0), ned_coordinates(1), ned_coordinates(2), point_lat, point_lon, point_alt);
            point.lat = point_lat;
            point.lon = point_lon;
            point.alt = point_alt;
            point.intensity = it->amplitude;
            port_sonar_data.push_back(point);

            geo_to_ned(lat0, lon0, alt0, point_lat,  point_lon,  point_alt, n,   e,   d);
            pcl::PointXYZI sonar_point;
            sonar_point.x = n;
            sonar_point.y = e;
            sonar_point.z = -d;
            sonar_point.intensity = it->amplitude;
            point_cloud.push_back(sonar_point);
        }

        // Transform the starboard sonar data into geo coordinates
        for (std::vector<common::Sidescan3DPoint>::iterator it = stbd_data.begin(); it != stbd_data.end(); ++it)
        {
            sonar_roll = -it->angle + stbd_transducer_angle;
            R_s_b = avl::euler_to_matrix<double>(sonar_roll, 0.0, 0.0);
            body_coordinates = R_s_b*sonar_coordinates*it->range;
            ned_coordinates = R_b_n*body_coordinates;
            ned_to_geo(lat, lon, alt, ned_coordinates(0), ned_coordinates(1), ned_coordinates(2), point_lat, point_lon, point_alt);
            point.lat = point_lat;
            point.lon = point_lon;
            point.alt = point_alt;
            point.intensity = it->amplitude;
            stbd_sonar_data.push_back(point);

            geo_to_ned(lat0, lon0, alt0, point_lat,  point_lon,  point_alt, n,   e,   d);
            pcl::PointXYZI sonar_point;
            sonar_point.x = n;
            sonar_point.y = e;
            sonar_point.z = -d;
            sonar_point.intensity = it->amplitude;
            point_cloud.push_back(sonar_point);
        }

        pcl_pub.publish(point_cloud);

        // Setup the socket to read the next ping
        socket.set_match(Match(PINGDSP_SONAR_DATA_HEADER, &TempSonarNode::tcp_data_header_callback, this));

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Open the TCP connection to the sonar computer
        socket.connect(get_param<std::string>("~sonar_computer_address"), PINGDSP_SONAR_DATA_PORT);
        socket.set_match(Match(PINGDSP_SONAR_DATA_HEADER, &TempSonarNode::tcp_data_header_callback, this));

        // Set up the publisher for point cloud data
        pcl_pub = node_handle->advertise<pcl::PointCloud<pcl::PointXYZI>>("/device/pcl_sonar", 1);

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
            socket.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function when the node is started. Can
    //              be overriden by a derived node class.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        socket.close();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    TempSonarNode node(argc, argv);
    node.start();
    return 0;
}
