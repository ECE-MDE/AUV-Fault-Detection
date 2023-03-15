//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node for 900 MHz radio communication via an XTend-PKG Digi
//              Radio. Intended for topside use.
//
// Servers:     /radio/thruster_stop (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  /dive/range_bearing (avl_msgs/MagBearingMsg)
//
// Subscribers: device/gps         (avl_msgs/GpsMsg)
//==============================================================================

// Node base class
#include <avl_devices/device_node.h>

// Serial port class
#include <avl_asio/tcp_socket.h>

// Utility functions
#include <avl_core/util/string.h>
#include <avl_core/util/geo.h>

// ROS message includes
#include <avl_msgs/GpsMsg.h>
#include <avl_msgs/MagBearingMsg.h>
#include <std_srvs/Trigger.h>
using namespace avl_msgs;

// Digi command protocol
#include <avl_devices/protocol/digi.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TopsideDigiRadioNode : public DeviceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        TopsideDigiRadioNode constructor
    //--------------------------------------------------------------------------
    TopsideDigiRadioNode(int argc, char **argv) : DeviceNode(argc, argv)
    {

    }

private:

    // Serial device instance
    TcpSocket tcp;

    // Variables for basic class
    double radio_device;

    // Subscribers
    ros::Subscriber gps_sub;

    // Services
    ros::ServiceServer tc_stop_srv;

    // Timers for regular message transmissions
    ros::Timer feedback_timer;
    ros::Duration feedback_duration;

    // Tracking Structs
    int nav_now{};
    int nav_age{};
    double nav_lat{};
    double nav_lon{};
    int nav_v_now{};
    int nav_v_age{};
    double nav_v{};
    double nav_course{};
    int topside_gps_now{};
    int topside_gps_age{};
    double topside_gps_lat{};
    double topside_gps_lon{};
    int gps_now{};
    int gps_age{};
    double gps_lat{};
    double gps_lon{};
    int rpm_now{};
    int rpm_age{};
    double rpm{};
    double last_vehicle_comms{3600};
    double gps_d{};
    double gps_h{};
    double nav_d{};
    double nav_h{};

private:

    //--------------------------------------------------------------------------
    // Name:        tc_stop_callback
    // Description: Sends a TC stop request via radio
    // Arguments:   - request: request received on the service(empty)
    //              - response.success: boolean response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool tc_stop_callback(std_srvs::Trigger::Request& request,
        std_srvs::Trigger::Response& response)
    {
        tcp.write("stop\r\n");
        response.success = true;

        return true;
    }


    //--------------------------------------------------------------------------
    // Name:        gps_callback
    // Description: Called when gps subscriber receives a message.
    // Arguments:   - msg: data from gps ROS TOPIC.
    //--------------------------------------------------------------------------
    void gps_callback(const GpsMsg::ConstPtr& msg)
    {
        topside_gps_lat = msg->lat;
        topside_gps_lon = msg->lon;
        topside_gps_now = ros::Time::now().sec;
        return;

    }

    //--------------------------------------------------------------------------
    // Name:        feedback_timer_callback
    // Description: Publish tracking information
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void feedback_timer_callback(const ros::TimerEvent& event)
    {
          topside_gps_age = ros::Time::now().sec - topside_gps_now;
          gps_d = distance(topside_gps_lat, topside_gps_lon, gps_lat, gps_lon);
          gps_h = final_bearing(topside_gps_lat, topside_gps_lon, gps_lat, gps_lon);
          log_data("[TRACKING_GPS] \t %.0fm \t %.0fdeg \t topside_age: %u \t vehicle_age: %.0f",
            gps_d, gps_h, topside_gps_age, gps_age + last_vehicle_comms);

          nav_d = distance(topside_gps_lat, topside_gps_lon, nav_lat, nav_lon);
          nav_h = final_bearing(topside_gps_lat, topside_gps_lon, nav_lat, nav_lon);
          log_data("[TRACKING_NAV] \t %.0fm \t %.0fdeg \t topside_age: %u \t vehicle_age: %.0f",
            nav_d, nav_h, topside_gps_age, nav_age + last_vehicle_comms);

          log_data("[TRACKING_COURSE] %.1fm/s  %.0fdeg \t\t\t vehicle_age: %.0f",
            nav_v, nav_course, nav_v_age + last_vehicle_comms);
          log_data("[TRACKING_RPM] \t %.0f \t\t\t\t\t vehicle_age: %.0f",
            rpm, rpm_age + last_vehicle_comms);

        last_vehicle_comms += event.current_real.toSec() - event.last_real.toSec();
    }

    //--------------------------------------------------------------------------
    // Name:        read_handler
    // Description: Handler for messages over RF
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void read_handler(std::vector<uint8_t> data)
    {
        try
        {
            // Format the data as a string, and remove all \r characters
            std::string line = std::string(data.begin(), data.end());
            log_data(line);
            avl::strip(line, '\r');


            // Split the comma delimited NMEA message
            std::vector<std::string> split_line = avl::split(line, "\n");
            std::vector<std::string> nav_line = avl::split(split_line[1]," ");
            nav_age = std::stoi(nav_line.at(5));
            avl::strip(nav_line[2], 'N');
            nav_lat = std::stod(nav_line.at(2));
            avl::strip(nav_line[3], 'E');
            nav_lon = std::stod(nav_line.at(3));
            log_data("[NAV] %.5fN, %.5fE Age: %u", nav_lat, nav_lon, nav_age);

            std::vector<std::string> course_line= avl::split(split_line[3]," ");
            nav_v_age = std::stoi(course_line.at(5));
            avl::strip(course_line[2], 'm');
            avl::strip(course_line[2], '/');
            avl::strip(course_line[2], 's');
            nav_v = std::stod(course_line.at(2));
            avl::strip(course_line[3], 'd');
            avl::strip(course_line[3], 'e');
            avl::strip(course_line[3], 'g');
            nav_course = std::stod(course_line.at(3));
            log_data("[NAV_COURSE] %.5fm/s, %.5fdeg Age: %u", nav_v, nav_course, nav_v_age);

            std::vector<std::string> gps_line= avl::split(split_line[5]," ");
            gps_age = std::stoi(gps_line.at(4));
            avl::strip(gps_line[2], 'N');
            gps_lat = std::stod(gps_line.at(1));
            avl::strip(gps_line[3], 'E');
            gps_lon = std::stod(gps_line.at(2));
            log_data("[GPS] %.5fN, %.5fE Age: %u", gps_lat, gps_lon, gps_age);

            std::vector<std::string> rpm_line= avl::split(split_line[7]," ");
            rpm_age = std::stoi(rpm_line.at(3));
            rpm = std::stod(rpm_line.at(1));
            log_data("[RPM] %.2f Age: %u", rpm, rpm_age);

            last_vehicle_comms = 0;
        }
        catch( const std::exception &ex)
        {
            log_warning("Message parse error");
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
        params.add(Parameter("NAV_RANGE",static_cast<int>(nav_d) ));
        params.add(Parameter("NAV_BEARING", static_cast<int>(nav_h)));
        params.add(Parameter("NAV_AGE", ( topside_gps_age > nav_age + last_vehicle_comms) ? static_cast<int>(topside_gps_age) : static_cast<int>(nav_age + last_vehicle_comms) ));
        params.add(Parameter("GPS_RANGE", static_cast<int>(gps_d)));
        params.add(Parameter("GPS_BEARING", static_cast<int>(gps_h)));
        params.add(Parameter("GPS_AGE", ( topside_gps_age > gps_age + last_vehicle_comms) ? static_cast<int>(topside_gps_age) : static_cast<int>(gps_age + last_vehicle_comms )));
        params.add(Parameter("COURSE_SPEED", nav_v));
        params.add(Parameter("COURSE_HEADING", static_cast<int>(nav_course)));
        params.add(Parameter("COURSE_AGE", static_cast<int>(nav_v_age + last_vehicle_comms)));
        params.add(Parameter("RPM", static_cast<int>(rpm)));
        params.add(Parameter("RPM_AGE", static_cast<int>(rpm_age + last_vehicle_comms)));
        params.add(Parameter("SIG_STRENGTH", radio_device));

        return params;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        log_info("initializing Digi Radio...");

        // Log data headers

        // Set up services
        tc_stop_srv = node_handle->advertiseService("radio/thruster_stop",
          &TopsideDigiRadioNode::tc_stop_callback, this);

        // Set up subscribers
        gps_sub = node_handle->subscribe("device/gps", 1,
          &TopsideDigiRadioNode::gps_callback, this);

        // Configure feedback request timer
        feedback_duration = ros::Duration(1/(get_param<float>("~output_rate")));
        feedback_timer = node_handle->createTimer(feedback_duration,
          &TopsideDigiRadioNode::feedback_timer_callback, this);

        try
        {
            std::string tcp_address = get_param<std::string>("~tcp/address");
            int tcp_port = get_param<int>("~tcp/port");
            log_info("Attempting to connect to " + tcp_address + ":" + std::to_string(tcp_port));
            // Open the TCP port
            tcp.connect(tcp_address, tcp_port);

            // Set the match condition to a newline
            tcp.set_match(Match("--END--", &TopsideDigiRadioNode::read_handler, this));

            feedback_timer.start();
            log_info("TCP Connection successful");
        }
        catch (const std::exception &ex)
        {
            log_warning("TCP Connection failed");
            feedback_timer.stop();
            return;
            //TODO probably try to close the port (if it's already open)
        }

        // Set device name and default DEVICE packet output rates
        set_device_name("TOPSIDE_RADIO");
        set_device_packet_output_rate(1, INTERFACE_FSD);
        set_device_packet_output_rate(1, INTERFACE_BSD);


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
            tcp.spin_once();
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
    TopsideDigiRadioNode node(argc, argv);
    node.start();
    return 0;
}
