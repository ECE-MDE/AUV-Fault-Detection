//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node for 900 MHz radio communication via an XTend-PKG Digi
//              Radio
//
// Servers:     None
//
// Clients:     /dive/thruster_enable   (avl_devices/MotorSrv)
//
// Publishers:  None
//
// Subscribers: device/gps         (avl_devices/GpsMsg)
//              device/rpm         (std_msgs/Float64)
//              /dive/course        (avl_devices/MagBearingMsg)
//              nav/inertial_nav   (avl_devices/Navigation)
//==============================================================================

// Node base class
#include <avl_devices/device_node.h>

// Serial port class
#include <avl_asio/serial_port.h>

// Utility functions
#include <avl_core/util/string.h>

// ROS message includes
#include <avl_msgs/GpsMsg.h>
#include <avl_msgs/NavigationMsg.h>
#include <avl_msgs/MagBearingMsg.h>
#include <avl_msgs/MotorSrv.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// Digi command protocol
#include <avl_devices/protocol/digi.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class DigiRadioNode : public DeviceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        DigiRadioNode constructor
    //--------------------------------------------------------------------------
    DigiRadioNode(int argc, char **argv) : DeviceNode(argc, argv)
    {

    }

private:

    // Serial device instance
    SerialPort serial;

    // Variables for basic class
    double radio_device;

    // Subscribers
    ros::Subscriber course_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber nav_sub;
    ros::Subscriber rpm_sub;

    // Services
    ros::ServiceClient motor_state_client;

    // Timers for regular message transmissions
    ros::Timer feedback_timer;
    ros::Duration feedback_duration;

    // Tracking stats
    int nav_now{};
    int nav_age{};
    double nav_lat{};
    double nav_lon{};
    int nav_v_now{};
    int nav_v_age{};
    double nav_v{};
    double nav_course{};
    int gps_now{};
    int gps_age{};
    double gps_lat{};
    double gps_lon{};
    int rpm_now{};
    int rpm_age{};
    double rpm{};
    bool thruster_req{};

private:

    //--------------------------------------------------------------------------
    // Name:        gps_callback
    // Description: Called when gps subscriber receives a message.
    // Arguments:   - msg: data from gps ROS TOPIC.
    //--------------------------------------------------------------------------
    void gps_callback(const GpsMsg::ConstPtr& msg)
    {
        gps_lat = msg->lat;
        gps_lon = msg->lon;
        gps_now = ros::Time::now().sec;
        return;

    }

    //--------------------------------------------------------------------------
    // Name:        nav_callback
    // Description: Called when navigation subscriber receives a message.
    // Arguments:   - msg: data from nav ROS TOPIC.
    //--------------------------------------------------------------------------
    void nav_callback(const NavigationMsg::ConstPtr& msg)
    {
        nav_lat = msg->lat;
        nav_lon = msg->lon;
        nav_now = ros::Time::now().sec;
        return;

    }

    //--------------------------------------------------------------------------
    // Name:        course_callback
    // Description: Called when course subscriber receives a message.
    // Arguments:   - msg: data from ROS TOPIC.
    //--------------------------------------------------------------------------
    void course_callback(const MagBearingMsg::ConstPtr& msg)
    {
        nav_v = msg->mag;
        nav_course = msg->bearing;
        nav_v_now = ros::Time::now().sec;
        return;

    }

    //--------------------------------------------------------------------------
    // Name:        rpm_callback
    // Description: Called when rpm subscriber receives a message.
    // Arguments:   - msg: data from rpm ROS TOPIC.
    //--------------------------------------------------------------------------
    void rpm_callback(const std_msgs::Float64::ConstPtr& msg)
    {
        rpm = msg->data;
        rpm_now = ros::Time::now().sec;
        return;

    }

    //--------------------------------------------------------------------------
    // Name:        feedback_timer_callback
    // Description: Called when feedback timer reach. Send the request packet.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void feedback_timer_callback(const ros::TimerEvent& event)
    {

          //Form feedback ASCII packet
          std::string message = "--START--\n\r";
          message += "Nav Location: ";
          message += std::to_string(nav_lat) + "N ";
          message += std::to_string(nav_lon) + "E ";
          nav_age = ros::Time::now().sec - nav_now;
          message += "Age: " + std::to_string(nav_age) + "\n\r\n\r";

          message += "Nav Course: ";
          message += std::to_string(nav_v) + "m/s ";
          message += std::to_string(nav_course) + "deg ";
          nav_v_age = ros::Time::now().sec - nav_v_now;
          message += "Age: " + std::to_string(nav_v_age) + "\n\r\n\r";

          message += "GPS: ";
          message += std::to_string(gps_lat) + "N ";
          message += std::to_string(gps_lon) + "E ";
          gps_age = ros::Time::now().sec - gps_now;
          message += "Age: " + std::to_string(gps_age) + "\n\r\n\r";

          message += "RPM: ";
          message += std::to_string(rpm) + " ";
          rpm_age = ros::Time::now().sec - rpm_now;
          message += "Age: " + std::to_string(rpm_age) + "\n\r";
          message += "--END--";
          std::vector<uint8_t> feedback = avl::to_bytes(message);

          serial.write(feedback);
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
            // if the request contains "stop", disable thruster
            std::string line = std::string(data.begin(), data.end());
            log_info("Message received over RF: %s", line.c_str());
            if ( line.find("stop") != std::string::npos )
            {
                log_warning("Thruster stop requested");
                thruster_req = true;
                MotorSrv motor_enable;
                motor_enable.request.state_enable = false;
                motor_enable.request.rpm = 0;
                motor_enable.request.timeout = 0;
                motor_state_client.call(motor_enable);
                if (!motor_enable.response.success)
                {
                    log_warning("Thruster stop request failed");
                    std::string message = "\n\n\n\n!!!!Disable request failed!!!!"
                        "\n\n\n\n\r";
                    serial.write( avl::to_bytes(message) );
                    return;
                }
                std::string message = "\n\n\n\nDisable request completed"
                    "\n\n\n\n\r";
                serial.write( avl::to_bytes(message) );
                return;

            }
            std::string message = "\n\n\n\nUnrecognized Request:\n\r"
                + line + "\nOnly recognizable commands are:\n\r"
                "stop (case sensitive)\n\n\n\n\r";
            serial.write( avl::to_bytes(message) );
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

        // Set up the publishers

        // Set up subscribers
        gps_sub = node_handle->subscribe("device/gps", 1,
          &DigiRadioNode::gps_callback, this);
        nav_sub = node_handle->subscribe("nav/inertial_nav", 1,
          &DigiRadioNode::nav_callback, this);
        course_sub = node_handle->subscribe("dive/course", 1,
          &DigiRadioNode::course_callback, this);
        rpm_sub = node_handle->subscribe("device/rpm", 1,
          &DigiRadioNode::rpm_callback, this);

        // Service Clients
        motor_state_client = node_handle->serviceClient<MotorSrv>
            ("dive/thruster_enable");

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set the match condition to a newline
        serial.set_match(Match("\r", &DigiRadioNode::read_handler, this));

        // Configure feedback request timer
        feedback_duration = ros::Duration(1/(get_param<float>("~output_rate")));
        feedback_timer = node_handle->createTimer(feedback_duration,
          &DigiRadioNode::feedback_timer_callback, this);

        // Set device name and default DEVICE packet output rates
        set_device_name("RADIO");
        set_device_packet_output_rate(0, INTERFACE_FSD);
        set_device_packet_output_rate(0, INTERFACE_BSD);


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
    DigiRadioNode node(argc, argv);
    node.start();
    return 0;
}
