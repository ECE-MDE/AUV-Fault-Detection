//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to communicate with the XEOS strobe and iridium
//              function
//
// Servers:     /dive/strobe (avl_msgs/StrobeSrv)
//              /xeos/command (avl_msgs/StringSrv)
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================


// Core utility
#include <avl_asio/serial_port.h>
#include <avl_core/util/string.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>
#include <avl_core/util/misc.h>

// Node base class
#include <avl_devices/device_node.h>

// ROS message includes
#include <avl_msgs/StringSrv.h>
#include <avl_msgs/StrobeSrv.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class IridiumStrobeNode : public DeviceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        IridiumStrobeNode constructor
    //--------------------------------------------------------------------------
    IridiumStrobeNode(int argc, char **argv) : DeviceNode(argc, argv) {}

private:

    // Serial device instance
    SerialPort serial;

    // Services
    ros::ServiceServer command_srv;
    ros::ServiceServer strobe_server;

    // Ros Timer for device query/command
    ros::Timer wake_timer;
    ros::Timer defer_timer;
    ros::Duration wake_duration;
    ros::Duration defer_duration;

    // Boolean for whether to set gps timer upong receipt of a "$Timer" message
    bool gps_timer{true};

private:

    //--------------------------------------------------------------------------
    // Name:        command_srv_callback
    // Description: Takes a string argument and passes directly to the Xeos
    // Arguments:   - request.str: seconds between strobe sequences
    //              - response.success: boolean of whether command was ACK'ed
    //--------------------------------------------------------------------------
    bool command_srv_callback(StringSrv::Request& request,
        StringSrv::Response& response)
    {
        std::string message = request.str + '\n';
        log_info("Request to send command to Xeos: %s", request.str.c_str());
	delay_write(request.str);

        response.success = true;
            // response.success = false; TODO, add logic to check if response
            // was acknowledged

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        change_strobe_callback
    // Description: A function that changes the rate and pulse count of the
    //              strobe
    // Arguments:   - request.period: seconds between strobe sequences
    //              - request.pulse: number of pulses per strobe sequence
    //              - response.success: boolean of whether command was ACK'ed
    //--------------------------------------------------------------------------
    bool change_strobe_callback(StrobeSrv::Request& request,
                                StrobeSrv::Response& response)
    {
        serial.write("$switch L 0\n");
	ros::Duration(0.5).sleep();
        log_info("Request to strobe at interval of %d s with %d pulses",
            request.period, request.pulse);

        response.success = true;
        try
        {
            if (request.pulse == 0 || request.period == 0)
            {
                serial.write("$switch B 0\n");
            }
            else
            {
                serial.write("$switch B 1\n");
            }

            serial.write("$strobe " + std::to_string(request.pulse) + ' ' +
                         std::to_string(request.period) + '\n');

            // response.success = false; TODO, add logic to check if response
            // was acknowledged
        }

        catch (...)
        {

            response.success = false;
            return false;

        }
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        delay_write
    // Description: Write a serial message with a delay prior to sending
    // Arguments:   - line: content of message to send
    // 		    - delay: time in seconds to wait before sending
    //--------------------------------------------------------------------------
    void delay_write(std::string line, float delay = 0.1)
    {
	ros::Duration(delay).sleep();
        log_data("[XEOS] <%s", line.c_str());
	serial.write(line + '\n');
    }

    //--------------------------------------------------------------------------
    // Name:        defer_callback
    // Description: Push back Xeos internal timers
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void defer_callback(const ros::TimerEvent& event)
    {
	defer_timer.start();
	// Set wake timer to default
	wake_timer.setPeriod( wake_duration );
	// Add 30s to defer time to ensure Xeos doesn't enter GPS or Iridium
	std::string str_time =
		std::to_string(static_cast<int>( defer_duration.toSec() + 30 ));
	delay_write("$deferig " + str_time);
    }

    //--------------------------------------------------------------------------
    // Name:        wake_callback
    // Description: Used to initialize Xeos once it's confirmed on
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void wake_callback(const ros::TimerEvent& event)
    {
	defer_timer.stop();
	// Query for status TODO check response for passing
	delay_write("$status");
    }

    //--------------------------------------------------------------------------
    // Name:        read_handler
    // Description: Handler for Xeos messages
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void read_handler(std::vector<uint8_t> data)
    {
	    //for (size_t i{0}; i< data.size(); i++)
	//		log_debug("Char #%u: %u", i , data.at(i));
	while (data.at(0) == 0)
		data.erase(data.begin());
        // Format the data as a string, and remove any \r\n characters
        std::string line = std::string(data.begin(), data.end());
        avl::strip(line, '\r');
        avl::strip(line, '\n');
        // Log data
        log_data("[XEOS] >%s", line.c_str());

	// Reset wake timer

        try
        {
	  // Don't process nmea strings
	  if ( line.compare(0,2,"&G") == 0 )
  		  return;
	  else if ( line.compare(0,10,"Timer Time") == 0 )
		// Start NMEA output
		delay_write("$gnmea 1", 0.3);
	  else if ( line.compare(0,6,"Status") == 0 )
		// Turn off underwater mode
		delay_write("$switch U 0");
	  else if ( line.compare(0,6,"Switch") == 0 )
		// Cancel startup timer
		delay_write("$evtcancel 1");
	  else if ( line.compare(0,5,"Event") == 0 )
		// Enter engineering mode
		delay_write("$engmode 2009");
	  else if ( line.compare(0,8,"Eng mode") == 0 )
		// Set 5min sbd interval
		delay_write("$timer sbd 5m");
	  else if ( line.compare(0,11,"Timer Modes") == 0 )
	  {
		// First receipt is sbd response, second is gps
		if (gps_timer)
			delay_write("$timer gps 5m", 0.5);
		else
		{
			ros::TimerEvent empty_event{};
			defer_callback(empty_event);
		}
		gps_timer = !gps_timer;
	  }
	  else if ( line.compare(0,10,"Diagnostic") == 0 )
	  {
		// Assume Xeos has just turned on
		delay_write("$status", 10);
	  }
        }
        catch (const std::exception &ex)
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
        // params.add_parameter(Parameter("PRES", pressure_device));

        return params;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set the match condition to a newline
        serial.set_match(Match("\n", &IridiumStrobeNode::read_handler, this));

        // Set device name and default DEVICE packet output rates
        set_device_name("XEOS_IRIDIUM_STROBE");
        set_device_packet_output_rate(1.0, INTERFACE_FSD);
        set_device_packet_output_rate(1.0, INTERFACE_BSD);

        // Setup services
        command_srv = node_handle->advertiseService("xeos/command",
            &IridiumStrobeNode::command_srv_callback, this);
        strobe_server = node_handle->advertiseService("dive/strobe",
            &IridiumStrobeNode::change_strobe_callback, this);

	// Configure comms request timer
        defer_duration = ros::Duration(get_param<float>
            ("~defer_duration"));
        wake_duration = ros::Duration(200.0);
        defer_timer = node_handle->createTimer(defer_duration,
            &IridiumStrobeNode::defer_callback, this);
        wake_timer = node_handle->createTimer(wake_duration,
            &IridiumStrobeNode::wake_callback, this);
	defer_timer.stop();
	wake_timer.stop();
	// Check for Xeos comms
	delay_write("$status");

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
    IridiumStrobeNode node(argc, argv);
    node.start();
    return 0;
}
