//==============================================================================
// Autonomous Vehicle Library
//
// Description: Interfaces with a computer running PingDSP's 3DSS control
//              software for control of a PingDSP 3D sidescan sonar. Provides
//              services to enable/disable the sonar and start/stop recoring.
//
// Servers:     device/ping_dsp/enable_sonar (std_srvs/Trigger)
//              device/ping_dsp/disable_sonar (std_srvs/Trigger)
//              device/ping_dsp/start_recording (std_srvs/Trigger)
//              device/ping_dsp/stop_recording (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>
#include <avl_core/util/misc.h>

// TCP socket class
#include <avl_asio/tcp_socket.h>

// Command handler class
#include <avl_comms/command_handler.h>
using namespace avl;

// Util functions
#include <avl_core/util/string.h>

// PingDSP sonar command protocol
#include <avl_devices/protocol/pingdsp_sonar.h>

// ROS messages
#include <std_srvs/Trigger.h>
#include <avl_msgs/NavigationMsg.h>
using namespace avl_msgs;

using namespace boost::asio::ip;

//==============================================================================
//                              ENUM DEFINITION
//==============================================================================

// Enum listing sonar commands
enum SonarCommand
{
    SONAR_ENABLE,
    SONAR_START_RECORDING,
    SONAR_STOP_RECORDING,
    SONAR_DISABLE
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class PingdspSonarNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        PingdspSonarNode constructor
    //--------------------------------------------------------------------------
    PingdspSonarNode(int argc, char **argv) : Node(argc, argv), io_service(),
        udp_socket(io_service, udp::endpoint(udp::v4(), 0))
    {

    }

private:

    // IP address of the computer running the 3DSS sonar control software
    std::string sonar_computer_address;

    // TCP socket for connection to computer running 3DSS sonar control software
    TcpSocket socket;
    double retry_duration = 10.0;

    // Subscriber for inertial nav information to be fed to the sonar
    ros::Subscriber inertial_nav_sub;

    // Service servers for control of the sonar data stream and recording
    ros::ServiceServer enable_sonar_server;
    ros::ServiceServer disable_sonar_server;
    ros::ServiceServer start_recording_server;
    ros::ServiceServer stop_recording_server;

    // Path to folder to save data to form config file
    std::string data_folder;

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

    // TEMPORARY UDP ***********************************************************

    // Boost ASIO IO service used to run async io
    boost::asio::io_service io_service;

    // Boost ASIO UDP socket instance
    udp::socket udp_socket;

    // Endpoint for the UDP server
    udp::endpoint udp_server_endpoint;

    // *************************************************************************

private:

    //--------------------------------------------------------------------------
    // Name:        send_udp_message
    // Description: Sends a message to the UDP server.
    // Arguments:   - message: message to send to the UDP server.
    //--------------------------------------------------------------------------
    void send_udp_message(std::string message)
    {
        try
        {
            udp_socket.send_to(boost::asio::buffer(message, message.size()),
                udp_server_endpoint);
        }
        catch (const std::exception& ex)
        {
            log_error("failed to send message to UDP server (%s)", ex.what());
        }
    }

    //--------------------------------------------------------------------------
    // Name:        inertial_nav_msg_callback
    // Description: Called when an inertial nav message is received.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void inertial_nav_msg_callback(const NavigationMsg& message)
    {

        // Format an NMEA message to send to the sonar
        char buffer[1024];
        sprintf(buffer, "$NAV,%.9f,%.3f,%.3f,%.3f,%.9f,%.9f,%.3f",
            avl::get_epoch_time(),
            message.roll,
            message.pitch,
            message.yaw,
            message.lat,
            message.lon,
            message.alt);
        std::string nmea_msg(buffer);
        nmea_msg = nmea_msg + avl::get_nmea_checksum(nmea_msg) + "\r\n";

        // Write the NMEA message to the sonar
        send_udp_message(nmea_msg);

    }

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when a COMMAND packet is received by the
    //              communication architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - command_name: Command name of the received command.
    //              - params: Command parameter list.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool command_callback(CommsChannel channel, CommsInterface interface,
        std::string command_name, ParameterList params,
        bool& result, std::vector<uint8_t>& data)
    {

        // Handle SONAR commands
        if (command_name == "SONAR")
        {

            // Check that socket is connected to sonar computer
            if (!socket.is_connected())
            {
                result = false;
                std::string message = "sonar computer not connected";
                data = std::vector<uint8_t>(message.begin(), message.end());
                return true;
            }

            SonarCommand command = params.get("COMMAND")
                .to_enum<SonarCommand>();

            if (command == SONAR_ENABLE)
            {
                socket.write(PINGDSP_START_DATA_STREAM());
            }

            else if (command == SONAR_START_RECORDING)
            {

                // Get the current time as a string to use as the file name
                auto time = std::time(nullptr);
                auto local_time = *std::localtime(&time);
                std::ostringstream oss;
                oss << std::put_time(&local_time, "%d-%m-%Y %H-%M-%S");
                std::string time_string = oss.str();
                std::string filename = data_folder + time_string + ".3dss-dx";
                log_info("starting sonar data stream recording to filename %s",
                    filename.c_str());
                socket.write(PINGDSP_START_RECORDING(filename, true));

            }

            else if (command == SONAR_STOP_RECORDING)
            {
                socket.write(PINGDSP_STOP_RECORDING());
            }

            else if (command == SONAR_DISABLE)
            {
                socket.write(PINGDSP_STOP_DATA_STREAM());
            }

            else
            {
                result = false;
                std::string message = "unknown sonar command";
                data = std::vector<uint8_t>(message.begin(), message.end());
                return true;
            }

            result = true;
            return true;

        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        enable_sonar_srv_callback
    // Description: Called when the enable_sonar service is requested. Starts
    //              the sonar data stream.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool enable_sonar_srv_callback(std_srvs::Trigger::Request& req,
                                   std_srvs::Trigger::Response& response)
    {

        // Check that socket is connected to sonar computer
        if (socket.is_connected())
        {
            log_info("enabling sonar data stream");
            socket.write(PINGDSP_START_DATA_STREAM());
            response.success = true;
            response.message = "success";
        }
        else
        {
            response.success = false;
            response.message = "sonar computer not connected";
        }

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        disable_sonar_srv_callback
    // Description: Called when the disable_sonar service is requested. Stops
    //              the sonar data stream.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool disable_sonar_srv_callback(std_srvs::Trigger::Request& req,
                                    std_srvs::Trigger::Response& response)
    {

        // Check that socket is connected to sonar computer
        if (socket.is_connected())
        {
            log_info("disabling sonar data stream");
            socket.write(PINGDSP_STOP_DATA_STREAM());
            response.success = true;
            response.message = "success";
        }
        else
        {
            response.success = false;
            response.message = "sonar computer not connected";
        }

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        start_recording_srv_callback
    // Description: Called when the start_recording service is requested. Starts
    //              recording the sonar data stream to file.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool start_recording_srv_callback(std_srvs::Trigger::Request& req,
                                      std_srvs::Trigger::Response& response)
    {

        // Check that socket is connected to sonar computer
        if (socket.is_connected())
        {

            // Get the current time as a string to use as the file name
            auto time = std::time(nullptr);
            auto local_time = *std::localtime(&time);
            std::ostringstream oss;
            oss << std::put_time(&local_time, "%d-%m-%Y %H-%M-%S");
            std::string time_string = oss.str();
            std::string filename = data_folder + time_string + ".3dss-dx";
            log_info("starting sonar data stream recording to filename %s",
                filename.c_str());
            socket.write(PINGDSP_START_RECORDING(filename, true));
            response.success = true;
            response.message = "success (sonar recording to " + filename + ")";

        }
        else
        {
            response.success = false;
            response.message = "sonar computer not connected";
        }

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        stop_recording_srv_callback
    // Description: Called when the stop_recording service is requested. Stops
    //              recording the sonar data stream.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool stop_recording_srv_callback(std_srvs::Trigger::Request& req,
                                     std_srvs::Trigger::Response& response)
    {

        // Check that socket is connected to sonar computer
        if (socket.is_connected())
        {
            log_info("stopping sonar data stream recording");
            socket.write(PINGDSP_STOP_RECORDING());
            response.success = true;
            response.message = "success";
        }
        else
        {
            response.success = false;
            response.message = "sonar computer not connected";
        }

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        tcp_read_callback
    // Description: Called when a TCP message is read.
    // Arguments:   - data: message bytes
    //--------------------------------------------------------------------------
    void tcp_read_callback(std::vector<uint8_t> data)
    {

        // Format the data as a string, and remove any \r\n characters
        std::string line = std::string(data.begin(), data.end());
        avl::strip(line, '\r');
        avl::strip(line, '\n');
        log_debug("[rx] "+ line);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get the config file params
        data_folder = get_param<std::string>("~data_folder");
        sonar_computer_address = get_param<std::string>("~sonar_computer_address");

        // Set up the service servers
        enable_sonar_server = node_handle->advertiseService(
            "device/ping_dsp/enable_sonar",
            &PingdspSonarNode::enable_sonar_srv_callback, this);
        disable_sonar_server = node_handle->advertiseService(
            "device/ping_dsp/disable_sonar",
            &PingdspSonarNode::disable_sonar_srv_callback, this);
        start_recording_server = node_handle->advertiseService(
            "device/ping_dsp/start_recording",
            &PingdspSonarNode::start_recording_srv_callback, this);
        stop_recording_server = node_handle->advertiseService(
            "device/ping_dsp/stop_recording",
            &PingdspSonarNode::stop_recording_srv_callback, this);

        // Set up the inertial nav subscriber
        inertial_nav_sub = node_handle->subscribe("nav/inertial_nav", 1,
            &PingdspSonarNode::inertial_nav_msg_callback, this);

        // Set the command handler's callback
        command_handler.set_callback(&PingdspSonarNode::command_callback, this);

        // Open the TCP connection to the sonar computer
        do
        {

            try
            {
                log_info("attempting to connect to sonar computer (%s:%d)...",
                    sonar_computer_address.c_str(), PINGDSP_SONAR_CONTROL_PORT);
                socket.connect(sonar_computer_address,
                    PINGDSP_SONAR_CONTROL_PORT);
                socket.set_match(Match("\n",
                    &PingdspSonarNode::tcp_read_callback, this));
                log_info("successfully connected to sonar computer");
            }
            catch (const std::exception& ex)
            {
                log_warning("failed to connect to sonar computer, "
                    "trying again in %.1f seconds...", retry_duration);
                ros::Duration(retry_duration).sleep();
            }

        } while (!socket.is_connected() && ros::ok());

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Initialize the 3DSS control software

        log_info("Initializing 3DSS control software...");
        socket.write(PINGDSP_INIT_SONAR_MODE());
        socket.write(PINGDSP_CONNECT());
        socket.write(PINGDSP_COMMIT());
        socket.write(PINGDSP_UPDATE_TIME());

        // Configure the sonar from the config file settings

        log_info("configuring Ping DSP sonar...");

        log_info("setting sonar range...");
        socket.write(PINGDSP_SET_RANGE(get_param<int>("~range")));

        log_info("setting sound speed..");
        socket.write(PINGDSP_SET_SOUND_SPEED(get_param<double>("~sound_speed")));

        log_info("setting sonar gain...");
        socket.write(PINGDSP_SET_GAIN(get_param<double>("~const_gain"),
                                      get_param<double>("~linear_gain"),
                                      get_param<double>("~log_gain")));


        log_info("configuring 3D sidescan...");
        socket.write(PINGDSP_SET_SIDESCAN_3D(get_param<int>("~angles"),
                                             get_param<int>("~smoothing"),
                                             get_param<double>("~threshold"),
                                             get_param<double>("~tolerance")));

        log_info("configuring bathymetry...");
        BathymetryConfig config;
        config.min_depth = get_param<double>("~min_depth");
        config.max_depth = get_param<double>("~max_depth");
        config.swath = get_param<double>("~swath");
        config.bin_count = get_param<int>("~bin_count");
        config.bin_width = get_param<double>("~bin_width");
        config.bottom_track_cells = get_param<int>("~bottom_track_cells");
        config.bottom_track_width = get_param<double>("~bottom_track_width");
        config.bottom_track_height = get_param<double>("~bottom_track_height");
        config.bottom_track_height_percent = get_param<double>("~bottom_track_height_percent");
        config.bottom_track_alpha = get_param<double>("~bottom_track_alpha");
        socket.write(PINGDSP_SET_BATHYMETRY(config));

        log_info("configuring transmit...");
        socket.write(PINGDSP_SET_TRANSMIT(get_param<std::string>("~pulse_type"),
                                          get_param<int>("~power"),
                                          get_param<int>("~beamwidth"),
                                          get_param<int>("~angle")));

        log_info("commiting settings changes...");
        socket.write(PINGDSP_COMMIT());

        log_info("sonar configuration complete");

        // TEMPORARY UDP *******************************************************

        // Create the client and server IP addresses
        using namespace boost::asio::ip;
        udp::resolver resolver(io_service);
		udp::resolver::query query(udp::v4(), sonar_computer_address, "23841");
		udp::resolver::iterator iter = resolver.resolve(query);
		udp_server_endpoint = *iter;

        // *********************************************************************

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
            io_service.poll_one();
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

        log_info("shutting down Ping DSP sonar...");

        // Stop the sonar
        socket.write(PINGDSP_STOP_RECORDING());
        socket.write(PINGDSP_STOP_DATA_STREAM());

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    PingdspSonarNode node(argc, argv);
    node.start();
    return 0;
}
