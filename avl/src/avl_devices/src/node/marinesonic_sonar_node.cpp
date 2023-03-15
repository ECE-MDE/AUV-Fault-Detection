//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to interface with the Marine Sonic Sea Scan embedded
//              sonar module. Starts sonar data collection to the sonar
//              module's disk when the node is started. Stops data collection
//              when the node is shut down.
//
// Servers:     device/enable_sonar (std_srvs/Trigger)
//              device/disable_sonar (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================

// Core utility
#include <avl_core/node.h>
#include <avl_asio/tcp_socket.h>
#include <avl_core/util/string.h>

// ROS services
#include <std_srvs/Trigger.h>

// Marine Sonic sonar commands
#include <avl_devices/protocol/marinesonic_sonar.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class MarinesonicSonarNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        MarinesonicSonarNode constructor
    //--------------------------------------------------------------------------
    MarinesonicSonarNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Sonar computer TCP client
    TcpSocket tcp_client;

    // Service servers for turning the sonar on and off
    ros::ServiceServer enable_sonar_server;
    ros::ServiceServer disable_sonar_server;

    // Sonar configurations for turning the sonar pinging on and off
    SonarConfig sonar_on_config;
    SonarConfig sonar_off_config;

private:

    //--------------------------------------------------------------------------
    // Name:        tcp_read_handler
    // Description: Handler for NMEA messages read from the sonar computer TCP
    //              client.
    // Arguments:   - data: vector of bytes read
    //--------------------------------------------------------------------------
    void tcp_read_handler(std::vector<uint8_t> data)
    {
        std::string line = std::string(data.begin(), data.end());
        avl::strip(line, '\r');
        avl::strip(line, '\n');
        log_debug("[rx] " + line);
    }

    //--------------------------------------------------------------------------
    // Name:        enable_sonar_srv_callback
    // Description: Called when the enable_sonar service is called. Enables
    //              sonar pinging and begins recording the data to the sonar
    //              module disk.
    // Arguments:   - req: request received on the service
    //              - res: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool enable_sonar_srv_callback(std_srvs::Trigger::Request& req,
                                   std_srvs::Trigger::Response& res)
    {
        log_debug("enable_sonar service called");
        tcp_client.write(MS_CONFIGURE_SONAR(sonar_on_config));
        tcp_client.write(MS_START_DISK_STREAM());
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        disable_sonar_srv_callback
    // Description: Called when the enable_sonar service is called. Stops
    //              writing sonar data to the disk and disables sonar pinging.
    // Arguments:   - req: request received on the service
    //              - res: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool disable_sonar_srv_callback(std_srvs::Trigger::Request& req,
                                     std_srvs::Trigger::Response& res)
    {
        log_debug("disable_sonar service called");
        tcp_client.write(MS_STOP_DISK_STREAM());
        tcp_client.write(MS_CONFIGURE_SONAR(sonar_off_config));
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Wait for the sonar computer to boot
        log_info("waiting for sonar computer to boot...");
        ros::Duration(get_param<double>("~boot_time")).sleep();
        log_info("sonar computer boot complete");

        // Set up the sonar computer TCP client. Match condition is newline for
        // NMEA formatted messages
        tcp_client.set_match(Match("\n", &MarinesonicSonarNode::tcp_read_handler, this));
        tcp_client.set_read_timeout(get_param<int>("~tcp/read_timeout"));
        tcp_client.connect(get_param<std::string>("~tcp/address"),
                           get_param<int>("~tcp/port"));

        // Configure the sonar using the sonar configuration command over TCP,
        // and pause for the sonar to set the settings

        // Set up the sonar configurations. The off configuration is used to
        // turn sonar pinging off. The on configuration starts pinging in manual
        // ping rate mode

        sonar_off_config.ping_mode = "OFF";
        sonar_off_config.ping_rate = get_param<float>("~ping_rate");
        sonar_off_config.max_range = get_param<float>("~max_range");
        sonar_off_config.min_range = get_param<float>("~min_range");
        sonar_off_config.num_samples = get_param<int>("~num_samples");
        sonar_off_config.sample_size = get_param<int>("~sample_size");
        sonar_off_config.left_transducer_index = get_param<int>("~left_transducer_index");
        sonar_off_config.right_transducer_index = get_param<int>("~right_transducer_index");

        sonar_on_config.ping_mode = "MANUAL";
        sonar_on_config.ping_rate = get_param<float>("~ping_rate");
        sonar_on_config.max_range = get_param<float>("~max_range");
        sonar_on_config.min_range = get_param<float>("~min_range");
        sonar_on_config.num_samples = get_param<int>("~num_samples");
        sonar_on_config.sample_size = get_param<int>("~sample_size");
        sonar_on_config.left_transducer_index = get_param<int>("~left_transducer_index");
        sonar_on_config.right_transducer_index = get_param<int>("~right_transducer_index");

        // Send the sonar off configuration and ensure the sonar is not writing
        // to the disk
        tcp_client.write(MS_CONFIGURE_SONAR(sonar_off_config));
        tcp_client.write(MS_STOP_DISK_STREAM());

        // Set up the service servers
        enable_sonar_server = node_handle->advertiseService("device/enable_sonar",
            &MarinesonicSonarNode::enable_sonar_srv_callback, this);
        disable_sonar_server = node_handle->advertiseService("device/disable_sonar",
            &MarinesonicSonarNode::disable_sonar_srv_callback, this);

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
            tcp_client.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function or when an exception is thrown
    //              in the init or run functions.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        tcp_client.write(MS_STOP_DISK_STREAM());
        ros::Duration(1.0).sleep();
        tcp_client.close();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    MarinesonicSonarNode node(argc, argv);
    node.start();
    return 0;
}
