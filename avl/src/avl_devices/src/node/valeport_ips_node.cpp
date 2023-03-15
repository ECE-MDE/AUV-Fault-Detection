//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node for pressure measurements from a Valeport MiniIPS
//              pressure sensor.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/pressure (std_msgs/Float64)
//
// Subscribers: None
//==============================================================================

// Base node class
#include <avl_core/node.h>

// ROS message includes
#include <std_msgs/Float64.h>

// Device driver
#include <avl_core/serial_device.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ValeportIpsNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ValeportIpsNode constructor
    //--------------------------------------------------------------------------
    ValeportIpsNode(int argc, char **argv) : Node(argc, argv)
    {
        
    }

private:

    // Publisher for depth data messages
    ros::Publisher depth_pub;

    SerialDevice ips;

private:


    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        log_info("initializing Valeport MiniIPS...");

        // Open the connection to the Valeport MiniIPS
        ips.open(get_param<std::string>("~serial/port_name"),
                 get_param<int>("~serial/baudrate"));
        ips.set_read_timeout(get_param<int>("~serial/read_timeout"));
        ips.flush();

        // Set up the publisher for depth data
        depth_pub = node_handle->advertise<std_msgs::Float64>("depth", 1);


        log_info("configuring Valeport MiniIPS...");

        // Configure the Valeport MiniIps. Pauses were added because the
        // device is slow...
        float pause_time_sec = 0.5;

        // ips.enableSetupMode(true);
        // log_info("enabled setup mode");
        // ros::Duration(pause_time_sec).sleep();

        // ips.setUnits(PRES_PSI);
        // log_info("units set to PSI");
        // ros::Duration(pause_time_sec).sleep();

        // ips.setDatarate((DataRate)get_param<int>("~data_rate"));
        // log_info("data rate set to %d Hz", get_param<int>("~data_rate"));
        // ros::Duration(pause_time_sec).sleep();

        // ips.enableSetupMode(false);
        // log_info("disabled setup mode");
        // ros::Duration(pause_time_sec).sleep();
        // ips.flush();

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //              Reads IMU data and publishes it to the imu topic.
    //--------------------------------------------------------------------------
    void run()
    {

        log_info("parsing depth messages...");

        while (ros::ok())
        {

            ros::spinOnce();
            spin_rate.sleep();

            // Read in the pressure data packet
            //float pressure = ips.getPressure();
            float pressure = -1.0;

            log_data("Pressure: +%.3f PSI", pressure);

            // Create and publish the depth message
            std_msgs::Float64 depth_msg;
            depth_msg.data = pressure;
            depth_pub.publish(depth_msg);

        }

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    ValeportIpsNode node(argc, argv);
    node.start();
    return 0;
}
