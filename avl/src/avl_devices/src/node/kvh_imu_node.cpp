//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node for IMU measurements from the KVH 1725 IMU. The IMU
//              measures and outputs angular rate data from it's fiber optic
//              gyroscope and linear acceleration data from its MEMS
//              accelerometer.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/imu (avl_msgs/ImuMsg)
//
// Subscribers: None
//==============================================================================

// Base node class
#include <avl_core/node.h>
#include <avl_asio/serial_port.h>
#include <avl_core/util/string.h>
#include <avl_core/util/misc.h>

// ROS message includes
#include <avl_msgs/ImuMsg.h>
using namespace avl_msgs;

// Device driver
#include <avl_devices/protocol/kvh_imu.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class KvhImuNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        KvhImuNode constructor
    //--------------------------------------------------------------------------
    KvhImuNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publisher for IMU data messages
    ros::Publisher imu_pub;

    // Serial port instance
    SerialPort serial;

    // Sampling period from config file
    double dt;

private:

    //--------------------------------------------------------------------------
    // Name:        status_to_string
    // Description: Converts KVH built-in test bytes to a PASS or FAIL string
    //              based on the given byte and bit numbers.
    // Arguments:   - data: BIT2 data bytes
    //              - byte_number: BIT2 data byte number to check for pass or
    //                fail
    //              - bit_num: bit number in the given byte to check for pass
    //                or fail
    //--------------------------------------------------------------------------
    const char* status_to_string(std::vector<uint8_t> data, size_t byte_num,
        size_t bit_num)
    {
        uint8_t status_bit = data.at(byte_num) >> bit_num & 0x01;
        if (status_bit == 0x01)
            return "\033[92mPASS";
        else
            return "\033[91mFAIL";
    }

    //--------------------------------------------------------------------------
    // Name:        bit2_header_read_handler
    // Description: Called when an IMU built-in test 2 header is read.
    // Arguments:   - data: BIT2 header bytes
    //--------------------------------------------------------------------------
    void bit2_header_read_handler(std::vector<uint8_t> data)
    {

        log_debug("received BIT2 header");

        // Set the match condition to the next 9 bytes of the fixed length
        // build-in test data message with the callback as the data read handler
        serial.set_match(Match(9, &KvhImuNode::bit2_data_read_handler, this));

    }

    //--------------------------------------------------------------------------
    // Name:        bit2_data_read_handler
    // Description: Called when an IMU built-in test data is read.
    // Arguments:   - data: BIT data bytes
    //--------------------------------------------------------------------------
    void bit2_data_read_handler(std::vector<uint8_t> data)
    {

        log_debug("received BIT2 data");

        log_info("================================================================================");
        log_info("Built-in Test Results:");
        log_info("Gyro X SLD:         %s", status_to_string(data, 0, 0));
        log_info("Gyro X MODDAC:      %s", status_to_string(data, 0, 1));
        log_info("Gyro X Phase:       %s", status_to_string(data, 0, 2));
        log_info("Gyro X Flash:       %s", status_to_string(data, 0, 3));
        log_info("Gyro Y SLD:         %s", status_to_string(data, 0, 4));
        log_info("Gyro Y MODDAC:      %s", status_to_string(data, 0, 5));
        log_info("Gyro Y Phase:       %s", status_to_string(data, 0, 6));

        log_info("Gyro Y Flash:       %s", status_to_string(data, 1, 0));
        log_info("Gyro Z SLD:         %s", status_to_string(data, 1, 1));
        log_info("Gyro Z MODDAC:      %s", status_to_string(data, 1, 2));
        log_info("Gyro Z Phase:       %s", status_to_string(data, 1, 3));
        log_info("Gyro Z Flash:       %s", status_to_string(data, 1, 4));
        log_info("Accel X:            %s", status_to_string(data, 1, 5));
        log_info("Accel Y:            %s", status_to_string(data, 1, 6));

        log_info("Accel Z:            %s", status_to_string(data, 2, 0));
        log_info("Gyro X SLD Temp:    %s", status_to_string(data, 2, 2));
        log_info("Gyro Y SLD Temp:    %s", status_to_string(data, 2, 4));
        log_info("Gyro Z SLD Temp:    %s", status_to_string(data, 2, 6));

        log_info("Accel X Temp:       %s", status_to_string(data, 3, 0));
        log_info("Accel Y Temp:       %s", status_to_string(data, 3, 1));
        log_info("Accel Z Temp:       %s", status_to_string(data, 3, 2));
        log_info("GCB Temp:           %s", status_to_string(data, 3, 3));
        log_info("IMU Temp:           %s", status_to_string(data, 3, 4));
        log_info("GCB DSP SPI Flash:  %s", status_to_string(data, 3, 5));
        log_info("GCB FPGA SPI Flash: %s", status_to_string(data, 3, 6));

        log_info("IMU SDP SPI Flash:  %s", status_to_string(data, 4, 0));
        log_info("IMU FPGA SPI Flash: %s", status_to_string(data, 4, 1));
        log_info("GCB 1.2V:           %s", status_to_string(data, 4, 2));
        log_info("GCB 3.3V:           %s", status_to_string(data, 4, 3));
        log_info("GCB 5V:             %s", status_to_string(data, 4, 4));
        log_info("IMU 1.2V:           %s", status_to_string(data, 4, 5));
        log_info("IMU 3.3V:           %s", status_to_string(data, 4, 6));

        log_info("IMU 5V:             %s", status_to_string(data, 5, 0));
        log_info("GCB FPGA:           %s", status_to_string(data, 5, 2));
        log_info("IMU FPGA:           %s", status_to_string(data, 5, 3));
        log_info("Hi-Speed SPORT:     %s", status_to_string(data, 5, 4));
        log_info("Aux SPORT:          %s", status_to_string(data, 5, 5));
        log_info("Suff. Resources:    %s", status_to_string(data, 5, 6));

        log_info("Gyro EO Volts Pos.: %s", status_to_string(data, 6, 0));
        log_info("Gyro EO Volts Neg.: %s", status_to_string(data, 6, 1));
        log_info("Gyro X Volts:       %s", status_to_string(data, 6, 2));
        log_info("Gyro Y Volts:       %s", status_to_string(data, 6, 3));
        log_info("Gyro Z Volts:       %s", status_to_string(data, 6, 4));

        log_info("GCB ADC Comms:      %s", status_to_string(data, 7, 0));
        log_info("MSYNC Ext. Timing:  %s", status_to_string(data, 7, 1));

        // log_info("IMU 5V: ");
        log_info("================================================================================");

        auto kvh_data_match = Match(KVH_DATA_HEADER,
            &KvhImuNode::data_header_read_handler, this);
        auto kvh_bit2_match = Match(KVH_BIT2_HEADER,
            &KvhImuNode::bit2_header_read_handler, this);
        serial.set_match({kvh_data_match, kvh_bit2_match});

    }

    //--------------------------------------------------------------------------
    // Name:        data_header_read_handler
    // Description: Called when an IMU message header is read.
    // Arguments:   - data: IMU header bytes
    //--------------------------------------------------------------------------
    void data_header_read_handler(std::vector<uint8_t> data)
    {

        // Set the match condition to the next 28 bytes of the fixed length
        // data message with the callback as the data read handler
        serial.set_match(Match(28, &KvhImuNode::data_read_handler, this));

    }

    //--------------------------------------------------------------------------
    // Name:        data_read_handler
    // Description: Called when IMU message data is read.
    // Arguments:   - data: IMU data bytes
    //--------------------------------------------------------------------------
    void data_read_handler(std::vector<uint8_t> data)
    {

        // Parse the data into an ImuMsg and publish it if valid
        ImuMsg imu_msg = parse_imu_data(data, dt);
        if (imu_msg.valid)
            imu_pub.publish(imu_msg);

        // Log the data
        log_data("[imu] %+.9f %+.9f %+.9f %+.9f %+.9f %+.9f %d %d",
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
            imu_msg.temperature,
            static_cast<int>(imu_msg.valid));

        // Set the match condition back to the KVH data message header to read
        // the next data message
        auto kvh_data_match = Match(KVH_DATA_HEADER,
            &KvhImuNode::data_header_read_handler, this);
        auto kvh_bit2_match = Match(KVH_BIT2_HEADER,
            &KvhImuNode::bit2_header_read_handler, this);
        serial.set_match({kvh_data_match, kvh_bit2_match});

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log the data headers
        log_data("[imu] \\omega_x \\omega_y \\omega_z f_x f_y f_z temp valid");
        log_data("[imu] rad/s rad/s rad/s m/s^2 m/s^2 m/s^2 F bool");

        // Open the serial port
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set up the publisher for IMU data
        imu_pub = node_handle->advertise<ImuMsg>("device/imu", 1);

        // Configure the IMU from the config file settings by entering config
        // mode, sending the configuration commands, and disabling config mode
        dt = 1.0 / static_cast<double>(get_param<int>("~kvh/datarate"));
        serial.write(KVH_ENABLE_CONFIG_MODE(true));
        serial.write(KVH_SET_BAUDRATE(get_param<int>("~kvh/baudrate")));
        serial.write(KVH_SET_DATA_RATE(get_param<int>("~kvh/datarate")));
        serial.write(KVH_ENABLE_FILTERING(get_param<bool>("~kvh/enable_filtering")));
        serial.write(KVH_SET_ROTATION_MATRIX(get_param<std::vector<double>>("~kvh/rotation_matrix")));
        serial.write(KVH_SET_TEMP_UNITS(get_param<std::string>("~kvh/temp_units")));
        serial.write(KVH_SET_GYRO_UNITS("RAD"));
        serial.write(KVH_SET_GYRO_FORMAT("RATE"));
        serial.write(KVH_SET_GYRO_FILTER_TYPE(get_param<std::string>("~kvh/gyro_filter_type")));
        serial.write(KVH_SET_ACCEL_UNITS("METERS"));
        serial.write(KVH_SET_ACCEL_FORMAT("ACCEL"));
        serial.write(KVH_SET_ACCEL_FILTER_TYPE(get_param<std::string>("~kvh/accel_filter_type")));
        serial.write(KVH_ENABLE_CONFIG_MODE(false));
        serial.write("?bit,2\r\n");

        // Set the match condition to the KVH data message header to read data
        // messages
        auto kvh_data_match = Match(KVH_DATA_HEADER, &KvhImuNode::data_header_read_handler, this);
        auto kvh_bit2_match = Match(KVH_BIT2_HEADER, &KvhImuNode::bit2_header_read_handler, this);
        serial.set_match({kvh_data_match, kvh_bit2_match});

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //              Reads IMU data and publishes it to the imu topic.
    //--------------------------------------------------------------------------
    void run()
    {

        ros::Rate spin_rate(10000);
        while (ros::ok())
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
    KvhImuNode node(argc, argv);
    node.start();
    return 0;
}
