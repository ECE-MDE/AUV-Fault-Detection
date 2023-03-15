//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides command strings to configure a KVH IMU and parse its
//              data over RS232.
//==============================================================================

#ifndef KVH_IMU_H
#define KVH_IMU_H

// Utility functions
#include <avl_core/util/byte.h>
#include <avl_core/util/vector.h>

// ROS messages
#include <avl_msgs/ImuMsg.h>
using namespace avl_msgs;

//==============================================================================
//                            VARIABLE DEFINITIONS
//==============================================================================

// IMU data message header
const std::vector<uint8_t> KVH_DATA_HEADER = { 0xFE, 0x81, 0xFF, 0x55 };

// IMU built-in self-test data header
const std::vector<uint8_t> KVH_BIT2_HEADER = { 0xFE, 0x81, 0x00, 0xAB };

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        parse_imu_data
// Description: Parses an IMU data message into an IMU message. NOTE: Data
//              should not include the header bytes.
// Arguments:   - data: IMU data to be parsed.
//              - dt: Sampling period in seconds.
// Returns:     ImuData message containing IMU measurement info.
//------------------------------------------------------------------------------
inline ImuMsg parse_imu_data(std::vector<uint8_t> data, double dt)
{

    // Get the bytes that represent the IMU data values and convert them to
    // their proper units. The vector must be reversed for proper endianness
    float rot_x = avl::from_bytes<float>(avl::subvector(data,0,4));
    float rot_y = avl::from_bytes<float>(avl::subvector(data,4,4));
    float rot_z = avl::from_bytes<float>(avl::subvector(data,8,4));
    float acc_x = avl::from_bytes<float>(avl::subvector(data,12,4));
    float acc_y = avl::from_bytes<float>(avl::subvector(data,16,4));
    float acc_z = avl::from_bytes<float>(avl::subvector(data,20,4));
    int16_t temperature = avl::from_bytes<uint16_t>(avl::subvector(data,26,2));
    bool data_valid = (data.at(24) == 0b01110111);

    // Convert Gs to m/s^2
    acc_x = acc_x * 9.80665;
    acc_y = acc_y * 9.80665;
    acc_z = acc_z * 9.80665;

    // Put the parsed IMU data into the IMU message
    ImuMsg imu_msg;
    imu_msg.dt = dt;
    imu_msg.angular_velocity.x = data_valid ? rot_x : NAN;
    imu_msg.angular_velocity.y = data_valid ? rot_y : NAN;
    imu_msg.angular_velocity.z = data_valid ? rot_z : NAN;
    imu_msg.linear_acceleration.x = data_valid ? acc_x : NAN;
    imu_msg.linear_acceleration.y = data_valid ? acc_y : NAN;
    imu_msg.linear_acceleration.z = data_valid ? acc_z : NAN;
    imu_msg.temperature = temperature;
    imu_msg.valid = data_valid;

    return imu_msg;

}

//------------------------------------------------------------------------------
// Name:        KVH_ENABLE_CONFIG_MODE
// Description: Enables or disables IMU configuration mode. This mode must be
//              enabled before settings can be changed.
// Arguments:   - enable: true to enter config mode, false to leave config mode
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_ENABLE_CONFIG_MODE(bool enable)
{
    char command[128];
    sprintf(command, "=CONFIG,%d\r\n", (int)enable);
    return std::string(command);
}

//------------------------------------------------------------------------------
// Name:        KVH_RESET_SETTINGS
// Description: Resets all settings to their factory defaults.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_RESET_SETTINGS()
{
    return std::string("=rstcfg\r\n");
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_BAUDRATE
// Description: Command to set the IMU RS485 baudrate in bits per second. The
//              following are valid values:
//                  9600, 19200, 38400, 57600, 115200, 460800, 576000, 921600
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_BAUDRATE(int baudrate)
{
    char command[128];
    sprintf(command, "=BAUD,%d\r\n", baudrate);
    return std::string(command);
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_DATARATE
// Description: Command to set the IMU output datarate in Hz. The following are
//              valid values:
//                  1, 5, 10, 25, 50, 100, 250, 500, 750, 1000
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_DATA_RATE(int datarate)
{
    char command[128];
    sprintf(command, "=DR,%d\r\n", (int)datarate);
    return std::string(command);
}

//------------------------------------------------------------------------------
// Name:        KVH_ENABLE_FILTERING
// Description: Command to enable or disable IMU output filtering.
// Arguments:   - enable: true to enable output filtering, false to disable.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_ENABLE_FILTERING(bool enable)
{
    char command[128];
    sprintf(command, "=FILTEN,%d\r\n", (int)enable);
    return std::string(command);
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_ROTATION_MATRIX
// Description: Command to set a rotation matrix that will rotate all output
//              measurements. Can be used to align the instrument axes to a
//              set of vehicle axes.
// Arguments:   - matrix: 9 element vector defining a row major rotation matrix.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_ROTATION_MATRIX(std::vector<double> matrix)
{

    // The vector must have exactly 9 elements representing a 3x3 matrix
    if (matrix.size() == 9)
    {

        char command[128];
        sprintf(command, "=AXES,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
            matrix.at(0), matrix.at(1), matrix.at(2),
            matrix.at(3), matrix.at(4), matrix.at(5),
            matrix.at(6), matrix.at(7), matrix.at(8));

        return std::string(command);

    }
    else
    {
        throw std::runtime_error("vector representing rotation matrix must have"
            " 9 elements");
    }

}

//------------------------------------------------------------------------------
// Name:        KVH_SET_TEMP_UNITS
// Description: Command to set the IMU temperature units.
//                  C: Celsius
//                  F: Fahrenheit
// Arguments:   - units: string representing units choice
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_TEMP_UNITS(std::string units)
{
    return "=TEMPUNITS," + units + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_GYRO_UNITS
// Description: Command to set the IMU gyroscope units.
//                  DEG: deg/s (or degrees for delta output format)
//                  RAD: rad/s (or radians for delta output format)
// Arguments:   - units: string representing units choice
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_GYRO_UNITS(std::string units)
{
    return "=ROTUNITS," + units + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_GYRO_FORMAT
// Description: Command to set the IMU gyroscope output format.
//                  DELTA: Change in angle output
//                  RATE:  Angular velocity output
// Arguments:   - format: string representing format choice
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_GYRO_FORMAT(std::string format)
{
    return "=ROTFMT," + format + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_GYRO_FILTER_TYPE
// Description: Command to set the IMU gyroscope output filter type.
//                  CHEBY:  Chebychev type II low-pass
//                  BUTTER: Butterworth low-pass
//                  AVE:    uniform averager
// Arguments:   - type: string representing filter type choice
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_GYRO_FILTER_TYPE(std::string type)
{
    return "=FILTTYPE,G," + type + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_ACCEL_UNITS
// Description: Command to set the IMU accelerometer units.
//                  METERS: m/s^2 (or m/s for delta output format)
//                  FEET:  ft/s (or feet for delta output format)
// Arguments:   - units: string representing units choice
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_ACCEL_UNITS(std::string units)
{
    return "=LINUNITS," + units + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_ACCEL_FORMAT
// Description: Command to set the IMU accelerometer output format.
//                  DELTA: Change in position output
//                  ACCEL: Linear acceleration output
// Arguments:   - format: string representing format choice
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_ACCEL_FORMAT(std::string format)
{
    return "=LINFMT," + format + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        KVH_SET_ACCEL_FILTER_TYPE
// Description: Command to set the IMU accelerometer output filter type.
//                  CHEBY:  Chebychev type II low-pass
//                  BUTTER: Butterworth low-pass
//                  AVE:    uniform averager
// Arguments:   - type: string representing filter type choice
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string KVH_SET_ACCEL_FILTER_TYPE(std::string type)
{
    return "=FILTTYPE,A," + type + "\r\n";
}

#endif // KVH_IMU_H
