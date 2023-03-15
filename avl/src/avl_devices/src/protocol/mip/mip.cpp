//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides MIP command packets for configuring a Microstrain AHRS
//              and a function to parse AHRS data into an AHRS message.
//==============================================================================

#include <avl_devices/protocol/mip/mip.h>

// Core utility
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>
#include <avl_core/util/math.h>

// Eigen includes
#include <Eigen/Core>
using namespace Eigen;

// MIP packet class
#include <avl_devices/protocol/mip/mip_packet.h>

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        parse_ahrs_data
// Description: Parses an AHRS data structure from a MIP packet.
// Arguments:   - packet: MIP packet to be parsed
// Returns:     AHRS data structure containing Euler angle, gyroscope,
//              accelerometer, and magnetometer measurements
//------------------------------------------------------------------------------
AhrsMsg parse_ahrs_data(MipPacket packet)
{

    // Ensure the MIP packet consists of AHRS data
    if (packet.get_descriptor() != IMU_DATA)
    {
        throw std::runtime_error("parse_ahrs_data: attempted to parse AHRS data"
            " from incorrect MIP packet type");
    }

    // Parse the MIP packet into an AHRS message
    AhrsMsg ahrs_msg;

    try
    {

        // Extract the sensor fields from the MIP packet
        MipField euler_field = packet.get_field(IMU_CF_EULER_ANGLES);
        MipField accel_field = packet.get_field(IMU_SCALED_ACCEL);
        MipField gyro_field = packet.get_field(IMU_SCALED_GYRO);
        MipField mag_field = packet.get_field(IMU_SCALED_MAG);

        // Convert sensor field data from bytes to floats
        ahrs_msg.theta.x = avl::from_bytes<float>(
            avl::subvector(euler_field.get_data(), 0, 4));
        ahrs_msg.theta.y = avl::from_bytes<float>(
            avl::subvector(euler_field.get_data(), 4, 4));
        ahrs_msg.theta.z = avl::from_bytes<float>(
            avl::subvector(euler_field.get_data(), 8, 4));

        ahrs_msg.a.x = avl::from_bytes<float>(
            avl::subvector(accel_field.get_data(), 0, 4));
        ahrs_msg.a.y = avl::from_bytes<float>(
            avl::subvector(accel_field.get_data(), 4, 4));
        ahrs_msg.a.z = avl::from_bytes<float>(
            avl::subvector(accel_field.get_data(), 8, 4));

        ahrs_msg.w.x = avl::from_bytes<float>(
            avl::subvector(gyro_field.get_data(), 0, 4));
        ahrs_msg.w.y = avl::from_bytes<float>(
            avl::subvector(gyro_field.get_data(), 4, 4));
        ahrs_msg.w.z = avl::from_bytes<float>(
            avl::subvector(gyro_field.get_data(), 8, 4));

        ahrs_msg.mag.x = avl::from_bytes<float>(
            avl::subvector(mag_field.get_data(), 0, 4));
        ahrs_msg.mag.y = avl::from_bytes<float>(
            avl::subvector(mag_field.get_data(), 4, 4));
        ahrs_msg.mag.z = avl::from_bytes<float>(
            avl::subvector(mag_field.get_data(), 8, 4));

        // Wrap the heading to 2pi radians
        ahrs_msg.theta.z = avl::wrap_to_2pi(ahrs_msg.theta.z);

        // Convert the acceleration from g's to m/s^2
        ahrs_msg.a.x = ahrs_msg.a.x * 9.80665;
        ahrs_msg.a.y = ahrs_msg.a.y * 9.80665;
        ahrs_msg.a.z = ahrs_msg.a.z * 9.80665;

    }
    catch (const std::exception& ex)
    {
        throw std::runtime_error("parse_ahrs_data: failed to parse IMU data MIP"
            " packet");
    }

    return ahrs_msg;

}

//------------------------------------------------------------------------------
// Name:            AHRS_PING
// Description:     MIP command to ping the AHRS, prompting it to respond with a
//                  ping response message.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_PING()
{
    MipPacket packet;
    packet.set_descriptor(BASE_COMMAND);
    packet.add_field(BASE_PING);
    return packet.get_bytes();
}

//------------------------------------------------------------------------------
// Name:            AHRS_SET_IDLE
// Description:     MIP command to set the AHRS to idle mode.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_IDLE()
{
    MipPacket packet;
    packet.set_descriptor(BASE_COMMAND);
    packet.add_field(BASE_SET_IDLE);
    return packet.get_bytes();
}

//------------------------------------------------------------------------------
// Name:            AHRS_RESET
// Description:     MIP command to reset the AHRS.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_RESET()
{
    MipPacket packet;
    packet.set_descriptor(BASE_COMMAND);
    packet.add_field(BASE_DEVICE_RESET);
    return packet.get_bytes();
}

//------------------------------------------------------------------------------
// Name:            AHRS_ENABLE_MAG_AUTO_CALIBRATION
// Description:     MIP command to enable magneometer hard iron and soft iron
//                  auto calibration.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_ENABLE_MAG_AUTO_CALIBRATION()
{
    MipPacket packet;
    packet.set_descriptor(ESTIMATION_FILTER_COMMAND);
    packet.add_field(ESTIMATION_CONTROL_FLAGS, {0x01, 0x06, 0x00});
    return packet.get_bytes();
}

//------------------------------------------------------------------------------
// Name:            AHRS_CAPTURE_MAG_AUTO_CALIBRATION
// Description:     MIP command to capture magneometer hard iron and soft iron
//                  auto calibration values.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_CAPTURE_MAG_AUTO_CALIBRATION()
{
    MipPacket packet;
    packet.set_descriptor(ESTIMATION_FILTER_COMMAND);
    packet.add_field(ESTIMATION_CAPTURE_AUTO_CALIBRATION, {0x03});
    return packet.get_bytes();
}

//------------------------------------------------------------------------------
// Name:            AHRS_SET_HARD_IRON
// Description:     MIP command to set the magnetometer hard iron vector.
// Arguments:       - b: Hard iron calibration vector.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_HARD_IRON(Vector3f b)
{

    // Check for an invalid hard iron vector
    if (std::isnan(b.sum()))
        throw std::runtime_error("AHRS_SET_HARD_IRON: invalid b vector");

    // Set up a 3DM command MIP packet
    MipPacket packet;
    packet.set_descriptor(TDM_COMMAND);

    // Vector to hold bytes representing the matrix elements. The first element
    // indicates that we're writing the settings
    std::vector<uint8_t> field_data = { GLOBAL_WRITE };

    // Turn the 3 hard iron floats into an array of bytes and append it to the
    // field bytes
    for(size_t i = 0; i < 3; i++)
        avl::append(field_data, avl::to_bytes(b(i)));

    // Add the field to the packet
    packet.add_field(TDM_MAGNETOMETER_HARD_IRON, field_data);

    return packet.get_bytes();

}

//------------------------------------------------------------------------------
// Name:            AHRS_GET_HARD_IRON
// Description:     MIP command to get the current magnetometer hard iron
//                  vector.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_GET_HARD_IRON()
{

    // Set up a 3DM command MIP packet
    MipPacket packet;
    packet.set_descriptor(TDM_COMMAND);

    // Vector to hold bytes representing the matrix elements. The first element
    // indicates that we're saving the settings
    std::vector<uint8_t> field_data = { GLOBAL_READ };

    // Turn the 3 hard iron floats into an array of bytes and append it to the
    // field bytes. These values are ignored when using "save current settings
    // as startup settings" mode
    Vector3f b = {0.0, 0.0, 0.0};
    for(size_t i = 0; i < 3; i++)
        avl::append(field_data, avl::to_bytes(b(i)));

    // Add the field to the packet
    packet.add_field(TDM_MAGNETOMETER_HARD_IRON, field_data);

    return packet.get_bytes();

}

//------------------------------------------------------------------------------
// Name:            AHRS_SAVE_HARD_IRON
// Description:     MIP command to save the current magnetometer hard iron
//                  vector as the startup value.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SAVE_HARD_IRON()
{

    // Set up a 3DM command MIP packet
    MipPacket packet;
    packet.set_descriptor(TDM_COMMAND);

    // Vector to hold bytes representing the matrix elements. The first element
    // indicates that we're saving the settings
    std::vector<uint8_t> field_data = { GLOBAL_SAVE };

    // Turn the 3 hard iron floats into an array of bytes and append it to the
    // field bytes. These values are ignored when using "save current settings
    // as startup settings" mode
    Vector3f b = {0.0, 0.0, 0.0};
    for(size_t i = 0; i < 3; i++)
        avl::append(field_data, avl::to_bytes(b(i)));

    // Add the field to the packet
    packet.add_field(TDM_MAGNETOMETER_HARD_IRON, field_data);

    return packet.get_bytes();

}

//------------------------------------------------------------------------------
// Name:            AHRS_SET_SOFT_IRON
// Description:     MIP command to set the magnetometer soft iron matrix.
// Arguments:       - A: Soft iron calibration matrix.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_SOFT_IRON(Matrix3f A)
{

    // Check for an invalid soft iron matrix
    if (std::isnan(A.sum()))
        throw std::runtime_error("AHRS_SET_SOFT_IRON: invalid A matrix");

    // Take the transpose of A since Eigen stores data in column-major format
    // but the AHRS wants row-major
    A.transposeInPlace();

    // Set up a 3DM command MIP packet
    MipPacket packet;
    packet.set_descriptor(TDM_COMMAND);

    // Vector to hold bytes representing the matrix elements. The first element
    // indicates that we're writing the settings
    std::vector<uint8_t> field_data = { GLOBAL_WRITE };

    // Turn the 9 soft iron floats into an array of bytes and append it to the
    // field bytes
    for(size_t i = 0; i < 9; i++)
        avl::append(field_data, avl::to_bytes(A(i)));

    // Add the field to the packet
    packet.add_field(TDM_MAGNETOMETER_SOFT_IRON, field_data);

    return packet.get_bytes();

}

//------------------------------------------------------------------------------
// Name:            AHRS_GET_SOFT_IRON
// Description:     MIP command to get the current magnetometer soft iron
//                  matrix.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_GET_SOFT_IRON()
{

    // Take the transpose of A since Eigen stores data in column-major format
    // but the AHRS wants row-major
    Matrix3f A = Matrix3f::Identity();
    A.transposeInPlace();

    // Set up a 3DM command MIP packet
    MipPacket packet;
    packet.set_descriptor(TDM_COMMAND);

    // Vector to hold bytes representing the matrix elements. The first element
    // indicates that we're saving the settings
    std::vector<uint8_t> field_data = { GLOBAL_READ };

    // Turn the 9 soft iron floats into an array of bytes and append it to the
    // field bytes. These values are ignored when using "save current settings
    // as startup settings" mode
    for(size_t i = 0; i < 9; i++)
        avl::append(field_data, avl::to_bytes(A(i)));

    // Add the field to the packet
    packet.add_field(TDM_MAGNETOMETER_SOFT_IRON, field_data);

    return packet.get_bytes();

}

//------------------------------------------------------------------------------
// Name:            AHRS_SAVE_SOFT_IRON
// Description:     MIP command to save the current magnetometer soft iron
//                  matrix as the startup value.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SAVE_SOFT_IRON()
{

    // Take the transpose of A since Eigen stores data in column-major format
    // but the AHRS wants row-major
    Matrix3f A = Matrix3f::Identity();
    A.transposeInPlace();

    // Set up a 3DM command MIP packet
    MipPacket packet;
    packet.set_descriptor(TDM_COMMAND);

    // Vector to hold bytes representing the matrix elements. The first element
    // indicates that we're saving the settings
    std::vector<uint8_t> field_data = { GLOBAL_SAVE };

    // Turn the 9 soft iron floats into an array of bytes and append it to the
    // field bytes. These values are ignored when using "save current settings
    // as startup settings" mode
    for(size_t i = 0; i < 9; i++)
        avl::append(field_data, avl::to_bytes(A(i)));

    // Add the field to the packet
    packet.add_field(TDM_MAGNETOMETER_SOFT_IRON, field_data);

    return packet.get_bytes();

}

//------------------------------------------------------------------------------
// Name:            AHRS_SET_IMU_MESSAGE_FORMAT
// Description:     Creates a vector of bytes representing the command to
//                  set the ahrs to return euler angles, angular velocities,
//                  accelerations, and magnetometer data all at 50 Hz
// Returns:         The bytes of the mip command that will configure the
//                  ahrs IMU
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_IMU_MESSAGE_FORMAT()
{

    // Set up a 3DM command MIP packet
    MipPacket packet;
    packet.set_descriptor(TDM_COMMAND);

    // Vector to hold the field data
    std::vector<uint8_t> field_data =
    {
        0x01, 0x04,        // Use new settings 0x01. 4 total settings
        0x0C, 0x00, 0x0A,  // Complementary Filter Euler Angles at 100 Hz
        0x05, 0x00, 0x0A,  // Angular velocities at 100 Hz
        0x04, 0x00, 0x0A,  // Accelerations at 100 Hz
        0x06, 0x00, 0x0A   // Magnetometer at 100 Hz
    };

    // Add the field to the packet
    packet.add_field(TDM_IMU_MESSAGE_FORMAT, field_data);

    return packet.get_bytes();

}

//------------------------------------------------------------------------------
// Name:            AHRS_ENABLE_IMU_STREAM
// Description:     MIP command to enable continuous IMU data streaming
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_ENABLE_IMU_STREAM()
{

    // Set up a 3DM command MIP packet
    MipPacket packet;
    packet.set_descriptor(TDM_COMMAND);

    // Vector to hold the field data. The first element indicates that we're
    // writing the settings, the second inducates the IMU stream, and the third
    // indicates enable
    std::vector<uint8_t> field_data = { GLOBAL_WRITE, 0x01, 0x01 };

    // Add the field to the packet
    packet.add_field(TDM_ENABLE_DATA_STREAM, field_data);

    // The first element indicates that we're writing the settings, the second
    // inducates the estimation filter stream, and the third indicates disable
    field_data = { GLOBAL_WRITE, 0x03, 0x00 };

    // Add the field to the packet
    packet.add_field(TDM_ENABLE_DATA_STREAM, field_data);

    return packet.get_bytes();

}

//------------------------------------------------------------------------------
// Name:            AHRS_SET_SENSOR_TO_VEHICLE_TRANSFORM
// Description:     MIP command to set the AHRS sensor to vehicle frame Euler
//                  angles.
// Arguments:       - euler_angles: sensor to vehicle frame Euler angles in
//                    degrees
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_SENSOR_TO_VEHICLE_TRANSFORM(
    std::vector<float> euler_angles)
{

    // Set up an estimation filter command MIP packet
    MipPacket packet;
    packet.set_descriptor(ESTIMATION_FILTER_COMMAND);

    // Vector to hold the field data. The first element indicates that we're
    // writing the settings
    std::vector<uint8_t> field_data = { GLOBAL_WRITE };

    // Convert the angles to radians, and then to an array of bytes and add them
    // to the field data
    for(size_t i = 0; i < 3; i++)
        avl::append(field_data, avl::to_bytes(
            avl::deg_to_rad(euler_angles.at(i))));

    // Add the field to the packet
    packet.add_field(ESTIMATION_SENSOR_TO_VEHICLE_TRANSFORM, field_data);

    return packet.get_bytes();

}
