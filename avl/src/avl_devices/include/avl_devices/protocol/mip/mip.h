//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides MIP command packets for configuring a Microstrain AHRS
//              and a function to parse AHRS data into an AHRS message.
//==============================================================================

#ifndef MIP_H
#define MIP_H

// Core utility
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>

// Eigen includes
#include <Eigen/Core>
using namespace Eigen;

// MIP packet class
#include <avl_devices/protocol/mip/mip_packet.h>

// ROS messages
#include <avl_msgs/AhrsMsg.h>
using namespace avl_msgs;

//==============================================================================
//                             MIP COMMAND MAPPING
//==============================================================================

// Global field defines
#define GLOBAL_WRITE       0x01
#define GLOBAL_READ        0x02
#define GLOBAL_SAVE        0x03
#define GLOBAL_ACKNOWLEDGE 0xF1

// Base command set
#define BASE_COMMAND      0x01
#define BASE_PING         0x01
#define BASE_SET_IDLE     0x02
#define BASE_DEVICE_RESET 0x7E

// 3DM command set
#define TDM_COMMAND                0x0C
#define TDM_IMU_MESSAGE_FORMAT     0x08
#define TDM_ENABLE_DATA_STREAM     0x11
#define TDM_MAGNETOMETER_HARD_IRON 0x3A
#define TDM_MAGNETOMETER_SOFT_IRON 0x3B

// Estimation filter command set
#define ESTIMATION_FILTER_COMMAND              0x0D
#define ESTIMATION_SENSOR_TO_VEHICLE_TRANSFORM 0x11
#define ESTIMATION_CONTROL_FLAGS               0x14
#define ESTIMATION_CAPTURE_AUTO_CALIBRATION    0x27

// IMU data command set
#define IMU_DATA            0x80
#define IMU_SCALED_GYRO     0x05
#define IMU_SCALED_ACCEL    0x04
#define IMU_SCALED_MAG      0x06
#define IMU_CF_EULER_ANGLES 0x0C
#define IMU_SCALED_PRESSURE 0x17

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
AhrsMsg parse_ahrs_data(MipPacket packet);

//------------------------------------------------------------------------------
// Name:            AHRS_PING
// Description:     MIP command to ping the AHRS, prompting it to respond with a
//                  ping response message.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_PING();

//------------------------------------------------------------------------------
// Name:            AHRS_SET_IDLE
// Description:     MIP command to set the AHRS to idle mode.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_IDLE();

//------------------------------------------------------------------------------
// Name:            AHRS_RESET
// Description:     MIP command to reset the AHRS.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_RESET();

//------------------------------------------------------------------------------
// Name:            AHRS_ENABLE_MAG_AUTO_CALIBRATION
// Description:     MIP command to enable magneometer hard iron and soft iron
//                  auto calibration.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_ENABLE_MAG_AUTO_CALIBRATION();

//------------------------------------------------------------------------------
// Name:            AHRS_CAPTURE_MAG_AUTO_CALIBRATION
// Description:     MIP command to capture magneometer hard iron and soft iron
//                  auto calibration values.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_CAPTURE_MAG_AUTO_CALIBRATION();

//------------------------------------------------------------------------------
// Name:            AHRS_SET_HARD_IRON
// Description:     MIP command to set the magnetometer hard iron vector.
// Arguments:       - b: Hard iron calibration vector.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_HARD_IRON(Vector3f b);

//------------------------------------------------------------------------------
// Name:            AHRS_GET_HARD_IRON
// Description:     MIP command to get the current magnetometer hard iron
//                  vector.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_GET_HARD_IRON();

//------------------------------------------------------------------------------
// Name:            AHRS_SAVE_HARD_IRON
// Description:     MIP command to save the current magnetometer hard iron
//                  vector as the startup value.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SAVE_HARD_IRON();

//------------------------------------------------------------------------------
// Name:            AHRS_SET_SOFT_IRON
// Description:     MIP command to set the magnetometer soft iron matrix.
// Arguments:       - A: Soft iron calibration matrix.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_SOFT_IRON(Matrix3f A);

//------------------------------------------------------------------------------
// Name:            AHRS_GET_SOFT_IRON
// Description:     MIP command to get the current magnetometer soft iron
//                  matrix.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_GET_SOFT_IRON();

//------------------------------------------------------------------------------
// Name:            AHRS_SAVE_SOFT_IRON
// Description:     MIP command to save the current magnetometer soft iron
//                  matrix as the startup value.
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SAVE_SOFT_IRON();

//--------------------------------------------------------------------------
// Name:            AHRS_SET_IMU_MESSAGE_FORMAT
// Description:     Creates a vector of bytes representing the command to
//                  set the ahrs to return euler angles, angular velocities,
//                  accelerations, and magnetometer data all at 50 Hz
// Returns:         The bytes of the mip command that will configure the
//                  ahrs IMU
//--------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_IMU_MESSAGE_FORMAT();

//------------------------------------------------------------------------------
// Name:            AHRS_ENABLE_IMU_STREAM
// Description:     MIP command to enable continuous IMU data streaming
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_ENABLE_IMU_STREAM();

//------------------------------------------------------------------------------
// Name:            AHRS_SET_SENSOR_TO_VEHICLE_TRANSFORM
// Description:     MIP command to set the AHRS sensor to vehicle frame Euler
//                  angles.
// Arguments:       - euler_angles: sensor to vehicle frame Euler angles in
//                    degrees
// Returns:         Corresponding array of MIP command bytes.
//------------------------------------------------------------------------------
std::vector<uint8_t> AHRS_SET_SENSOR_TO_VEHICLE_TRANSFORM(
    std::vector<float> euler_angles);

#endif //MIP_H
