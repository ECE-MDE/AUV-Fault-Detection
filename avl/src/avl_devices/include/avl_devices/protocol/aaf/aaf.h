//==============================================================================
// Autonomous Vehicle Library
//
// Description: Defines a mapping of packet descriptors and field descriptors
//              which implement the AAF binary communication protocol from NAVO.
//              Also provides helper functions for creating packets.
//==============================================================================

#ifndef AAF_H
#define AAF_H

// AAF packet class
#include <avl_devices/protocol/aaf/aaf_packet.h>

//==============================================================================
//                            AAF COMMAND MAPPING
//==============================================================================

// Packet types
const uint8_t SENSOR_DATA_DESC = 0x01;
const uint8_t SYSTEM_DATA_DESC = 0x02;
const uint8_t AUV_DATA_DESC =    0x03;
const uint8_t AUV_COMMAND_DESC = 0x04;

// SENSOR_DATA packet field descriptors
const uint8_t SENSOR_DATA_PRESSURE_DESC    = 0x00;
const uint8_t SENSOR_DATA_TEMPERATURE_DESC = 0x01;
const uint8_t SENSOR_DATA_BATTERY_DESC     = 0x02;
const uint8_t SENSOR_DATA_ADC_DESC         = 0x03;
const uint8_t SENSOR_DATA_COUNTER_DESC     = 0x04;
const uint8_t SENSOR_DATA_SWITCH_DESC      = 0x05;
const uint8_t SENSOR_DATA_MOTOR_SPEED_DESC = 0x06;

// SYSTEM_DATA packet field descriptors
const uint8_t SYSTEM_DATA_HEARTBEAT_DESC = 0x00;

// AUV_DATA packet field descriptors
const uint8_t AUV_DATA_TAIL_POWER_STATUS_DESC      = 0x00;
const uint8_t AUV_DATA_NOSE_POWER_STATUS_DESC      = 0x01;
const uint8_t AUV_DATA_FIN_ANGLE_MEASUREMENT_DESC  = 0x03;
const uint8_t AUV_DATA_FIN_CALIBRATION_STATUS_DESC = 0x04;

// AUV_COMMAND packet field descriptors
const uint8_t AUV_COMMAND_FIN_POSITIONS_DESC           = 0x00;
const uint8_t AUV_COMMAND_MOTOR_DESC                   = 0x01;
const uint8_t AUV_COMMAND_MAST_STATUS_DESC             = 0x02;
const uint8_t AUV_COMMAND_STROBE_DESC                  = 0x03;
const uint8_t AUV_COMMAND_FINS_HOME_DESC               = 0x04;
const uint8_t AUV_COMMAND_TARE_PRESSURE_DESC           = 0x05;
const uint8_t AUV_COMMAND_FIN_SET_POSITION_DESC        = 0x06;
const uint8_t AUV_COMMAND_FIN_START_CALIBRATION_DESC   = 0x07;
const uint8_t AUV_COMMAND_RESET_DESC                   = 0x08;
const uint8_t AUV_COMMAND_FIN_REQUEST_MEASUREMENT_DESC = 0x09;
const uint8_t AUV_COMMAND_MOTOR_SET_DIRECTION_DESC     = 0x0A;

//==============================================================================
//                            FUNCTION DECLARATIONS
//==============================================================================

// Packet creation helper functions
AafPacket SENSOR_DATA_PACKET();
AafPacket SYSTEM_DATA_PACKET();
AafPacket AUV_DATA_PACKET();
AafPacket AUV_COMMAND_PACKET();

// AUV_DATA packet field creation helper functions
AafField AUV_DATA_FIN_ANGLE_MEASUREMENT(float angle);

// AUV_COMMAND packet field creation helper functions
AafField AUV_COMMAND_FIN_POSITIONS(float top, float starboard, float bottom,
    float port);
AafField AUV_COMMAND_MOTOR(float speed);
AafField AUV_COMMAND_MAST_STATUS(uint16_t mode);
AafField AUV_COMMAND_STROBE(uint16_t mode);
AafField AUV_COMMAND_FINS_HOME();
AafField AUV_COMMAND_TARE_PRESSURE();
AafField AUV_COMMAND_FIN_SET_POSITION(uint16_t id, float position);
AafField AUV_COMMAND_FIN_START_CALIBRATION(uint16_t id, float min, float max,
    float home);
AafField AUV_COMMAND_RESET();
AafField AUV_COMMAND_FIN_REQUEST_MEASUREMENT();
AafField AUV_COMMAND_MOTOR_SET_DIRECTION(uint16_t action);

#endif // AAF_H
