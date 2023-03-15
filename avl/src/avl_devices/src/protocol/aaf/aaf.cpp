//==============================================================================
// Autonomous Vehicle Library
//
// Description: Defines a mapping of packet descriptors and field descriptors
//              which implement the AVL binary communication protocol. Also
//              provides helper functions for creating packets.
//==============================================================================

#include <avl_devices/protocol/aaf/aaf.h>

// AAF packet class
#include <avl_devices/protocol/aaf/aaf_packet.h>

// Util functions
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>

//------------------------------------------------------------------------------
// Name:        SENSOR_DATA_PACKET
// Description: Creates an empty SENSOR_DATA packet.
// Returns:     SENSOR_DATA packet.
//------------------------------------------------------------------------------
AafPacket SENSOR_DATA_PACKET()
{
    AafPacket packet;
    packet.set_descriptor(SENSOR_DATA_DESC);
    return packet;
}

//------------------------------------------------------------------------------
// Name:        SYSTEM_DATA_PACKET
// Description: Creates an empty SYSTEM_DATA packet.
// Returns:     SYSTEM_DATA packet.
//------------------------------------------------------------------------------
AafPacket SYSTEM_DATA_PACKET()
{
    AafPacket packet;
    packet.set_descriptor(SYSTEM_DATA_DESC);
    return packet;
}

//------------------------------------------------------------------------------
// Name:        AUV_DATA_PACKET
// Description: Creates an empty AUV_DATA packet.
// Returns:     AUV_DATA packet.
//------------------------------------------------------------------------------
AafPacket AUV_DATA_PACKET()
{
    AafPacket packet;
    packet.set_descriptor(AUV_DATA_DESC);
    return packet;
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_PACKET
// Description: Creates an empty AUV_COMMAND packet.
// Returns:     AUV_COMMAND packet.
//------------------------------------------------------------------------------
AafPacket AUV_COMMAND_PACKET()
{
    AafPacket packet;
    packet.set_descriptor(AUV_COMMAND_DESC);
    return packet;
}

//------------------------------------------------------------------------------
// Name:        AUV_DATA_FIN_ANGLE_MEASUREMENT
// Description: Creates an AUV_DATA packet FIN_ANGLE_MEASUREMENT field.
// Arguments:   - angle: measured fin angle in degrees
// Returns:     AUV_DATA packet FIN_ANGLE_MEASUREMENT field.
//------------------------------------------------------------------------------
AafField AUV_DATA_FIN_ANGLE_MEASUREMENT(float angle)
{
    return AafField(AUV_DATA_FIN_ANGLE_MEASUREMENT_DESC, avl::to_bytes(angle, true));
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_FIN_POSITIONS
// Description: Creates an AUV_COMMAND packet FIN_POSITIONS field.
// Arguments:   - top: top fin angle in degrees
//              - starboard: starboard fin angle in degrees
//              - bottom: bottom fin angle in degrees
//              - port: port fin angle in degrees
// Returns:     AUV_COMMAND packet FIN_POSITIONS field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_FIN_POSITIONS(float top, float starboard, float bottom, float port)
{
    std::vector<uint8_t> payload = avl::to_bytes(top, true);
    avl::append(payload, avl::to_bytes(starboard, true));
    avl::append(payload, avl::to_bytes(bottom, true));
    avl::append(payload, avl::to_bytes(port, true));
    return AafField(AUV_COMMAND_FIN_POSITIONS_DESC, payload);
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_MOTOR
// Description: Creates an AUV_COMMAND packet MOTOR field.
// Arguments:   - speed: motor speed from -1.0 to 1.0
// Returns:     AUV_COMMAND packet MOTOR field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_MOTOR(float speed)
{
    return AafField(AUV_COMMAND_MOTOR_DESC, avl::to_bytes(speed, true));
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_MAST_STATUS
// Description: Creates an AUV_COMMAND packet MAST_STATUS field.
// Arguments:   - mode: mast LED mode (0 = initialize, 1 = idle, 2 = run)
// Returns:     AUV_COMMAND packet MAST_STATUS field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_MAST_STATUS(uint16_t mode)
{
    return AafField(AUV_COMMAND_MAST_STATUS_DESC, avl::to_bytes(mode, true));
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_STROBE
// Description: Creates an AUV_COMMAND packet STROBE field.
// Arguments:   - mode: mast strobe mode (0 = off, 1 = on)
// Returns:     AUV_COMMAND packet STROBE field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_STROBE(uint16_t mode)
{
    return AafField(AUV_COMMAND_STROBE_DESC, avl::to_bytes(mode, true));
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_FINS_HOME
// Description: Creates an AUV_COMMAND packet FINS_HOME field.
// Returns:     AUV_COMMAND packet FINS_HOME field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_FINS_HOME()
{
    return AafField(AUV_COMMAND_FINS_HOME_DESC);
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_TARE_PRESSURE
// Description: Creates an AUV_COMMAND packet TARE_PRESSURE field.
// Returns:     AUV_COMMAND packet TARE_PRESSURE field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_TARE_PRESSURE()
{
    return AafField(AUV_COMMAND_TARE_PRESSURE_DESC);
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_FIN_SET_POSITION
// Description: Creates an AUV_COMMAND packet FIN_SET_POSITION field.
// Arguments:   - id: fin id (0 = top, 1 = starboard, 2 = bottom, 3 = port)
//              - position: fin angle in degrees
// Returns:     AUV_COMMAND packet FIN_SET_POSITION field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_FIN_SET_POSITION(uint16_t id, float position)
{
    std::vector<uint8_t> payload = avl::to_bytes(id, true);
    avl::append(payload, avl::to_bytes(position, true));
    return AafField(AUV_COMMAND_FIN_SET_POSITION_DESC, payload);
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_FIN_START_CALIBRATION
// Description: Creates an AUV_COMMAND packet FIN_START_CALIBRATION field.
// Arguments:   - id: fin id (0 = top, 1 = starboard, 2 = bottom, 3 = port)
//              - min: fin min angle in degrees
//              - min: fin max angle in degrees
//              - min: fin home angle in degrees
// Returns:     AUV_COMMAND packet FIN_START_CALIBRATION field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_FIN_START_CALIBRATION(uint16_t id, float min, float max, float home)
{
    std::vector<uint8_t> payload = avl::to_bytes(id, true);
    avl::append(payload, avl::to_bytes(min, true));
    avl::append(payload, avl::to_bytes(max, true));
    avl::append(payload, avl::to_bytes(home, true));
    return AafField(AUV_COMMAND_FIN_START_CALIBRATION_DESC, payload);
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_RESET
// Description: Creates an AUV_COMMAND packet RESET field.
// Returns:     AUV_COMMAND packet RESET field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_RESET()
{
    return AafField(AUV_COMMAND_RESET_DESC);
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_FIN_REQUEST_MEASUREMENT
// Description: Creates an AUV_COMMAND packet FIN_REQUEST_MEASUREMENT field.
// Returns:     AUV_COMMAND packet FIN_REQUEST_MEASUREMENT field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_FIN_REQUEST_MEASUREMENT()
{
    return AafField(AUV_COMMAND_FIN_REQUEST_MEASUREMENT_DESC);
}

//------------------------------------------------------------------------------
// Name:        AUV_COMMAND_MOTOR_SET_DIRECTION
// Description: Creates an AUV_COMMAND packet MOTOR_SET_DIRECTION field.
// Arguments:   - action: motor direction action (0 = reset, 1 = toggle)
// Returns:     AUV_COMMAND packet MOTOR_SET_DIRECTION field.
//------------------------------------------------------------------------------
AafField AUV_COMMAND_MOTOR_SET_DIRECTION(uint16_t action)
{
    return AafField(AUV_COMMAND_MOTOR_SET_DIRECTION_DESC, avl::to_bytes(action, true));
}
