//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a DEVICE packet conforming to the AVL
//              binary packet protocol, providing functions to construct them
//              and get the information described by their fields.
//==============================================================================

#ifndef DEVICE_PACKET_H
#define DEVICE_PACKET_H

// Packet base class
#include "packet.h"

// ParameterList class
#include "parameter_list.h"

namespace avl
{

//==============================================================================
//                            PACKET FIELD MAPPING
//==============================================================================

// Enum listing a DEVICE packet's fields in order starting at 0x00
enum DeviceField
{
    DEVICE_NAME,
    DEVICE_PARAMS_START
};

//==============================================================================
//                             STRUCT DEFINITION
//==============================================================================

// Command struct
struct DeviceInfo
{
    std::string name;
    ParameterList parameters;
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class DevicePacket : public Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        DevicePacket constructor
    // Description: Constructs a DEVICE packet from a base packet class.
    // Arguments:   - packet: Packet to construct the DEVICE packet from.
    //--------------------------------------------------------------------------
    DevicePacket(Packet packet);

    //--------------------------------------------------------------------------
    // Name:        DevicePacket constructor
    // Description: Constructs a DEVICE packet from a device info struct.
    // Arguments:   - device_info: Device info struct.
    //--------------------------------------------------------------------------
    DevicePacket(PacketHeader header, DeviceInfo device_info);

    //--------------------------------------------------------------------------
    // Name:        DevicePacket constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: Vector of packet bytes.
    //--------------------------------------------------------------------------
    DevicePacket(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        get_device_info
    // Description: Gets the device information from the DEVICE packet's fields.
    // Returns:     Device info struct described by the packet contents.
    //--------------------------------------------------------------------------
    DeviceInfo get_device_info();

};

}

#endif // DEVICE_PACKET_H
