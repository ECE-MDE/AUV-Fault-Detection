//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a DEVICE packet conforming to the AVL
//              binary packet protocol, providing functions to construct them
//              and get the information described by their fields.
//==============================================================================

#include "protocol/device_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        DevicePacket constructor
// Description: Constructs a DEVICE packet from a base packet class.
// Arguments:   - packet: Packet to construct the DEVICE packet from.
//------------------------------------------------------------------------------
DevicePacket::DevicePacket(Packet packet) : Packet(packet.get_bytes())
{
    if (packet.get_descriptor() != DEVICE_PACKET)
        throw std::runtime_error("packet is not a DEVICE packet");
}

//------------------------------------------------------------------------------
// Name:        DevicePacket constructor
// Description: Constructs a DEVICE packet from a device info struct.
// Arguments:   - device_info: Device info struct.
//------------------------------------------------------------------------------
DevicePacket::DevicePacket(PacketHeader header, DeviceInfo device_info) :
    Packet(header, DEVICE_PACKET)
{
    add_field(DEVICE_NAME, avl::to_bytes(device_info.name));
    add_fields(device_info.parameters.to_fields(DEVICE_PARAMS_START));
}

//------------------------------------------------------------------------------
// Name:        DevicePacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: Vector of packet bytes.
//------------------------------------------------------------------------------
DevicePacket::DevicePacket(std::vector<uint8_t> bytes) : Packet(bytes)
{

    if (!has_field(DEVICE_NAME))
        throw std::runtime_error("packet is missing required NAME field");

}

//------------------------------------------------------------------------------
// Name:        get_device_info
// Description: Gets the device information from the DEVICE packet's fields.
// Returns:     Device info struct described by the packet contents.
//------------------------------------------------------------------------------
DeviceInfo DevicePacket::get_device_info()
{

    DeviceInfo device_info;

    // Extract the command name string
    std::vector<uint8_t> name_data = get_field(DEVICE_NAME).get_data();
    device_info.name = std::string(name_data.begin(), name_data.end());

    // Create the prameter list from the remaining fields
    ParameterList param_list;
    param_list.from_fields(avl::subvector(payload,
        DEVICE_PARAMS_START, payload.size()-1));
    device_info.parameters = param_list;

    return device_info;

}

}
