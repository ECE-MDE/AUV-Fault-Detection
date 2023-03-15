//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a DEVICE packet conforming to the AVL
//              binary packet protocol, providing functions to construct them
//              and get the information described by their fields.
//==============================================================================

#include <avl_core/protocol/device_packet.h>

// Packet base class
#include <avl_core/protocol/packet.h>

// included for std::nan
#include <cmath>

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//--------------------------------------------------------------------------
// Name:        DevicePacket constructor
// Description: Constructs a DEVICE packet with a single parameter.
// Arguments:   - device_name: Name of the device
//              - param: DeviceParameter struct to populate packet
//                fields with.
//--------------------------------------------------------------------------
DevicePacket::DevicePacket(PacketHeader header, std::string device_name,
    DeviceParameter param) : Packet(header, DEVICE_PACKET)
{
    // Add the device name
    std::vector<uint8_t> name(device_name.begin(), device_name.end());
    add_field(DEVICE_NAME, name);

    // Add the fields for the parameter
    std::vector<uint8_t> param_name(param.name.begin(), param.name.end());
    add_field(0x01, param_name);
    add_field(0x02, {param.type});
    add_field(0x03, param.data);
}

//--------------------------------------------------------------------------
// Name:        DevicePacket constructor
// Description: Constructs a DEVICE packet with several parameters.
// Arguments:   - device_name: Name of the device
//              - params: Vector of parameter struct to populate packet
//                fields with.
//--------------------------------------------------------------------------
DevicePacket::DevicePacket(PacketHeader header, std::string device_name,
    std::vector<DeviceParameter> params) : Packet(header, DEVICE_PACKET)
{
    // Add the device name
    std::vector<uint8_t> name(device_name.begin(), device_name.end());
    add_field(DEVICE_NAME, name);

    // Add the fields for the parameters
    for(uint8_t i = 0; i < params.size(); i++)
    {
        std::vector<uint8_t> param_name(params.at(i).name.begin(), params.at(i).name.end());
        add_field(3*i + 0x01, param_name);
        add_field(3*i + 0x02, {params.at(i).type});
        add_field(3*i + 0x03, params.at(i).data);
    }

}

//------------------------------------------------------------------------------
// Name:        DevicePacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: vector of packet bytes
//------------------------------------------------------------------------------
DevicePacket::DevicePacket(std::vector<uint8_t> bytes) :
    Packet(bytes)
{

    // Ensure that the packet descriptor is correct and that the packet has
    // all of the required fields

    if (descriptor != DEVICE_PACKET)
        throw std::runtime_error("incorrect packet descriptor for a "
            "DEVICE packet");

    if (get_num_fields() < 4)
        throw std::runtime_error("insufficient fields for DEVICE packet");

}

//------------------------------------------------------------------------------
// Name:        get_param
// Description: Gets the parameter information from the DEVICE
//              packet's fields.
// Returns:     DeviceParameter struct described by the packet contents.
//------------------------------------------------------------------------------
DeviceParameter DevicePacket::get_param()
{

    DeviceParameter param;

    std::vector<uint8_t> name_data = get_field(0x01).get_data();
    param.name = std::string(name_data.begin(), name_data.end());

    param.type = static_cast<DeviceParameterType>(
        get_field(0x02).get_data().at(0));

    param.data = get_field(0x03).get_data();

    return param;

}

//------------------------------------------------------------------------------
// Name:        get_params
// Description: Gets the parameters information from the DEVICE
//              packet's fields.
// Returns:     Vector of DeviceParameter structs described by the packet
//              contents.
//------------------------------------------------------------------------------
std::vector<DeviceParameter> DevicePacket::get_params()
{

    std::vector<DeviceParameter> params;

    for(uint8_t i = 0; i < (get_num_fields()-1)/3; i++)
    {
        DeviceParameter param;
        std::vector<uint8_t> name_data = get_field(3*i + 0x01).get_data();
        param.name = std::string(name_data.begin(), name_data.end());

        param.type = static_cast<DeviceParameterType>(
            get_field(3*i + 0x02).get_data().at(0));

        param.data = get_field(3*i + 0x03).get_data();

        params.push_back(param);
    }

    return params;

}

}
