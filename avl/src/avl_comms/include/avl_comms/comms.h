//==============================================================================
// Autonomous Vehicle Library
//
// Description: Contains enums defined for the communication architecture.
//==============================================================================

#ifndef COMMS_H
#define COMMS_H

namespace avl
{

//==============================================================================
//                             ENUM DEFINITIONS
//==============================================================================

// Enum listing communication channels
enum CommsChannel
{
    CHANNEL_ETHERNET,
    CHANNEL_ACOUSTIC,
    CHANNEL_IRIDIUM,
    CHANNEL_MISSION
};

// Enum listing communication interfaces
enum CommsInterface
{
    INTERFACE_FSD,
    INTERFACE_BSD
};

//==============================================================================
//                             FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        channel_to_string
// Description: Converts a communication channel to a string representation of
//              the channel name.
// Arguments:   - channel: Channel to convert to string.
// Returns:     String representation of the channel name.
//------------------------------------------------------------------------------
inline std::string channel_to_string(CommsChannel channel)
{
    switch (channel)
    {
        case CHANNEL_ETHERNET:
            return "ETHERNET";
        case CHANNEL_ACOUSTIC:
            return "ACOUSTIC";
        case CHANNEL_IRIDIUM:
            return "IRIDIUM";
        case CHANNEL_MISSION:
            return "MISSION";
    }
    throw std::runtime_error("channel_to_string: invalid channel");
}

//------------------------------------------------------------------------------
// Name:        interface_to_string
// Description: Converts a communication interface to a string representation of
//              the interface name.
// Arguments:   - interface: Interface to convert to string.
// Returns:     String representation of the interface name.
//------------------------------------------------------------------------------
inline std::string interface_to_string(CommsInterface interface)
{
    switch (interface)
    {
        case INTERFACE_FSD:
            return "FSD";
        case INTERFACE_BSD:
            return "BSD";
    }
    throw std::runtime_error("interface_to_string: invalid interface");
}

} // namespace avl

#endif // COMMS_H
