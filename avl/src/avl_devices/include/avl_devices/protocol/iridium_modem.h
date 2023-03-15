//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides command strings to configure an Iridium SBD modem.
//==============================================================================

#ifndef IRIDIUM_MODEM_PROTOCOL_H
#define IRIDIUM_MODEM_PROTOCOL_H

// Utility functions
#include <avl_core/util/time.h>
#include <avl_core/util/string.h>

//==============================================================================
//                            SBD RESULT STRUCT
//==============================================================================

// Structure containing information about the results of an SBD session
typedef struct SbdResult
{

    int mo_status;
    int mo_msn;
    int mt_status;
    int mt_msn;
    int mt_length;
    int mt_queued;

} SbdResult;

//==============================================================================
//                          SBD STATUS MAPPINGS
//==============================================================================

// Mapping between the MO Status value returned by the iridium session,
// its corresponding string describing the result, and a boolean describing
// if the MO message transmission was a success or a failure
std::map<int, std::pair<std::string, bool>> mo_status_map =
{
    {0, std::make_pair("MO message, if any, transferred successfully.", true)},
    {1, std::make_pair("MO message, if any, transferred successfully, but the MT message in the queue was too big to be transferred.", true)},
    {2, std::make_pair("MO message, if any, transferred successfully, but the requested Location Update was not accepted.", true)},
    {10, std::make_pair("Gateway reported that the call did not complete in the allowed time.", false)},
    {11, std::make_pair("MO message queue at the Gateway is full.", false)},
    {12, std::make_pair("MO message has too many segments.", false)},
    {13, std::make_pair("Gateway reported that the session did not complete.", false)},
    {14, std::make_pair("Invalid segment size.", false)},
    {15, std::make_pair("Access is denied.", false)},
    {16, std::make_pair("9602 has been locked and may not make SBD calls.", false)},
    {17, std::make_pair("Gateway not responding (local session timeout).", false)},
    {18, std::make_pair("Connection lost (RF drop).", false)},
    {32, std::make_pair("No network service, unable to initiate call.", false)},
    {33, std::make_pair("Antenna fault, unable to initiate call.", false)},
    {34, std::make_pair("Radio is disabled, unable to initiate call.", false)},
    {35, std::make_pair("9602 is busy, unable to initiate call.", false)}
};

// Mapping between the MT Status value returned by the iridium session,
// its corresponding string describing the result, and a boolean describing
// if the MT message receive was a success or a failure
std::map<int, std::pair<std::string, bool>> mt_status_map =
{
    {0, std::make_pair("No MT SBD message to receive from the Gateway.", false)},
    {1, std::make_pair("MT SBD message successfully received from the Gateway.", true)},
    {2, std::make_pair("An error occurred while attempting to perform a mailbox check or receive a message from the Gateway.", false)},
};

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

inline SbdResult parse_sbd_result(std::string result_string)
{

    // Get the arguments to the status response and split it by the commas
    std::vector<std::string> split_result = avl::split(result_string, ",");

    SbdResult sbd_result;
    sbd_result.mo_status = std::stoi(avl::split(split_result.at(0), " ").at(1));
    sbd_result.mo_msn =    std::stoi(split_result[1]);
    sbd_result.mt_status = std::stoi(split_result[2]);
    sbd_result.mt_msn =    std::stoi(split_result[3]);
    sbd_result.mt_length = std::stoi(split_result[4]);
    sbd_result.mt_queued = std::stoi(split_result[5]);

    return sbd_result;

}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_ATTENTION
// Description: Attention code, to which the modem should respond OK\r\n.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_ATTENTION()
{
    return "AT\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_RESET_SETTINGS
// Description: Command to set all modem settings to their factory defaults.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_RESET_SETTINGS()
{
    return "AT&F0\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_SET_ECHO
// Description: Command to enable or disable command echo.
// Arguments:   - enable: true to enable, false to disable
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_SET_ECHO(bool enable)
{
    std::string command;
    if (enable)
        command = "ATE1\r\n";
    else
        command = "ATE0\r\n";
    return command;
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_SET_FLOW_CONTROL
// Description: Command to enable or disable RTS/CTS flow control.
// Arguments:   - enable: true to enable, false to disable
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_DISABLE_FLOW_CONTROL(bool enable)
{
    std::string command;
    if (enable)
        command = "AT&K1\r\n";
    else
        command = "AT&K0\r\n";
    return command;
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_SET_EVENT_REPORTING
// Description: Command to enable or disable indicator event reporting.
// Arguments:   - enable_signal_quality: true to enable signal quality
//                reporting, false to disable
//              - enable_service_availability: true to enable service
//                availability reporting, false to disable
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_SET_EVENT_REPORTING(bool enable_signal_quality,
                                               bool enable_service_availability)
{
    char command[128];
    sprintf(command, "AT+CIER=1,%d,%d,0\r\n",
        (int)enable_signal_quality, (int)enable_service_availability);
    return std::string(command);
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_SET_RING_NOTIFICATIONS
// Description: Command to enable or disable SBD ring notifications.
// Arguments:   - enable: true to enable, false to disable
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_SET_RING_NOTIFICATIONS(bool enable)
{
    std::string command;
    if (enable)
        command = "AT+SBDMTA=1\r\n";
    else
        command = "AT+SBDMTA=0\r\n";
    return command;
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_ENABLE_RADIO
// Description: Command to enable the Iridium radio.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_ENABLE_RADIO()
{
    return "AT*R1\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_SHUTDOWN
// Description: Command to flush all pending writes to EEPROM, shut down the
//              radio, and prepare the modem to be powered down.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_SHUTDOWN()
{
    return "AT*F\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_CLEAR_WRITE_BUFFER
// Description: Command to clear the mobile originated (write) buffer.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_CLEAR_WRITE_BUFFER()
{
    return "AT+SBDD0\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_CLEAR_READ_BUFFER
// Description: Command to clear the mobile terminated (read) buffer.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_CLEAR_READ_BUFFER()
{
    return "AT+SBDD1\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_GET_SIGNAL_QUALITY
// Description: Command to get the Iridium signal quality.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_GET_SIGNAL_QUALITY()
{
    return "AT+CSQ\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_WRITE_BYTES_TO_BUFFER
// Description: Command to write bytes to the mobile originated (write) buffer
//              on the modem, which can then be sent by initiating an SBD
//              session. The modem will respond with READY\r\n when it is ready
//              to receive the given number of bytes. Once the ready message is
//              received, receives the bytes should be sent to the modem
//              formatted as follows: <BYTES><2 BYTES CHECKSUM>\r\n. The
//              checksum is the least significant two bytes of the summation of
//              <BYTES>.
// Arguments:   - num_bytes: number of bytes that will be written to the buffer
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_WRITE_BYTES_TO_BUFFER(size_t num_bytes)
{
    if (num_bytes < 1 || num_bytes > 340)
        throw std::runtime_error("IRIDIUM_WRITE_BYTES_TO_BUFFER: number of"
                                 "bytes must be between 1 and 340");

    return "AT+SBDWB=" + std::to_string(num_bytes) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_BINARY_TRANSFER
// Description: Command to transfer bytes to the mobile originated (write) buffer
//              on the modem. Adds the two byte checksum to the bytes. Result is
//              formatted as follows: <BYTES><2 BYTES CHECKSUM>\r\n. The
//              checksum is the least significant two bytes of the summation of
//              <BYTES>.
// Arguments:   - bytes: bytes to be written to the buffer
// Returns:     Bytes to be transferred with checksum appended.
//------------------------------------------------------------------------------
inline std::vector<uint8_t> IRIDIUM_BINARY_TRANSFER(std::vector<uint8_t> bytes)
{

    if (bytes.size() < 1 || bytes.size() > 340)
        throw std::runtime_error("IRIDIUM_BINARY_TRANSFER: number of"
                                 "bytes must be between 1 and 340");

    uint32_t sum = 0x00;
    for (uint8_t byte: bytes)
        sum += byte;
    uint8_t sum_lsb_high = (sum >> 8) & 0xFF;
    uint8_t sum_lsb_low = sum & 0xFF;

    bytes.push_back(sum_lsb_high);
    bytes.push_back(sum_lsb_low);

    return bytes;

}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_READ_BYTES_FROM_BUFFER
// Description: Command to read bytes from the mobile terminated (read) buffer
//              on the modem, which were received by the modem during an SBD
//              session. The modem will then send the bytes in the format
//              <2 BYTES LENGTH><BYTES><2 BYTES CHECKSUM>\r\n. The number
//              <2 BYTES LENGTH> includes  only the length of <BYTES>. The
//              checksum is the least significant two bytes of the summation of
//              <BYTES>.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_READ_BYTES_FROM_BUFFER()
{
    return "AT+SBDRB\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_WRITE_STRING_TO_BUFFER
// Description: Command to write a text string to the mobile originated (write)
//              buffer on the modem, which can then be sent by initiating an SBD
//              session.
// Arguments:   - data: string to be written to buffer
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_WRITE_STRING_TO_BUFFER(std::string data)
{
    return "AT+SBDWT=" + data + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_READ_STRING_FROM_BUFFER
// Description: Command to read a text string from the mobile terminated (read)
//              buffer on the modem, which were received by the modem during an
//              SBD session. The modem response is:
//                  +SBDRT:\r <STRING>\r\n
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_READ_STRING_FROM_BUFFER()
{
    return "AT+SBDRT\r\n";
}

//------------------------------------------------------------------------------
// Name:        IRIDIUM_START_SBD_SESSION
// Description: Command to start an SBD session to transfer messages between the
//              modem and the Iridium network.
// Arguments:   - ring_alert_response: flag indicating whether the session
//                initiation is in response to a ring alert
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string IRIDIUM_START_SBD_SESSION(bool ring_alert_response)
{
    std::string command;
    if (ring_alert_response)
        command = "AT+SBDIXA\r\n";
    else
        command = "AT+SBDIX\r\n";
    return command;
}

#endif // IRIDIUM_MODEM_PROTOCOL_H
