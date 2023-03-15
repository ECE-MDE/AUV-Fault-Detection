//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides command strings to configure and control a WHOI
//              micromodem
//==============================================================================

#ifndef WHOI_MICROMODEM_H
#define WHOI_MICROMODEM_H

// Utility functions
#include <avl_core/util/misc.h>

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        WHOI_NMEA_PASSTHROUGH
// Description: Passes a given NMEA message to another COM port on the WHOI
//              micromodem. The NMEA message given to this function should ONLY
//              include the message string and should NOT include the $
//              character, the checksum, or the CRLF at the end. These
//              characters will be appended automatically by the micromodem.
// Arguments:   - message: NMEA message to be passed through
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string WHOI_NMEA_PASSTHROUGH(std::string message,
    bool port1, bool port2, bool port3, bool port4)
{

    // Command configuration parameters. They are set to strip the CCPST message
    // and append the $ and checksum
    int target_ports = 0b0000;
    int strip_msg = 1;
    int prepend_dollar = 1;
    int append_cksum = 1;
    int append_CRLF = 0;

    // Set up the bit mask based on the arguments
    if (port1)
        target_ports |= 0b0001;
    if (port2)
        target_ports |= 0b0010;
    if (port3)
        target_ports |= 0b0100;
    if (port4)
        target_ports |= 0b1000;

    char command[1024];
    sprintf(command, "$CCPST,%d,%d,%d,%d,%d,,%s",
        target_ports, strip_msg, prepend_dollar, append_cksum, append_CRLF,
        message.c_str());
    return command + avl::get_nmea_checksum(command) + "\r\n";

}

//------------------------------------------------------------------------------
// Name:        WHOI_SET_PARAMETER
// Description: Sets a setting value on the micromodem. Each setting has a
//              three letter setting name and an integer value. See the WHOI
//              micromodem Software Interface Guide for available settings.
// Arguments:   - name: three letter setting name
//              - value: integer setting value
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string WHOI_SET_PARAMETER(std::string name, int value)
{
    char command[128];
    sprintf(command, "$CCCFG,%s,%d", name.c_str(), value);
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        WHOI_GET_ALL_PARAMETERS
// Description: Command to print all micromodem setting values.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string WHOI_GET_ALL_PARAMETERS()
{
    std::string command = "$CCCFQ,ALL";
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        WHOI_GET_DEVICE_INFO
// Description: Command to print all micromodem device information.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string WHOI_GET_DEVICE_INFO()
{
    std::string command = "$CCCFQ,info";
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        WHOI_PING_DIGITAL_TRANSPONDERS
// Description: Command to ping REMUS digital transponders. Causes transponders
//              to return a response ping. The micromodem will wait for the
//              specified duration for responses and then print a message with
//              transponder response information.
// Arguments:   - channel: interrogation channel (1-4)
//              - response_duration: time to listen for transponder responses in
//                milliseconds
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
std::string WHOI_PING_DIGITAL_TRANSPONDERS(int channel, int response_duration)
{
    char command[128];
    sprintf(command, "$CCPDT,1,%d,0,0,%d,1,1,1,1", channel, response_duration);
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        WHOI_INITIATE_CYCLE
// Description: Command to initiate an acoustic cycle, either downlink or uplink
//              depending on the source and destination device ID's given.
// Arguments:   - source_id: source micromodem ID
//              - dest_id: destination micromodem ID
//              - packet_type: acoustic packet encoding type (0-6). See the
//                micromodem software interface guide for packet types.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
std::string WHOI_INITIATE_CYCLE(int source_id, int dest_id, int packet_type)
{
    char command[128];
    sprintf(command, "$CCCYC,1,%d,%d,%d,0,1", source_id, dest_id, packet_type);
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        WHOI_TRANSMIT_BINARY_DATA
// Description: Command to transmit binary data in response to a data request
//              command. Source and destination ID should match the data request
//              message's source and destination ID.
// Arguments:   - source_id: source micromodem ID
//              - dest_id: destination micromodem ID
//              - data: vector of bytes to be sent
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
std::string WHOI_TRANSMIT_BINARY_DATA(int source_id, int dest_id,
    std::vector<uint8_t> data)
{

    // Format the vector of bytes as a hex-encoded string
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::uppercase;
    std::vector<uint8_t>::const_iterator it;

    for (it = data.begin(); it != data.end(); it++)
        ss << std::setw(2) << static_cast<unsigned>(*it);

    std::string data_string = ss.str();

    // Format the command
    char command[2048];
    sprintf(command, "$CCTXD,%d,%d,0,%s",
        source_id, dest_id, data_string.c_str());
    return command + avl::get_nmea_checksum(command) + "\r\n";

}

//------------------------------------------------------------------------------
// Name:        WHOI_REMOTE_START_USBL_PINGS
// Description: Command to tell a remote modem to start USBL pings.
// Arguments:   - dest_id: destination micromodem ID
//              - rate: The ping repetition rate in tens of Hz. Allowable values
//                are 1, 2, 5, 10,20,40,50,100,200, corresponding to rates of
//                0.1, 0.2, 0.5, 1, 2, 4, 5, 10 and 20 Hz.
//              - duration: The number of seconds to transmit, range is
//                0 to 3600.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
std::string WHOI_REMOTE_START_USBL_PINGS(int dest_id, int rate, int duration)
{

    // $CCACM,dest,rate,ack,cmd_string*CS
    // $CCXSB,enable_flag,signal_type,reprate_hzx10,nsecs,timing_mode*CS
    // $CCACM,6,3,1,CCXSB,1,1,10,5,0

    // Format the command
    char command[2048];
    sprintf(command, "$CCACM,%d,3,1,CCXSB,1,1,%d,%d,0",
        dest_id, rate, duration);
    return command + avl::get_nmea_checksum(command) + "\r\n";

}

//------------------------------------------------------------------------------
// Name:        WHOI_START_USBL_PINGS
// Description: Command to tell the modem to start USBL pings.
// Arguments:   - signal_type: 0 for FSK, 1 for PSK
//              - reprate_hzx10: The ping repetition rate in tens of Hz.
//                Allowable values are 1, 2, 5, 10,20,40,50,100,200,
//                corresponding to rates of 0.1, 0.2, 0.5, 1, 2, 4, 5, 10 and
//                20 Hz.
//              - nsecs: The number of seconds to transmit, range is 0 to 3600.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
std::string WHOI_START_USBL_PINGS(int signal_type, int reprate_hzx10, int nsecs)
{


    // $CAUSB, YYYY-MM-ddTHH:mm:ss.ssssZ,azimuth_deg,el_deg,owtt,timing_mode*CS

    // $CCXSB,enable_flag,signal_type,reprate_hzx10,nsecs,timing_mode*CS
    // Format the command
    char command[2048];
    sprintf(command, "$CCXSB,1,%d,%d,%d,0", signal_type, reprate_hzx10, nsecs);
    return command + avl::get_nmea_checksum(command) + "\r\n";

}

//------------------------------------------------------------------------------
// Name:        WHOI_STOP_USBL_PINGS
// Description: Command to tell the modem to stop USBL pings.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
std::string WHOI_STOP_USBL_PINGS()
{

    // $CCXSB,enable_flag,signal_type,reprate_hzx10,nsecs,timing_mode*CS
    // Format the command
    char command[2048];
    sprintf(command, "$CCXSB,0,0,1,0,0");
    return command + avl::get_nmea_checksum(command) + "\r\n";

}

//------------------------------------------------------------------------------
// Name:        WHOI_START_LISTEN_USBL_PINGS
// Description: Command to tell the modem to start listening for USBL pings.
// Arguments:   - signal_type: 0 for FSK, 1 for PSK
//              - reprate_hzx10: The ping repetition rate in tens of Hz.
//                Allowable values are 1, 2, 5, 10,20,40,50,100,200,
//                corresponding to rates of 0.1, 0.2, 0.5, 1, 2, 4, 5, 10 and
//                20 Hz.
//              - nsecs: The number of seconds to transmit, range is 0 to 3600.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
std::string WHOI_START_LISTEN_USBL_PINGS(int signal_type, int reprate_hzx10,
    int nsecs)
{

    // $CCRSB,enable_flag,signal_type,reprate_hzx10,nsecs,timing_mode*CS
    // Format the command
    char command[2048];
    sprintf(command, "$CCRSB,1,%d,%d,%d,0", signal_type, reprate_hzx10, nsecs);
    return command + avl::get_nmea_checksum(command) + "\r\n";

}

//------------------------------------------------------------------------------
// Name:        WHOI_STOP_LISTEN_USBL_PINGS
// Description: Command to tell the modem to stop listening for USBL pings.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
std::string WHOI_STOP_LISTEN_USBL_PINGS()
{

    // $CCRSB,enable_flag,signal_type,reprate_hzx10,nsecs,timing_mode*CS
    // Format the command
    char command[2048];
    sprintf(command, "$CCRSB,0,0,1,0,0");
    return command + avl::get_nmea_checksum(command) + "\r\n";

}

#endif // WHOI_MICROMODEM_H
