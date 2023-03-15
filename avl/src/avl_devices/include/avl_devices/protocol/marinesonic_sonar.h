//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides command strings to configure a Marine Sonic Sea Scan
//              sonar.
//==============================================================================

#ifndef MARINESONIC_SONAR_H
#define MARINESONIC_SONAR_H

// Utility functions
#include <avl_core/util/misc.h>

//==============================================================================
//                            SONAR CONFIGURATION
//==============================================================================

typedef struct SonarConfig
{

    // Sonar ping mode (OFF, AUTO, MANUAL, MAX)
    std::string ping_mode;

    // Sonar ping rate in Hz ( to )
    float ping_rate;

    // Maximum sonar ping range in meters (3.0 to 500.0)
    float max_range;

    // Minimum sonar range in meters (0.0 to max_range)
    float min_range;

    // Number of samples to acquire with each ping (256, 512, 1024)
    int num_samples;

    // Number of bits per sample (8, 12)
    int sample_size;

    // Index of tansducer on left and right transducer bus channels to use for
    // data collection (0 to 6)
    int left_transducer_index;
    int right_transducer_index;

} SonarConfig;

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        MS_CONFIGURE_SONAR
// Description: Command to configure the Marinesonic sonar.
// Arguments:   - config: sonar configuratin structure
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string MS_CONFIGURE_SONAR(SonarConfig config)
{
    char buf[1024];
    sprintf(buf, "$PSST,SONAR,SET,%s,ON,%.2f,%.2f,%.2f,,%d,%d,%d,%d",
        config.ping_mode.c_str(),
        config.ping_rate,
        config.max_range,
        config.min_range,
        config.num_samples,
        config.sample_size,
        config.left_transducer_index,
        config.right_transducer_index);
    std::string command = std::string(buf);
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        MS_START_SDS_STREAM
// Description: Command to start the sonar's SDS data stream to a given IP
//              address over UDP.
// Arguments:   - destination_address: destination IP address that the sonar
//                will stream SDS data to
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string MS_START_SDS_STREAM(std::string destination_address)
{
    std::string command = "$PSST,DATA,NETWORK,START," + destination_address;
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        MS_STOP_UDP_STREAM
// Description: Command to stop the sonar's UDP SDS data stream.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string MS_STOP_UDP_STREAM()
{
    std::string command = "$PSST,DATA,NETWORK,STOP";
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        MS_START_DISK_STREAM
// Description: Command to start writing the SDS data stream to the sonar
//              computer's disk.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string MS_START_DISK_STREAM()
{
    std::string command = "$PSST,DATA,DISK,START,AUTO,,65535,";
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        MS_STOP_DISK_STREAM
// Description: Command to stop writing the SDS data stream to the sonar
//              computer's disk.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string MS_STOP_DISK_STREAM()
{
    std::string command = "$PSST,DATA,DISK,STOP";
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        MS_SHUTDOWN
// Description: Command to shut down the sonar system.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string MS_SHUTDOWN()
{
    std::string command = "$PSST,GENERAL,SHUTDOWN";
    return command + avl::get_nmea_checksum(command) + "\r\n";
}

#endif // MARINESONIC_SONAR_H
