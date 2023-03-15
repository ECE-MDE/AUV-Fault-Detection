//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides functions to configure and run a Valeport miniCT
//              Probe conductivity and temperature sensor.
//==============================================================================

#ifndef VALEPORT_H
#define VALEPORT_H

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        VALEPORT_SET_FORMAT
// Description: Command to set the miniCT data output format.
// Arguments:   - format: Output format string. See valeport manual.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string VALEPORT_SET_FORMAT(std::string format)
{
    return std::string("#082;") + format;
}

//------------------------------------------------------------------------------
// Name:        VALEPORT_SET_OUTPUT_RATE
// Description: Command to set the miniCT data output rate in Hz.
// Arguments:   - rate: Data output rate in Hz.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string VALEPORT_SET_OUTPUT_RATE(int output_rate)
{
    char command[128];
    sprintf(command, "M%d\r\n", output_rate);
    return std::string(command);
}

#endif // VALEPORT_H
