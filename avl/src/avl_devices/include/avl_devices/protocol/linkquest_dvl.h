//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides command strings to get and set LinkQuest DVL
//              configuration parameters over RS232.
//==============================================================================

#ifndef LINKQUEST_DVL_H
#define LINKQUEST_DVL_H

//==============================================================================
//                              DVL DATA STRUCT
//==============================================================================

// NavQuest 600 Micro DVL data output structure for the NQ1 output format
typedef struct DvlNQ1Data
{

    std::string error_code;

    // Good (1) or bad (0) beam result for each of the 4 beams
    uint8_t good_or_bad[4];

    // Instrument altitude above the bottom in meters for each beam
    // Negative value means the information is not valid.
    float v_altitude[4];

    // Radial velocity in mm/sec of bottom for each beam.
    float velo_rad[4];

    // Radial velocity in mm/sec of water for each beam.
    float wvelo_rad[4];

    // Water velocity credit, a non-negative number from 0 to 50.
    // Higher values indicate better calculation
    uint8_t wvelo_credit[4];

    // Instrument velocity in mm/sec in instrument coordinates
    float velo_instrument[3];

    // Instrument velocity flag
    // flag = 2 water velocity (no valid bottom velocity)
    // flag = 1 bottom velocity
    // flag = 0 not valid
    uint8_t velo_instrument_flag;

    // Instrument velocity in mm/sec in earth coordinates
    float velo_earth[3];

    // Earth velocity flag
    // flag = 2 water velocity (no valid bottom velocity)
    // flag = 1 bottom velocity
    // flag = 0 not valid
    uint8_t velo_earth_flag;

    // Current velocity in mm/sec relative to the DVL system in instrument
    // coordinates
    float water_velo_instrument[3];

    // Water velocity instrument flag
    // flag = 2 valid
    // flag = 0 not valid
    uint8_t water_velo_instrument_flag;

    // Current velocity in mm/sec relative to the DVL system in earth
    // coordinates
    float water_velo_earth[3];

    // Water velocity earth flag
    // flag = 2 valid
    // flag = 0 not valid
    uint8_t water_velo_earth_flag;

    // Roll, pitch, and yaw in degrees
    float rpy[3];

    // Altitude estimate in meters from the bottom
    float altitude_estimate;

    // DVL temperature in Celsius
    float dvl_temperature;

    // DVL pressure: invalid for Micro DVL
    float dvl_pressure;

    // Salinity in parts per thousand
    float salinity;

    // Speed of sound used in velocity calculation m/sec
    uint16_t dvl_speed_of_sound;

} DvlNQ1Data;

//==============================================================================
//                          LINKQUEST COMMAND MAPPING
//==============================================================================

// LinkQuest DVL command mapping. All commands are formatted as:
//     #&!LQNQ.COMD<number>
// where <number> is one of the command numbers defined here

const std::string CMD_BASE("#&!LQNQ.COMD");
#define CMD_NQ_START                       "2525"
#define CMD_NQ_STOP                        "2828"
#define CMD_SET_WORKING_MODE               "2929"
#define CMD_GET_WORKING_MODE               "3030"
#define CMD_SET_OUTPUT_PERIOD              "0101"
#define CMD_GET_OUTPUT_PERIOD              "0202"
#define CMD_SET_TRIGGER_OUT                "5050"
#define CMD_GET_TRIGGER_OUT                "5151"
#define CMD_SET_OUTPUT_FORMAT              "0505"
#define CMD_RESET_OUTPUT_FORMAT            "1616"
#define CMD_GET_OUTPUT_FORMAT              "0606"
#define CMD_SET_SPEED_OF_SOUND             "5555"
#define CMD_GET_SPEED_OF_SOUND             "5656"
#define CMD_SET_ENABLE_USER_SOUND_VELOCITY "6565"
#define CMD_GET_ENABLE_USER_SOUND_VELOCITY "6666"

//==============================================================================
//                            FUNCTION DEFINITONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        parse_nq1_message
// Description: Parses a DVL NQ1 data message into a DvlNQ1Data struct.
// Arguments:   - line: NQ1 data message to be parsed
// Returns:     DvlNQ1Data struct containing DVL measurement info.
//------------------------------------------------------------------------------
inline DvlNQ1Data parse_nq1_message(std::string message)
{

    // Data struct to parse the message data into
    DvlNQ1Data nq1_data;

    // NQ1 format string for parsing with sscanf
    const char *format = "%*s "                         // $#NQ.RES
                         "%*s %u %u %u %u %f %f %f %f " // error_code, good_or_bad, v_altitude
                         "%f %f %f %f %f %f %f %f "     // velo_rad, wvelo_rad
                         "%u %u %u %u "                 // wvelo_credit
                         "%f %f %f %u"                  // velo_instrument, velo_instrument_flag
                         "%f %f %f %u"                  // velo_earth, velo_earth_flag
                         "%f %f %f %u"                  // water_velo_instrument, water_velo_instrument_flag
                         "%f %f %f %u"                  // water_velo_earth, water_velo_earth_flag
                         "%f %f %f"                     // roll, pitch, yaw
                         "%f %f %f %f"                  // altitude_estimate, dvl_temperature, dvl_pressure, salinity
                         "%u";                          // dvl_speed_of_sound

    // Use sscanf to parse the string into the struct using the above format
    sscanf(message.c_str(), format,
        &nq1_data.good_or_bad[0],           &nq1_data.good_or_bad[1],
        &nq1_data.good_or_bad[2],           &nq1_data.good_or_bad[3],
        &nq1_data.v_altitude[0],            &nq1_data.v_altitude[1],
        &nq1_data.v_altitude[2],            &nq1_data.v_altitude[3],
        &nq1_data.velo_rad[0],              &nq1_data.velo_rad[1],
        &nq1_data.velo_rad[2],              &nq1_data.velo_rad[3],
        &nq1_data.wvelo_rad[0],             &nq1_data.wvelo_rad[1],
        &nq1_data.wvelo_rad[2],             &nq1_data.wvelo_rad[3],
        &nq1_data.wvelo_credit[0],          &nq1_data.wvelo_credit[1],
        &nq1_data.wvelo_credit[2],          &nq1_data.wvelo_credit[3],
        &nq1_data.velo_instrument[0],       &nq1_data.velo_instrument[1],
        &nq1_data.velo_instrument[2],       &nq1_data.velo_instrument_flag,
        &nq1_data.velo_earth[0],            &nq1_data.velo_earth[1],
        &nq1_data.velo_earth[2],            &nq1_data.velo_earth_flag,
        &nq1_data.water_velo_instrument[0], &nq1_data.water_velo_instrument[1],
        &nq1_data.water_velo_instrument[2], &nq1_data.water_velo_instrument_flag,
        &nq1_data.water_velo_earth[0],      &nq1_data.water_velo_earth[1],
        &nq1_data.water_velo_earth[2],      &nq1_data.water_velo_earth_flag,
        &nq1_data.rpy[0],                   &nq1_data.rpy[1],
        &nq1_data.rpy[2],                   &nq1_data.altitude_estimate,
        &nq1_data.dvl_temperature,          &nq1_data.dvl_pressure,
        &nq1_data.salinity,                 &nq1_data.dvl_speed_of_sound);

    // TEMP: Why does the DVL report negative velocity?
    nq1_data.velo_instrument[0] = -nq1_data.velo_instrument[0];
    nq1_data.velo_instrument[1] = -nq1_data.velo_instrument[1];
    nq1_data.velo_instrument[2] = -nq1_data.velo_instrument[2];

    return nq1_data;

}

//------------------------------------------------------------------------------
// Name:        DVL_NQ_START
// Description: Command to leave configuration mode and begin reporting
//              velocity measurements.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_NQ_START()
{
    return CMD_BASE + CMD_NQ_START + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_NQ_STOP
// Description: Command to enter configuration mode and stop reporting
//              velocity measurements.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_NQ_STOP()
{
    return CMD_BASE + CMD_NQ_STOP + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_SET_WORKING_MODE
// Description: Command to set the DVL working mode.
//                  mode = 0: power-on continuous
//                  mode = 1: command continuous
//                  mode = 2: command step (trigger in mode)
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_SET_WORKING_MODE(int mode)
{
    return CMD_BASE + CMD_SET_WORKING_MODE + " " + std::to_string(mode) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_GET_WORKING_MODE
// Description: Command to get the DVL working mode currently set on the
//              DVL.
//                  mode = 0: power-on continuous
//                  mode = 1: command continuous
//                  mode = 2: command step (trigger in mode)
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_GET_WORKING_MODE()
{
    return CMD_BASE + CMD_GET_WORKING_MODE + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_SET_OUTPUT_PERIOD
// Description: Command to set the DVL output period in seconds. Precision
//              up to 0.01 seconds.
// Arguments    - period: output period in seconds
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_SET_OUTPUT_PERIOD(double period)
{
    char period_char[128];
    sprintf(period_char, "%.2f", period);
    return CMD_BASE + CMD_SET_OUTPUT_PERIOD + " " + period_char + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_GET_OUTPUT_PERIOD
// Description: Command to leave configuration mode and begin reporting
//              velocity measurements.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_GET_OUTPUT_PERIOD()
{
    return CMD_BASE + CMD_GET_OUTPUT_PERIOD + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_SET_TRIGGER_OUT
// Description: Command to enable or disable the DVL trigger out option.
// Arguments:   - enable: true to enable trigger out, false to disable
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_SET_TRIGGER_OUT(bool enable)
{
    return CMD_BASE + CMD_SET_TRIGGER_OUT + " "
        + std::to_string((int)enable) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_GET_TRIGGER_OUT
// Description: Command to get the status of the DVL trigger out option.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_GET_TRIGGER_OUT()
{
    return CMD_BASE + CMD_GET_TRIGGER_OUT + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_SET_OUTPUT_FORMAT
// Description: Command to enable the specified DVL output format. Multiple
//              formats may be enabled by sending this command more than
//              once with  different formats.
//                  format =  0: All formats?
//                  format =  1: NQ1
//                  format =  2: Multi-Cell Current Profiling (radial)
//                  format =  3: Multi-Cell Current Profiling (instrument)
//                  format =  4: Multi-Cell Current Profiling (earth)
//                  format = 13: WH300 PD4 (Beam Coordinate)
//                  format = 12: WH300 PD6 (Instrument Coordinate)
//                  format = 11: WH300 PD4 (Instrument Coordinate)
//                  format = 32: VDDPT
//                  format = 31: GPDPT
//                  format = 33: ParoScientific
//                  format = 62: NQ Info
// Arguments:   - format: output format option
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_SET_OUTPUT_FORMAT(int format)
{
    return CMD_BASE + CMD_SET_OUTPUT_FORMAT + " "
        + std::to_string(format) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_RESET_OUTPUT_FORMAT
// Description: Command to disable the specified DVL output format. Multiple
//              formats may be disabled by sending this command more than
//              once with  different formats.
//                  format =  0: All formats?
//                  format =  1: NQ1
//                  format =  2: Multi-Cell Current Profiling (radial)
//                  format =  3: Multi-Cell Current Profiling (instrument)
//                  format =  4: Multi-Cell Current Profiling (earth)
//                  format = 13: WH300 PD4 (Beam Coordinate)
//                  format = 12: WH300 PD6 (Instrument Coordinate)
//                  format = 11: WH300 PD4 (Instrument Coordinate)
//                  format = 32: VDDPT
//                  format = 31: GPDPT
//                  format = 33: ParoScientific
//                  format = 62: NQ Info
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_RESET_OUTPUT_FORMAT(int format)
{
    return CMD_BASE + CMD_RESET_OUTPUT_FORMAT + " "
        + std::to_string(format) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_GET_OUTPUT_FORMAT
// Description: Command to get a list of the DVL output formats currently
//              enabled.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_GET_OUTPUT_FORMAT()
{
    return CMD_BASE + CMD_GET_OUTPUT_FORMAT + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_SET_SPEED_OF_SOUND
// Description: Command to set the DVL speed of sound in m/s.
// Arguments:   - speed: speed of sound in m/s
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_SET_SPEED_OF_SOUND(int speed)
{
    return CMD_BASE + CMD_SET_SPEED_OF_SOUND + " "
        + std::to_string(speed) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_GET_SPEED_OF_SOUND
// Description: Command to get the DVL speed of sound in m/s.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_GET_SPEED_OF_SOUND()
{
    return CMD_BASE + CMD_GET_SPEED_OF_SOUND + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_SET_ENABLE_USER_SOUND_VELOCITY
// Description: Command to enable the user-set speed of sound value. If this is
//              not enabled, the DVL will calculate the speed of sound itself.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_SET_ENABLE_USER_SOUND_VELOCITY(bool enable)
{
    return CMD_BASE + CMD_SET_ENABLE_USER_SOUND_VELOCITY + " "
        + std::to_string((int)enable) + "\r\n";
}

//------------------------------------------------------------------------------
// Name:        DVL_GET_ENABLE_USER_SOUND_VELOCITY
// Description: Command to get the status of the user-set speed of sound
//              value. If this is not enabled, the DVL will calculate the
//              speed of sound itself.
// Returns:     Corresponding command string.
//------------------------------------------------------------------------------
inline std::string DVL_GET_ENABLE_USER_SOUND_VELOCITY()
{
    return CMD_BASE + CMD_GET_ENABLE_USER_SOUND_VELOCITY + "\r\n";
}

#endif // LINKQUEST_DVL_H
