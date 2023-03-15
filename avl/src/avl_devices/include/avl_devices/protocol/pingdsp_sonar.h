//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides functions to configure and run a Ping DSP custom sonar
//              module by interfacting with a windows computer running
//              Ping DPS's 3DSS control software.
//==============================================================================

#ifndef PINGDSP_SONAR_H
#define PINGDSP_SONAR_H

// Util functions
#include <avl_core/util/math.h>

//==============================================================================
//                            TYPEDEF DECLARATIONS
//==============================================================================

// Ports used by 3DSS sonar control software
#define PINGDSP_SONAR_CONTROL_PORT 23840
#define PINGDSP_SONAR_DATA_PORT 23848

// Expected preamble for sonar data
const std::vector<uint8_t> PINGDSP_SONAR_DATA_HEADER = { 0x50, 0x49, 0x4E, 0x47,
                                                       0x27, 0x2B, 0x3A, 0xD8,
                                                       0x74, 0x2A, 0x1C, 0x33,
                                                       0xE9, 0xB0, 0x73, 0xB1 };

// Bathymetry configuration parameters
typedef struct BathymetryConfig
{

    double min_depth; // (-1 to 200)
    double max_depth; // (0 to 200)
    double swath; // (0 to 20)
    int bin_count; // (3 to 1440)
    double bin_width; // (0.05 to 2)
    int bottom_track_cells; // (3 to 256)
    double bottom_track_width; // (0.1 to 10)
    double bottom_track_height; // (0.1 to 10)
    double bottom_track_height_percent; // (0 to 25)
    double bottom_track_alpha; // (0.01 to 0.99)

} BathymetryConfig;

// Struct to represent 3D sonar points in Geo coordinates
typedef struct GeoSonarPoint
{

    double lat; // radians
    double lon; // radians
    double alt; // meters from mean sea level
    double intensity; // Intensity of the point

} GeoSonarPoint;

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        PINGDSP_INIT_SONAR_MODE
// Description: Command to initialize the 3DSS control software into sonar mode.
//------------------------------------------------------------------------------
inline std::string PINGDSP_INIT_SONAR_MODE()
{
    return std::string("app --init --mode=sonar\r\n");
}

//------------------------------------------------------------------------------
// Name:        PINGDSP_CONNECT
// Description: Command to connect the 3DSS control software to the sonar
//              electronics module.
//------------------------------------------------------------------------------
inline std::string PINGDSP_CONNECT()
{
    return std::string("sonar --connect\r\n");
}

//------------------------------------------------------------------------------
// Name:        PINGDSP_UPDATE_TIME
// Description: Command to update the sonar electronics module's time from the
//              sonar computer's time.
//------------------------------------------------------------------------------
inline std::string PINGDSP_UPDATE_TIME()
{
    return std::string("sonar --updatetime\r\n");
}

//------------------------------------------------------------------------------
// Name:        PINGDSP_COMMIT
// Description: Command to connect the 3DSS control software to the sonar
//              electronics module.
//------------------------------------------------------------------------------
inline std::string PINGDSP_COMMIT()
{
    return std::string("commit\r\n");
}

//------------------------------------------------------------------------------
// Name:        PINGDSP_SET_SOUND_SPEED
// Description: Command to set the speed of sound value in m/s used by the
//              sonar. Values out out of range will be truncated.
// Arguments:   - sound_speed: speed of sound in m/s (range: 1300 to 1600)
//------------------------------------------------------------------------------
inline std::string PINGDSP_SET_SOUND_SPEED(double sound_speed)
{

    // Truncate the sound speed to its range
    sound_speed = avl::clamp(sound_speed, 1300.0, 1600.0);

    // Format the sound speed command
    char command[128];
    sprintf(command, "sv --bulk=%.3f --face=%.3f\r\n",
        sound_speed, sound_speed);

    return command;

}

//------------------------------------------------------------------------------
// Name:        PINGDSP_SET_GAIN
// Description: Command to set the sonar constant, linear, and log gain
//              values. Values out out of range will be truncated. According
//              to Ping DSP's document, standard values are const = 0,
//              linear = 0.2, and log = 30. Both port and starboard
//              parameters are set symmetrically.
// Arguments:   - const_gain: constant gain in dB (range: -60 to 60)
//              - linear_gain: linear gain in dB/m (range: 0 to 3.5)
//              - log_gain: log gain in dB/log(m) (range: 0 to 60)
//------------------------------------------------------------------------------
inline std::string PINGDSP_SET_GAIN(double const_gain, double linear_gain,
    double log_gain)
{

    // Truncate the gains to their ranges
    const_gain = avl::clamp(const_gain, -60.0, 60.0);
    linear_gain = avl::clamp(linear_gain, 0.0, 3.5);
    log_gain = avl::clamp(log_gain, 0.0, 60.0);

    // Format the gain command
    char command[128];
    sprintf(command, "gain --const=%.3f --linear=%.3f --log=%.3f\r\n",
        const_gain, linear_gain, log_gain);

    return command;

}

//------------------------------------------------------------------------------
// Name:        PINGDSP_SET_SIDESCAN_3D
// Description: Command to set the sonar's 3D sidescan parameters. Both port
//              and starboard parameters are set symmetrically.
// Arguments:   - angles: number of angles (1, 2, 3, 4)
//              - smoothing: amount of smoothing (0, 2, 3, 5, 10, 15, 20,
//                25, 30, 40, 50)
//              - threshold: minimum energy threshold in dB (-150 to 0)
//              - tolerance: solution tolerance (0.001 to 0.999)
//------------------------------------------------------------------------------
inline std::string PINGDSP_SET_SIDESCAN_3D(int angles, int smoothing,
    double threshold, double tolerance)
{

    // Format the sidescan3d command
    char command[128];
    sprintf(command, "sidescan3d --angles=%d --smoothing=%d --threshold=%.3f --tolerance=%.3f\r\n",
        angles, smoothing, threshold, tolerance);

    return command;

}

//------------------------------------------------------------------------------
// Name:        PINGDSP_SET_BATHYMETRY
// Description: Command to set the sonar's bathymetry parameters.
// Arguments:   - config: bathymetry configuration parameter struct
//------------------------------------------------------------------------------
inline std::string PINGDSP_SET_BATHYMETRY(BathymetryConfig config)
{

    // Format the bathymetry command
    char command[1024];
    sprintf(command, "bathymetry --mindepth=%.3f --maxdepth=%.3f --swath=%.3f --binning=equidistant:%d:%.3f --bottomtrack=cartesian:%d:%.3f:%.3f:%.3f:%.3f\r\n",
        config.min_depth,
        config.max_depth,
        config.swath,
        config.bin_count,
        config.bin_width,
        config.bottom_track_cells,
        config.bottom_track_width,
        config.bottom_track_height,
        config.bottom_track_height_percent,
        config.bottom_track_alpha);

    return command;

}

//------------------------------------------------------------------------------
// Name:        PINGDSP_SET_TRANSMIT
// Description: Command to set the sonar's ping transmission parameters.
//              Pulse type  bb80 requires a range >= 50 and pulse types
//              bb550 and bb700 require a range >= 150m. Both port and
//              starboard parameters are set symmetrically.
// Arguments:   - pulse_type: pulse type (none, nb10, nb15, nb25, nb50, bb80,
//                bb200, bb550, bb700)
//              - power: transmit power percentage (1, 2, 5, 10, 20, 50, 80,
//                100)
//              - beamwidth: beamwidth in degrees (19, 22, 30, 44, 55, 90)
//              - angle: beam angle in degrees (-45, -30, -20. -15, -10, 0,
//                10, 15, 20, 30, 45)
//------------------------------------------------------------------------------
inline std::string PINGDSP_SET_TRANSMIT(std::string pulse_type, int power,
    int beamwidth, int angle)
{

    // Format the transmit command
    char command[128];
    sprintf(command, "transmit --pulse=%s --power=%d --beamwidth=%d --angle=%d\r\n",
        pulse_type.c_str(), power, beamwidth, angle);

    return command;

}

//------------------------------------------------------------------------------
// Name:        PINGDSP_SET_RANGE
// Description: Command to set the sonar range in meters. The sonar may be
//              set to any of the following ranges:
//              (15, 25, 37, 50, 75, 100, 150, 200, 250)
//              An exception will be thrown if an invalid range is chosen.
// Arguments:   - range: sonar range in meters, chosen from valid values
//------------------------------------------------------------------------------
inline std::string PINGDSP_SET_RANGE(int range)
{

    // Format the acquisition command with the range. Assume 100% duty cycle and
    // continuous trigger
    char command[128];
    sprintf(command, "acquisition --range=%d --dutycycle=100 --trigger=continuous\r\n",
        range);

    return command;

}

//------------------------------------------------------------------------------
// Name:        PINGDSP_START_DATA_STREAM
// Description: Command to start the sonar data stream. Data can be recorded
//              with the PINGDSP_START_RECORDING command.
//------------------------------------------------------------------------------
inline std::string PINGDSP_START_DATA_STREAM()
{
    return std::string("sonar --run\r\n");;
}

//------------------------------------------------------------------------------
// Name:        PINGDSP_STOP_DATA_STREAM
// Description: Command to stop the sonar data stream.
//------------------------------------------------------------------------------
inline std::string PINGDSP_STOP_DATA_STREAM()
{
    return std::string("sonar --stop\r\n");
}

//------------------------------------------------------------------------------
// Name:        PINGDSP_START_RECORDING
// Description: Command to open a .3dss-dx file on the sonar computer and
//              start recording the sonar data stream.
// Arguments:   - filename: path to file to record data
//              - overwrite: true to overwrite the file if it already exists
//------------------------------------------------------------------------------
inline std::string PINGDSP_START_RECORDING(std::string filename, bool overwrite)
{
    if (overwrite)
        return "record --start --overwrite --filename=\"" + filename + "\"\r\n";
    else
        return "record --start --filename=\"" + filename + "\"\r\n";
}

//------------------------------------------------------------------------------
// Name:        PINGDSP_STOP_RECORDING
// Description: Command to stop recording incoming sonar data.
//------------------------------------------------------------------------------
inline std::string PINGDSP_STOP_RECORDING()
{
    return std::string("record --stop\r\n");
}

#endif // PINGDSP_SONAR_H
