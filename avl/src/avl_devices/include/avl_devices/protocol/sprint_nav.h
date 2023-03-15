//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides communication protocols for the Sonardyne SprintNav
//==============================================================================

#ifndef SPRINT_NAV_H
#define SPRINT_NAV_H

#include <avl_core/util/math.h>
#include <avl_core/util/string.h>
#include <Eigen/Dense>
#include <unordered_map>
using Eigen::MatrixXd;
using Eigen::VectorXd;

//==============================================================================
//                            GLOBAL CONSTANT DEFINITIONS
//==============================================================================

const uint8_t DLE{0x10}; //Data Link Escape character
const uint8_t STX{0x02}; //Start of Text character
const uint8_t ETX{0x03}; //End of Text character
const uint8_t max_connect_attempts{5}; //Max attempts to reconnect to device
constexpr bool LSB = true; //Defines messages that are sent LSB first
//==============================================================================
//                            STRUCTURES
//==============================================================================
typedef struct Obst
{
    double time;  //system time (Unix s) of first data receipt

    uint16_t rejection;     //16 but rejection status

    float mahad;            //mahalanobis distance

    std::vector<uint8_t> specific;  //bytes specific to sensor type
} Obst;

typedef struct Status
{
    struct Tms
    {
        double sys_time;
        double utc;                 //time in s
        double time_since_update;

        float sd;                   //expected standarad deviation (s)

        uint8_t source;             //0=none,1=rtc,2=zda,3=gga,4=zda+pps
        uint8_t zda_count;
        uint8_t pps_count;
        uint8_t zda_rej_count;      //LS byte of the counts
        uint8_t pps_rej_count;
        uint8_t zda_pps_proc_count;
        uint8_t filter_reset_count;

        bool pps_rising;            //false if valid on falling edge
    } tms;

    struct Bist
    {
        double sys_time; //time (s)

        uint64_t imu;
        uint64_t comms;
        uint64_t cca;
        uint64_t ains;
        uint16_t ahrs;

        std::string fw; //major.minor.interim build
    } bist;
} Status;

typedef struct Pwrstat
{
    double ext_pwr;                 // external power in volts

    double fifteen_v;               // 15 volt bus in volts
    double fifteen_v_i;             // 15 volt bus current in amps

    double five_v;                  // 5 volt bus in volts
    double five_v_i;                // 5 volt bus current in amps

    double neg_fifteen_v;           // -15 volt bus

    double battery_v;               // battery volts
    double battery_temp;            // battery temperature in Celsius

    std::string status;             // status in string
    std::string pic_charger_stat;   // PIC / charger status in string
} Pwrstat;


typedef struct Config
{
    size_t output_rate;   //Hz
    size_t log_rate;      //Hz
} Config;


typedef struct Dvl
{
    uint8_t config;

    uint16_t bit;
    uint16_t sound_speed;      //m/s
    uint16_t checksum;
    bool shallow_current;   //altitude is too shallow for reference layer

    float current_layer[2]; //range from vehicle of reference layer (m)
    float tofp;             //time of first ping (s)
    float temp;             //water temp at transducer head (degC)

    struct Xyz
  {
        float x;
        float y;           //speed (m/s)
        float z;
        float err;         //TODO verify this is error
        bool valid;
    } v, current;         //bottom velocity, current/reference velocity wrt vehicle

    struct Beam  //PD4: Beams 1&3 are FWD; Beams 2&3 are STBD
    {
        float range;
        bool correlation;
        bool echo;  //invert these three bits to remove negative
        bool current_correlation;
    } beam1, beam2, beam3, beam4;


} Dvl;

typedef struct Nav
{
    double time;            //Unix s
    double latitude;        //degrees
    double longitude;       //degrees
    float depth;            //meters
    float altitude;         //meters
    float roll;             //degrees
    float pitch;            //degrees
    float heading;          //degrees

    double vN;              //North m/s
    double vE;              //East m/s
    double vDwn;            //Down m/s
    double wFwd;            //Roll deg/s
    double wStbd;           //Pitch deg/s
    double wDwn;            //Yaw deg/s
    double aFwd;            //m/s^2
    double aStbd;           //m/s^2
    double aDwn;            //m/s^2
    double vX;              //m/s
    double vY;              //m/s
    double vZ;              //m/s

    double speed;           //3D velocity magnitude m/s
    double course;          //velocity degrees from North
    double gamma;           //velocity degrees from Horizontal

    float posMajor;         //meters; horizontal pos 1sigma error ellipse
    float posMinor;         //meters; semi-minor axis
    float dirPMajor;        //degrees; direction of semi-major axis
    float stdDepth;         //meters; 1sigma depth error
    float stdLevN;          //degrees; 1sigma error about North
    float stdLevE;          //degrees; 1sigma error about East
    float stdHeading;       //degrees; 1sigma heading error
    float velMajor;         //m/s; horizontal velocity 1sigma error ellipse
    float velMinor;         //m/s; semi-minor axis
    float dirVMajor;        //degrees; direction of semi-major axis
    float velDown;          //m/s; 1sigma down velocity error

    struct LNavStatus
    {
        uint16_t status;
        std::string message;
        bool orientation, pos, alt, orientation_source, susbl, depth, dvl, lbl, zupt,
            xpos, gps, zmd, usbl;
    } status;
} Nav;

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        BIST_TOSTRING
// Description: Convert the bist into a string
// Arguments:   - flag: a uint64_t of a BIST
//              - type: an integer representing what BIST it is
//                      0 is imu, 1 is comms, 2 is cca, 3 is ains, and
//                      4 is ahrs
// Returns:     A string of the error messages from the BIST
//------------------------------------------------------------------------------
  std::string BIST_TOSTRING(uint64_t flag, int type)
  {
    std::unordered_map<int, std::string> imu_map;
    imu_map[0] = "Serious hardware issue (no go)";
    imu_map[1] = "ISA/IMU data not ok";
    imu_map[2] = "Firmware not started";
    imu_map[3] = "Gyro power not ok";
    imu_map[4] = "X gyro problem - tripped or sensor status not Normal";
    imu_map[5] = "Y gyro problem - tripped or sensor status not Normal";
    imu_map[6] = "Z gyro problem - tripped or sensor status not Normal";
    imu_map[7] = "External power supply not ok";
    imu_map[8] = "Battery not ok";
    imu_map[9] = "Real Time Clock not ok";
    imu_map[10] = "X accelerometer sensor temperature not ok (> 80 C)";
    imu_map[11] = "Y accelerometer sensor temperature not ok (> 80 C)";
    imu_map[12] = "Z accelerometer sensor temperature not ok (> 80 C)";
    imu_map[13] = "X accelerometer board temperature not ok (> 80 C)";
    imu_map[14] = "Y accelerometer board temperature not ok (> 80 C)";
    imu_map[15] = "Z accelerometer board temperature not ok (> 80 C)";
    imu_map[16] = "Accelerometer out of range";
    imu_map[17] = "AHRS result not ok";
    imu_map[32] = "Shutdown requested";
    imu_map[33] = "Current flash not used";
    imu_map[34] = "PIC not authenicated";
    imu_map[35] = "Outputs Restricted for Export Reasons";

    std::unordered_map<int, std::string> comms_map;
    comms_map[14] = "UART0 Rx General error (timetag overflow or frame or parity error)";
    comms_map[15] = "UART1 Rx General error (timetag overflow or frame or parity error)";
    comms_map[16] = "UART2 Rx General error (timetag overflow or frame or parity error)";
    comms_map[17] = "UART3 Rx General error (timetag overflow or frame or parity error)";
    comms_map[18] = "UART4 Rx General error (timetag overflow or frame or parity error)";
    comms_map[19] = "UART0 Tx Overflow error";
    comms_map[20] = "UART1 Tx Overflow error";
    comms_map[21] = "UART2 Tx Overflow error";
    comms_map[22] = "UART3 Tx Overflow error";
    comms_map[23] = "UART4 Tx Overflow error";
    comms_map[24] = "TRIG1 Rx Overrun";
    comms_map[25] = "TRIG2 Rx Overrun";
    comms_map[26] = "TRIG3 Rx Overrun";
    comms_map[27] = "TRIG4 Rx Overrun";
    comms_map[39] = "Ethernet Port0 Error";
    comms_map[40] = "Ethernet Port1 Error";
    comms_map[41] = "Ethernet Port2 Error";
    comms_map[42] = "Ethernet Port3 Error";
    comms_map[43] = "Ethernet Port4 Error";
    comms_map[47] = "Ethernet Reset";
    comms_map[48] = "Multiplex Protocol Checlsum error";
    comms_map[49] = "UART 0 Rx Parity Error";
    comms_map[50] = "UART 1 Rx Parity Error";
    comms_map[51] = "UART 2 Rx Parity Error";
    comms_map[52] = "UART 3 Rx Parity Error";
    comms_map[53] = "UART 4 Rx Parity Error";
    comms_map[54] = "UART 0 Rx Overrun Error";
    comms_map[55] = "UART 1 Rx Overrun Error";
    comms_map[56] = "UART 2 Rx Overrun Error";
    comms_map[57] = "UART 3 Rx Overrun Error";
    comms_map[58] = "UART 4 Rx Overrun Error";
    comms_map[59] = "UART 0 Rx Frame Error";
    comms_map[60] = "UART 1 Rx Frame Error";
    comms_map[61] = "UART 2 Rx Frame Error";
    comms_map[62] = "UART 3 Rx Frame Error";
    comms_map[63] = "UART 4 Rx Frame Error";

    std::unordered_map<int, std::string> cca_map;
    cca_map[15] = "Probelm setting up UARTs at startup";
    cca_map[16] = "SD card nto initialzied";
    cca_map[17] = "SD card file or directory deletion not ok";
    cca_map[18] = "Power Pass on T1 on";
    cca_map[19] = "Power Pass on T2 on";
    cca_map[20] = "Power Pass on T1 tripped";
    cca_map[21] = "Power Pass on T2 tripped";
    cca_map[23] = "Problem reading from or writing to the SD card";
    cca_map[25] = "PIC HW problem";
    cca_map[26] = "Power Pass on C1 On";
    cca_map[27] = "Power Pass on C1 Tripped";
    cca_map[28] = "Noise detected on Trigger output configured for DVL";
    cca_map[29] = "Output DVL trigger not detected, output line may be shorted";
    cca_map[30] = "Noise deteed on Trigger output for Lodestar generated PPS";
    cca_map[31] = "Output of Lodestar PPS not detected, output line may be shorted";
    cca_map[32] = "Firmware issue";

    std::unordered_map<int, std::string> ains_map;
    ains_map[1] = "AINS uninitialized";
    ains_map[2] = "no init orientation (AHRS not settled)";
    ains_map[4] = "no init position available for initialization from enabled sensors";
    ains_map[5] = "no init depth available for initialization from enabled sensors";
    ains_map[31] = "ZMD Bias large";
    ains_map[32] = "Horizontal position 1DRMS high - INS auto re-initialize";
    ains_map[33] = "Heading unreasonable";
    ains_map[34] = "Attitude unreasonable";
    ains_map[35] = "Gyro error large";
    ains_map[36] = "Accel error large";

    std::unordered_map<int, std::string> ahrs_map;
    ahrs_map[1] = "Unsettled";
    ahrs_map[2] = "Not position aided for > 10s";
    ahrs_map[3] = "Not velocity aided for > 10s";

    std::unordered_map<int, std::string> error_map;
    std::string answer = "";
    switch(type)
    {
    case 0:
        error_map = imu_map;
        break;
    case 1:
        error_map = comms_map;
        break;
    case 2:
        error_map = cca_map;
        break;
    case 3:
        error_map = ains_map;
        break;
    case 4:
        error_map = ahrs_map;
        break;
    }

    std::unordered_map<int, std::string>:: iterator i;
    for (i = error_map.begin(); i != error_map.end(); i++)
    {
        if ((flag >> i->first) & 1)
        {
            answer += i->second;
            answer += ", ";
        }
    }

    return answer;
}


//------------------------------------------------------------------------------
// Name:        SPRINTNAV_CHECKSUM
// Description: Calculate byte-wise exclusive-OR
// Arguments:   - packet: the pre-byte stuffed fields:ID, Port pass/snoop info,
// 		  Timestamp, and payload.
// Returns:     Corresponding checksum in hexidecimal.
//------------------------------------------------------------------------------
uint8_t SPRINTNAV_CHECKSUM(std::vector<uint8_t> packet)
{
    uint8_t checksum{};
    //calculate checksum
    for (uint8_t i = 0; i < packet.size() ; i++)
	  checksum ^= packet[i];

    return checksum;
}

//------------------------------------------------------------------------------
// Name:        CHECKSUM_CHECK
// Description: Compare reported checksum to calculated checksum
// Arguments:   - message: the checksum and pre-byte stuffed fields:ID, Port
//              pass/snoop info, Timestamp, and payload.
// Returns:     True if calculation matches reported.
//------------------------------------------------------------------------------
bool CHECKSUM_CHECK(std::vector<uint8_t> message)
{
    uint8_t checksum_reported = message.back();
    message.pop_back();
    uint8_t checksum_calculated = SPRINTNAV_CHECKSUM(message);
    return (checksum_reported == checksum_calculated);
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_UNWRAP
// Description: Returns a payload from a sprintnav multiplex formatted message
// Arguments:   - message: formatted message from the sprintnav
//              - &early_term: boolean to signal if more data is needed
// Returns:     message payload
//------------------------------------------------------------------------------
std::vector<uint8_t> SPRINTNAV_UNWRAP(std::vector<uint8_t> message,
  bool &early_term)
{
    bool DLEReceived = false;
    bool DLE_STX_found = false;
    bool DLE_ETX_found = false;
    std::vector<uint8_t> payload{};

    for (size_t i = 0; i < message.size(); i++)
    {
        uint8_t Databyte = message.at(i);

        switch (Databyte)
        {
            case DLE:
                if (DLEReceived == false)
                  //first DLE, wait for the next DLE to determine its use
                    DLEReceived = true;
                else if (DLE_STX_found)
                // record DLE pairs in the packet, but not if DLE_STX hasn't been found
                {
                    //DLE DLE found -- copy one DLE to payload buffer
                    DLEReceived = false;
                    payload.push_back(Databyte);
                }
                break;

            case STX:
                if (DLEReceived == true)
                {
                    // DLE STX Start of the payload found
                    DLEReceived = false;
                    DLE_STX_found = true;
                }
                else
                {
                    if (DLE_STX_found)
                        payload.push_back(Databyte);
                }
                break;

            case ETX:
                if (DLEReceived == true)
                {
                    //DLE ETX found
                    DLEReceived = false;
                    if (DLE_STX_found == true)
                        DLE_ETX_found = true;
                    DLE_STX_found = false;
                }
                else
                {
                    if (DLE_STX_found)
                        payload.push_back(Databyte);
                }
                break;

            default: //Data
                if (DLEReceived == true)
                    DLEReceived = false;
                if (DLE_STX_found)
                    payload.push_back(Databyte);
                break;

        }

    }

    if (!DLE_ETX_found)
        early_term =true;

    return payload;
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_MULTIPLEX
// Description: Wraps a payload packet with Sonardyne's multiplex format
// Arguments:   - packet: all bytes to be wrapped in multiplex format
// 		- mid: payload ID (ie NMEA GGA is 64)
// Returns:     Multiplex wrapped packet. Ready for transmission to SprintNav
//------------------------------------------------------------------------------
std::vector<uint8_t> SPRINTNAV_MULTIPLEX(std::vector<uint8_t> packet,
	       	uint16_t mid)
{
    //build the packet middle-out from the payload to be agnostic to optional
    //packets


    //-----Insert timestamp (optional)----
    //no timestamp to insert since not valid for input payloads to SprintNav


    //-----Insert Port Info (optional)----
    //TODO populate this packet when needed


    //-----Insert ID packet---------------
    uint16_t id{0}; //Define ID, bytes 2 & 3 of multiplex payload

    //Add timestamp indicator
    //populate with timestamp bit if ever needed

    //Add the SID
    //populate with SID data if ever needed

    //Insert Message ID
    id |= mid;

    uint8_t id_MSB = (id >> 8) & 0xFF;
    uint8_t id_LSB = id & 0xFF;
    packet.insert(packet.begin() , id_MSB );
    packet.insert(packet.begin() + 1, id_LSB );


    //-----Insert checksum-----------------
    packet.push_back( SPRINTNAV_CHECKSUM(packet) );


    //-----Byte stuffing------------------
    //search packet for any 'DLE' (0x10) characters for byte stuffing
    for (size_t i = 0; i < packet.size(); i++)
    {
        if ( packet[i] == DLE)
        {
            //Stuff DLE byte
            packet.insert( (packet.begin() + i) , DLE);
            i++; //iterate to move to next character
        }
    }


    //-----Insert start delimiter---------
    packet.insert(packet.begin() , DLE);
    packet.insert(packet.begin() + 1 , STX);


    //-----Insert end delimiter---------
    packet.push_back(DLE);
    packet.push_back(ETX);


    return packet;
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_UINT48_TIME
// Description: Converts a uint48 type in 1e-6 s to double in s
// Arguments:   - time_vector: vector containing uint48t time bytes
// 		- iterator: start position of uint48t within time_vector
// Returns:     Time in s
//------------------------------------------------------------------------------
double SPRINTNAV_UINT48_TIME(std::vector<uint8_t> time_vector,
    size_t iterator = 0)
{
    constexpr size_t byte_length = 6; //48 bits is 6 bytes
    time_vector = avl::subvector(time_vector, iterator, byte_length);
    //Vector needs to be 8 bytes to use avl::from_bytes
    avl::append(time_vector, {0x00, 0x00});

    uint64_t time = avl::from_bytes<uint64_t>(time_vector, LSB);
    return time/1e6;
}

//------------------------------------------------------------------------------
// Name:        COORD_TRANSFORM
// Description: Convert coordinate reference frame given euler angle and
//              initial vector.
// Arguments:   - theta:    euler-angle vector
//              - v_0:      vector from initial coordinate frame
//              - degrees:  specifies if theta is in degrees; defaults to false
// Returns:     Vector in new coordinate frame.
//------------------------------------------------------------------------------
Vector3d COORD_TRANSFORM(Vector3d theta, Vector3d v_0, bool degrees = false)
{
    MatrixXd C = avl::euler_to_matrix(theta, degrees);
    return C * v_0;
}

//------------------------------------------------------------------------------
// Name:        NED_TO_VEHICLE
// Description: Convert from North, East, Down frame to x, y, z.
// Arguments:   - *nav: current nav status pointer
// Returns:     Vector in vehicle x, y, z.
//------------------------------------------------------------------------------
void NED_TO_VEHICLE(Nav *nav)
{
    Vector3d theta(nav->roll, nav->pitch, nav->heading);
    Vector3d ned(nav->vN, nav->vE, nav->vDwn);
    //Convert from NED to vehicle frame
    Vector3d v_vehicle =  COORD_TRANSFORM(theta, ned, true);
    nav->vX = v_vehicle(0);
    nav->vY = v_vehicle(1);
    nav->vZ = v_vehicle(2);
    return;
}

//------------------------------------------------------------------------------
// Name:        SPEED_3D
// Description: Calculate total vehicle speed, course in the horizontal (N and
//              E) plane, and elevation angle (degrees from horizontal)
// Arguments:   - *nav: current nav status pointer
//------------------------------------------------------------------------------
void SPEED_AND_COURSE(Nav *nav)
{
    std::vector<double> s_course{};
    // Speed is absolute value of N, E, and down velocities
    nav->speed = sqrt( (nav->vN)*(nav->vN) + (nav->vE)*(nav->vE) + (nav->vDwn)*
        (nav->vDwn) );
    // Course is arctangent of N and E velocities. atan2 accounts for quadrant
    nav->course = atan2(nav->vE, nav->vN);
    // Convert -pi/pi to 0/2pi. Then radians to degrees
    if(nav->course < 0)
        nav->course += 2.0 * 3.141592654;
    nav->course *= 360.0/ (2.0 * 3.141592654);

    // Gamma is arcsin of total speed and -vDwn
    nav->gamma = asin( (-nav->vDwn) / (nav->speed) );
    nav->gamma *= 360.0/ (2.0 * 3.141592654);

    return;
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_LNAVUTC
// Description: Reads an LNAVUTC payload from the SprintNav and updates the
//              current navigation status structure
// Arguments:   - payload: LNAVUTC payload
//              - *nav: current navigation status structure pointer
// Returns:     - none
//------------------------------------------------------------------------------
void SPRINTNAV_LNAVUTC(std::vector<uint8_t> payload, Nav *nav)
{
    nav->time = SPRINTNAV_UINT48_TIME(payload, 1);
    nav->time *= 10; //LNAVUTC is 1e-5 s, so multiply by 10

    nav->latitude = avl::from_bytes<int32_t>(avl::subvector(payload, 7, 4), LSB);
    nav->latitude = (nav->latitude*90)/0x80000000;
    nav->longitude = avl::from_bytes<int32_t>(avl::subvector(payload, 11, 4), LSB);
    nav->longitude = (nav->longitude*180)/0x80000000;
    nav->depth = avl::from_bytes<int32_t>(avl::subvector(payload, 15, 4), LSB);
    nav->depth /=1000;

    // Altitude is NaN if raw measurement == 0
    uint16_t altitude_cm = avl::from_bytes<uint16_t>
        (avl::subvector(payload, 19, 2), LSB);
    nav->altitude = altitude_cm / 1e2;
    if ( altitude_cm == 0)
        nav->altitude = NAN;

    nav->roll = avl::from_bytes<int16_t>(avl::subvector(payload, 21, 2), LSB);
    nav->roll = (nav->roll*180)/0x8000;
    nav->pitch = avl::from_bytes<int16_t>(avl::subvector(payload, 23, 2), LSB);
    nav->pitch = (nav->pitch*180)/0x8000;
    nav->heading = avl::from_bytes<uint16_t>(avl::subvector(payload, 25, 2), LSB);
    nav->heading = (nav->heading*180)/0x8000;

    nav->vN = avl::from_bytes<int16_t>(avl::subvector(payload, 27, 2), LSB);
    nav->vN /= 1000;
    nav->vE = avl::from_bytes<int16_t>(avl::subvector(payload, 29, 2), LSB);
    nav->vE /= 1000;
    nav->vDwn = avl::from_bytes<int16_t>(avl::subvector(payload, 31, 2), LSB);
    nav->vDwn /= 1000;

    //Convert from NED to vehicle frame
    NED_TO_VEHICLE(nav);

    //Calculate speed and course in horizontal place
    SPEED_AND_COURSE(nav);

    nav->wFwd = avl::from_bytes<int16_t>(avl::subvector(payload, 33, 2), LSB);
    nav->wFwd /= 100;
    nav->wStbd = avl::from_bytes<int16_t>(avl::subvector(payload, 35, 2), LSB);
    nav->wStbd /= 100;
    nav->wDwn = avl::from_bytes<int16_t>(avl::subvector(payload, 37, 2), LSB);
    nav->wDwn /= 100;
    nav->aFwd = avl::from_bytes<int16_t>(avl::subvector(payload, 39, 2), LSB);
    nav->aFwd /= 1000;
    nav->aStbd = avl::from_bytes<int16_t>(avl::subvector(payload, 41, 2), LSB);
    nav->aStbd /= 1000;
    nav->aDwn = avl::from_bytes<int16_t>(avl::subvector(payload, 43, 2), LSB);
    nav->aDwn /= 1000;

    nav-> posMajor = avl::from_bytes<float>(avl::subvector(payload, 45, 4), LSB);
    nav-> posMinor = avl::from_bytes<float>(avl::subvector(payload, 49, 4), LSB);
    nav-> dirPMajor = avl::from_bytes<float>(avl::subvector(payload, 53, 4), LSB);
    nav-> stdDepth = avl::from_bytes<float>(avl::subvector(payload, 57, 4), LSB);
    nav-> stdLevN = avl::from_bytes<float>(avl::subvector(payload, 61, 4), LSB);
    nav-> stdLevE = avl::from_bytes<float>(avl::subvector(payload, 65, 4), LSB);
    nav-> stdHeading = avl::from_bytes<float>(avl::subvector(payload, 69, 4), LSB);
    nav-> velMajor = avl::from_bytes<float>(avl::subvector(payload, 73, 4), LSB);
    nav-> velMinor = avl::from_bytes<float>(avl::subvector(payload, 77, 4), LSB);
    nav-> dirVMajor = avl::from_bytes<float>(avl::subvector(payload, 81, 4), LSB);
    nav-> velDown = avl::from_bytes<float>(avl::subvector(payload, 85, 4), LSB);

    nav-> status.message.clear();
    uint16_t status = avl::from_bytes<uint16_t>(avl::subvector(payload, 89, 2), LSB);
    nav-> status.status = status;
    nav-> status.orientation = !(status & 0x01);
    if ( !(nav-> status.orientation))
        nav->status.message += "Orientation_invalid ";
    nav-> status.pos = !(status & 0x02);
    if ( !(nav-> status.pos))
        nav->status.message += "Position_invalid ";
    // Altitude status published in cpp file because its only true for one msg
    nav-> status.alt = !(status & 0x04);
    nav-> status.orientation_source = status & 0x10;
    if ( nav-> status.orientation_source)
        nav->status.message += "INS_orientation ";
    else
        nav->status.message += "AHRS_orientation ";
    nav-> status.susbl = !(status & 0x20);
    if ( nav-> status.susbl)
        nav->status.message += "SUSBL_used ";
    nav-> status.depth = !(status & 0x40);
    if ( nav-> status.depth)
        nav->status.message += "depth_used ";
    nav-> status.dvl = !(status & 0x80);
    if ( nav-> status.dvl)
        nav->status.message += "dvl_used ";
    nav-> status.lbl = !(status & 0x0100);
    if ( nav-> status.lbl)
        nav->status.message += "lbl_used ";
    nav-> status.zupt = !(status & 0x0200);
    if ( nav-> status.zupt)
        nav->status.message += "zupt_used ";
    nav-> status.xpos = !(status & 0x0400);
    if ( nav-> status.xpos)
        nav->status.message += "xpos_used ";
    nav-> status.gps = !(status & 0x0800);
    if ( nav-> status.gps)
        nav->status.message += "gps_used ";
    nav-> status.zmd = !(status & 0x1000);
    if ( nav-> status.zmd)
        nav->status.message += "zmd_used ";
    nav-> status.usbl = !(status & 0x2000);
    if ( nav-> status.usbl)
        nav->status.message += "usbl_used ";
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_OBST
// Description: Reads an OBST payload from the SprintNav and updates the sensor
//              OBST structure
// Arguments:   - payload: OBST payload
//              - *obst: current obst structure pointer
// Returns:     - none
//------------------------------------------------------------------------------
void SPRINTNAV_OBST(std::vector<uint8_t> payload, Obst *obst)
{
    obst->time = SPRINTNAV_UINT48_TIME(payload, 1);

    obst->rejection = avl::from_bytes<uint16_t>(avl::subvector(payload, 7, 2),
        LSB);

    obst->mahad = avl::from_bytes<float>(avl::subvector(payload, 9, 4), LSB);

    obst->specific = avl::subvector(payload, 13, payload.size() - 14);
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_PD4
// Description: Reads a PD4 payload from the SprintNav and updates the current
//              DVL status structure
// Arguments:   - payload: DVL payload
//              - *dvl: current dvl status structure pointer
// Returns:     - none
//------------------------------------------------------------------------------
void SPRINTNAV_PD4(std::vector<uint8_t> payload, Dvl *dvl)
{
    constexpr size_t header_n = 12;

    dvl->config = payload[header_n + 5];

    dvl->bit = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 40, 2), LSB);

    dvl->sound_speed = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 42, 2), LSB);

    dvl->shallow_current =  (payload[header_n + 35] & 0x10);


    dvl->current_layer[0] = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 31, 2), LSB);
    dvl->current_layer[0] /= 10;
    dvl->current_layer[1] = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 33, 2), LSB);
    dvl->current_layer[1] /= 10;

    dvl->tofp = (payload[header_n + 36] * 3600) + (payload[header_n + 37] * 60)
        + payload[header_n + 38] + (payload[header_n + 39] / 100);

    dvl->temp = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 44, 2), LSB);
    dvl->temp /= 100;

    dvl->checksum = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 46, 2), LSB);

    //bottom velocity in m/s
    dvl->v.x = avl::from_bytes<int16_t>(avl::subvector(payload, header_n + 6, 2),
        LSB);
    dvl->v.x /=1e3;
    dvl->v.y = avl::from_bytes<int16_t>(avl::subvector(payload, header_n + 8, 2),
        LSB);
    dvl->v.y /=1e3;
    dvl->v.z = avl::from_bytes<int16_t>(
        avl::subvector(payload, header_n + 10, 2), LSB);
    dvl->v.z /=1e3;
    dvl->v.err = avl::from_bytes<int16_t>(
        avl::subvector(payload, header_n + 12, 2), LSB);
    dvl->v.err /=1e3;

    dvl->v.valid = (dvl->v.x > -32.7 ); // invalid velocity if less than ~-32.7

    //Transform to vehicle frame (from PD4 frame, per UM-8275 A.2.1)
    Vector3d pd_transform(180, 0, 135); //180deg roll, 135deg heading
    Vector3d v(dvl->v.x, dvl->v.y, dvl->v.z);
    v = COORD_TRANSFORM(pd_transform, v, true);
    dvl->v.x = v(0);
    dvl->v.y = v(1);
    dvl->v.z = v(2);


    //bottom range in m
    dvl->beam1.range = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 14, 2), LSB);
    dvl->beam1.range /=1e2;
    dvl->beam2.range = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 16, 2), LSB);
    dvl->beam2.range /=1e2;
    dvl->beam3.range = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 18, 2), LSB);
    dvl->beam3.range /=1e2;
    dvl->beam4.range = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 20, 2), LSB);
    dvl->beam4.range /=1e2;

    //ref layer (current) velocity in m/s
    dvl->current.x = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 23, 2), LSB);
    dvl->current.x /=1e3;
    dvl->current.y = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 25, 2), LSB);
    dvl->current.y /=1e3;
    dvl->current.z = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 27, 2), LSB);
    dvl->current.z /=1e3;
    dvl->current.err = avl::from_bytes<int16_t>(avl::subvector(payload,
        header_n + 29, 2), LSB);
    dvl->current.err /=1e3;

    //invalid velocity if less than ~-32.7
    dvl->current.valid = (dvl->current.x > -32.7 );

    //beam flags
    dvl->beam1.correlation = !(payload[header_n + 22] & 0x01);
    dvl->beam1.echo = !(payload[header_n + 22] & 0x02);
    dvl->beam1.current_correlation = !(payload[header_n + 35] & 0x01);
    dvl->beam2.correlation = !(payload[header_n + 22] & 0x04);
    dvl->beam2.echo = !(payload[header_n + 22] & 0x08);
    dvl->beam2.current_correlation = !(payload[header_n + 35] & 0x02);
    dvl->beam3.correlation = !(payload[header_n + 22] & 0x10);
    dvl->beam3.echo = !(payload[header_n + 22] & 0x20);
    dvl->beam3.current_correlation = !(payload[header_n + 35] & 0x04);
    dvl->beam4.correlation = !(payload[header_n + 22] & 0x40);
    dvl->beam4.echo = !(payload[header_n + 22] & 0x80);
    dvl->beam4.current_correlation = !(payload[header_n + 35] & 0x08);
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_TMS
// Description: Reads a Time System Data payload from the SprintNav and updates
//                the sensor TMS structure
// Arguments:   - payload: TMS payload
//              - *status: current status structure pointer
// Returns:     - none
//------------------------------------------------------------------------------
void SPRINTNAV_TMS(std::vector<uint8_t> payload, Status *status)
{
    status->tms.sys_time = SPRINTNAV_UINT48_TIME(payload, 1);
    status->tms.utc = avl::from_bytes<uint64_t>(avl::subvector(payload, 7, 8),
        LSB);
    status->tms.utc /=1e6;
    status->tms.time_since_update = SPRINTNAV_UINT48_TIME(payload, 15);

    status->tms.sd= avl::from_bytes<float>(avl::subvector(payload, 21, 4),
        LSB);

    status->tms.source = payload[25];
    status->tms.zda_count = payload[27];
    status->tms.pps_count = payload[28];
    status->tms.zda_rej_count = payload[29];
    status->tms.pps_rej_count = payload[30];
    status->tms.zda_pps_proc_count = payload[31];
    status->tms.filter_reset_count = payload[32];

    status->tms.pps_rising = static_cast<bool>(payload[26]);
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_PWRSTAT
// Description: Reads a PWRSTAT payload from the SprintNav and updates
//                the PWRSTAT structure
// Arguments:   - payload: PWRSTAT payload
//              - *pwrstat: current pwrstat structure pointer
// Returns:     - none
//------------------------------------------------------------------------------
void SPRINTNAV_PWRSTAT (std::vector<uint8_t> payload, Pwrstat *pwrstat)
{
    std::string line = std::string(payload.begin(), payload.end());

    std::string delimiter;
    delimiter = '\n';
    std::vector<std::string> split_line = avl::split(line, delimiter);

    for (unsigned int i = 0; i < split_line.size(); i++)
    {
        int pos = split_line[i].find("=");
        split_line[i] = split_line[i].substr(pos + 1);
    }

    pwrstat->ext_pwr = std::stod(split_line[0]);
    pwrstat->fifteen_v = std::stod(split_line[1]);
    pwrstat->fifteen_v_i = std::stod(split_line[2])/1e3;
    pwrstat->five_v = std::stod(split_line[3]);
    pwrstat->five_v_i = std::stod(split_line[4])/1e3;
    pwrstat->neg_fifteen_v = std::stod(split_line[5]);
    pwrstat->battery_v = std::stod(split_line[6]);
    pwrstat->battery_temp = std::stod(split_line[7])-273.15;
    pwrstat->status = split_line[8];
    pwrstat->pic_charger_stat = split_line[9];

}


//------------------------------------------------------------------------------
// Name:        SPRINTNAV_BIST
// Description: Reads a BIST payload from the SprintNav and updates the current
//              BIST status structure
// Arguments:   - payload: BIST payload
//              - *bist: current bist status structure pointer
// Returns:     - none
//------------------------------------------------------------------------------
void SPRINTNAV_BIST(std::vector<uint8_t> payload, Status *status)
{
    status->bist.sys_time = SPRINTNAV_UINT48_TIME(payload, 1);

    uint16_t fw[4];
    for (size_t i = 0; i < 4; i++)
        fw[i] = avl::from_bytes<uint16_t>(avl::subvector(payload, 7 + 2*i, 2), LSB);
    status->bist.fw = std::to_string(fw[3]) + '_' + std::to_string(fw[2]) + '_'
        + std::to_string(fw[1]) + '_' + std::to_string(fw[0]);

    status->bist.imu = avl::from_bytes<uint64_t>
        (avl::subvector(payload, 15, 8), LSB);
    status->bist.comms = avl::from_bytes<uint64_t>
        (avl::subvector(payload, 23, 8), LSB);
    status->bist.cca = avl::from_bytes<uint64_t>
        (avl::subvector(payload, 31, 8), LSB);
    status->bist.ains = avl::from_bytes<uint64_t>
        (avl::subvector(payload, 41, 8), LSB);

    status->bist.ahrs = avl::from_bytes<uint16_t>
        (avl::subvector(payload, 39, 2), LSB);
}

//------------------------------------------------------------------------------
// Name:        SPRINTNAV_COMMAND
// Description: Issue a multiplexed command string to the SprintNav
// Arguments:   - Command string to be sent to SprintNav
// Returns:     - multiplexed command string to SprintNav
//------------------------------------------------------------------------------
std::vector<uint8_t> SPRINTNAV_COMMAND(std::string message)
{
    std::vector<uint8_t> packet( message.begin(), message.end() );

    return ( SPRINTNAV_MULTIPLEX( packet, 0) );
}
#endif // SPRINT_NAV_H
