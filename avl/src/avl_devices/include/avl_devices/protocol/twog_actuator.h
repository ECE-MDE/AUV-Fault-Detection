//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides functions to configure and run the 2G Actuator
//
//==============================================================================

#ifndef TWOG_ACT_H
#define TWOG_ACT_H

//==============================================================================
//                            GLOBAL CONSTANT DEFINITIONS
//==============================================================================

constexpr size_t twog_query_count{1};
constexpr size_t twog_act_count{4};
std::vector<uint8_t> SAVE_EE =  {0x24};
std::vector<uint8_t> REQ_AUTO = {0xB3};
std::vector<uint8_t> REQ_FS =   {0x93};
std::vector<uint8_t> REQ_SAMP = {0x65};
std::vector<uint8_t> REQ_CFG1 = {0x6D};
std::vector<uint8_t> REQ_CFG2 = {0x64};
std::vector<uint8_t> REQ_CFG3 = {0xA7};
std::vector<uint8_t> REQ_GS =   {0xAC};
std::vector<uint8_t> REQ_STD =  {0xA9};
std::vector<uint8_t> REQ_MP =   {0x9B};
uint8_t start_delim{ 0x5b }; //character to start a packet to twog actuator
uint8_t stop_delim{ 0x5d }; //character to end a packet to twog actuator
int8_t flipped = -1; //twog axis of rotation is inverted from avl

//==============================================================================
//                            CRC TABLE
//==============================================================================

const uint8_t CRC8_TABLE [256] = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,
    0x36,0x31,0x24,0x23,0x2A,0x2D,0x70,0x77,0x7E,0x79,
    0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,
    0x5A,0x5D,0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,
    0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,0x90,0x97,
    0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,
    0xB4,0xB3,0xBA,0xBD,0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,
    0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,
    0x81,0x86,0x93,0x94,0x9D,0x9A,0x27,0x20,0x29,0x2E,
    0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,
    0x0D,0x0A,0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,
    0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,0x89,0x8E,
    0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,
    0xAD,0xAA,0xA3,0xA4,0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,
    0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,
    0x5F,0x58,0x4D,0x4A,0x43,0x44,0x19,0x1E,0x17,0x10,
    0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,
    0x33,0x34,0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,
    0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,0x3E,0x39,
    0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,
    0x1A,0x1D,0x14,0x13,0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,
    0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,
    0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
    };

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

typedef struct Act
{
    size_t address;  //keep address first, this value gets initialized
    size_t location; //should match address, if not error

    bool enabled;   //motor on status

    uint8_t model_id;

    int8_t temp1;
    int8_t temp2;

    int32_t motor_match;

    float voltage;
    float current;
    float vel; //shaft velocity in deg/s

    struct Pos
    {
        double measured;
        double command;
        double max;     //max controlller angle
        double abs_max; //max user angle
        double home;
    } pos; //xform position in degrees

    struct Fault
    {
        uint8_t motor;
        uint8_t pos;
        uint8_t temp;
        uint8_t comms;
    } fault;

    std::string fw;
    std::string sn;
}Act;

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        TWOG_CHECKSUM
// Description: Calculation of a packet checksum.
// Arguments:   - packet: all bytes between the start and end delimiter
// Returns:     Corresponding checksum in hexidecimal.
//------------------------------------------------------------------------------
uint8_t TWOG_CHECKSUM(std::vector<uint8_t> buffer)
{
    uint8_t crc = 0x00;
    for(size_t i = 0; i < buffer.size(); i++)
        crc = CRC8_TABLE[crc ^ buffer.at(i)];
    return crc;
}

//------------------------------------------------------------------------------
// Name:        TWOG_WRAP
// Description: Wrap a payload message with Twog delimiter and CRC formatting.
// Arguments:   - channel:  which actuator to query/command
// 		- payload: vector of bytes to be written
// Returns:     Complete and formatted twog message
//------------------------------------------------------------------------------
std::vector<uint8_t> TWOG_WRAP(size_t channel, std::vector<uint8_t> payload)
{
    std::vector<uint8_t> message;
    message.push_back(channel);
    message.push_back(payload.size());
    avl::append(message, payload);
    message.push_back(TWOG_CHECKSUM(message));
    message.insert(message.begin(), start_delim);
    message.push_back(stop_delim);

    return message;
}

//------------------------------------------------------------------------------
// Name:        TWOG_ENABLE
// Description: Creates a motor enable/disable message in twog
//              command format.
// Arguments:   - enable: true to enable, false to disable
//------------------------------------------------------------------------------
std::vector<uint8_t> TWOG_ENABLE(size_t channel, bool enable)
{
    std::vector<uint8_t> payload;
    uint8_t type = 0x58; // motor control message ID
    payload.push_back(type);
    if (enable)
        payload.push_back(0x01);
    else
        payload.push_back(0x00);

    return TWOG_WRAP(channel, payload);
}

//------------------------------------------------------------------------------
// Name:        TWOG_SETPOINT
// Description: Creates a setpoint message in twog command format
// Arguments:   - position: degrees (right hand rule, positive axis out)
//              - *fin: current actuator status structure pointer
// Returns:     - formatted message ready for transmission to 2G actuator
//------------------------------------------------------------------------------
std::vector<uint8_t> TWOG_SETPOINT(Act *fin)
{
    uint8_t type = 0x53; // absolute setpoint message ID

    // if absolute position is exceeded, set as abs_max (with correct sign)
    if ( abs(fin->pos.command) > fin->pos.abs_max )
        fin->pos.command = fin->pos.command/abs(fin->pos.command) *
            fin->pos.abs_max;

    uint32_t mils = static_cast<uint32_t>(fin->pos.command*1000);
    mils *= flipped;
    //convert 32-bit position into 4 8-bit segments mils#
    int32_t holder = (mils) & 0xFF;
    uint8_t mils0 = holder;
    holder = (mils >> 8) & 0xFF;
    uint8_t mils1 = holder;
    holder = (mils >> 16) & 0xFF;
    uint8_t mils2 = holder;
    holder = (mils >> 24) & 0xFF;
    uint8_t mils3 = holder;

    //set absolute position
    std::vector<uint8_t> payload = { type, mils3 , mils2 , mils1 , mils0 };

    return TWOG_WRAP(fin->address, payload);
}

//------------------------------------------------------------------------------
// Name:        TWOG_QUERY
// Description: Provide query definition for twog based on iterative model.
// Arguments:   - channel:  which actuator to query/command
// 		- query_no: which type of query/packet to request
// Returns:     Payload packet to write
//------------------------------------------------------------------------------
std::vector<uint8_t> TWOG_QUERY(Act *fin, size_t query_no)
{
    std::vector<uint8_t> payload;
    size_t payload_length{};
    uint8_t packet_type{};

    //switch case for query number
    switch (query_no)
    {
        case 0:
            return TWOG_SETPOINT(fin);
            break;
        case 1:
            packet_type = 0x70; //system info
            payload_length = 24;
            break;
        case 2:
            packet_type = 0xA3; //system info 2
            payload_length = 26;
            break;
        case 3:
            packet_type = 0x68; //velocity
            payload_length = 9;
            break;
        case 4:
            packet_type = 0x66; //fault info
            payload_length = 5;
            break;
    }

    //insert payload length and type bytes
    payload.push_back(packet_type);

    //append zeros for remainder of payload
    for (size_t i=1; i < payload_length; i++ )
        payload.push_back(0x00);


    return TWOG_WRAP(fin->address, payload);
}

//------------------------------------------------------------------------------
// Name:        TWOG_FW
// Description: Reads a firmware payload from a twog actuator and updates
//              the current actuator status structure
// Arguments:   - payload: twog payload
//              - *fin: current actuator status structure pointer
//------------------------------------------------------------------------------
void TWOG_FW(std::vector<uint8_t> payload, Act *fin)
{
    std::string major = std::to_string( avl::from_bytes<uint16_t>(
        avl::subvector(payload, 1, 2) ) );
    std::string minor = std::to_string( avl::from_bytes<uint16_t>(
        avl::subvector(payload, 3, 2) ) );
    fin->fw = major + '.' + minor;
}

//------------------------------------------------------------------------------
// Name:        TWOG_NAME
// Description: Reads a name payload from a twog actuator and updates
//              the current actuator status structure
// Arguments:   - payload: twog payload
//              - *fin: current actuator status structure pointer
//------------------------------------------------------------------------------
void TWOG_NAME(std::vector<uint8_t> payload, Act *fin)
{
    for (size_t i = 1; i < payload.size(); i++)
        fin->sn += payload[i];
}

//------------------------------------------------------------------------------
// Name:        TWOG_SYS_STAT
// Description: Reads a System Status payload from a twog actuator and updates
//              the current actuator status structure
// Arguments:   - payload: twog payload
//              - *fin: current actuator status structure pointer
//------------------------------------------------------------------------------
void TWOG_SYS_STAT(std::vector<uint8_t> payload, Act *fin)
{
    fin->enabled = (payload[1] & 0x03);
    fin->temp1 = payload[15];
    fin->temp2 = payload[16];
    fin->voltage = avl::from_bytes<int32_t>( avl::subvector(payload, 17, 4) );
    fin->voltage = fin->voltage/1e3;
    fin->current = avl::from_bytes<int16_t>( avl::subvector(payload, 21, 2) );
    fin->current = fin->current/1e3;
    fin->pos.measured = avl::from_bytes<int32_t>( avl::subvector(payload, 3, 4) );
    fin->pos.measured = fin->pos.measured/1e3;
    if ( fin->pos.measured > 180.0 )
        fin->pos.measured -= 360.0;
    fin->pos.measured *= flipped;
}

//------------------------------------------------------------------------------
// Name:        TWOG_SYS_STAT2
// Description: Reads a System Status 2 payload from a twog actuator and updates
//              the current actuator status structure
// Arguments:   - payload: twog payload
//              - *fin: current actuator status structure pointer
//------------------------------------------------------------------------------
void TWOG_SYS_STAT2(std::vector<uint8_t> payload, Act *fin)
{
    // These are only two features available as May 2020
    fin->current = avl::from_bytes<int32_t>( avl::subvector(payload, 5, 4) );
    fin->current = fin->current/1e3;
    fin->motor_match = avl::from_bytes<int32_t>( avl::subvector(payload, 21, 4) );
}

//------------------------------------------------------------------------------
// Name:        TWOG_VEL
// Description: Reads a velocity payload from a twog actuator and updates
//              the current actuator status structure
// Arguments:   - payload: twog payload
//              - *fin: current actuator status structure pointer
//------------------------------------------------------------------------------
void TWOG_VEL(std::vector<uint8_t> payload, Act *fin)
{
    fin->vel = avl::from_bytes<int32_t>( avl::subvector(payload, 5, 4) );
    fin->vel = fin->vel/(60*1e3);
    fin->vel *= flipped;
}

//------------------------------------------------------------------------------
// Name:        TWOG_ACK
// Description: Reads an ACK payload from a twog actuator and updates the
//              current actuator status structure
// Arguments:   - payload: twog payload
//              - *fin: current actuator status structure pointer
//------------------------------------------------------------------------------
void TWOG_ACK(std::vector<uint8_t> payload, Act *fin)
{
    fin-> model_id = payload[1];
}

//------------------------------------------------------------------------------
// Name:        TWOG_FAULT
// Description: Reads a fault status payload from a twog actuator and updates
//              the current actuator status structure
// Arguments:   - payload: twog payload
//              - *fin: current actuator status structure pointer
// Returns:     - boolean set to true if a fault is detected
//------------------------------------------------------------------------------
bool TWOG_FAULT(std::vector<uint8_t> payload, Act *fin)
{
    // Initialize with no fault detected
    bool fault = false;

    fin-> fault.motor = payload[1];
    fin-> fault.pos = payload[2];
    fin-> fault.temp = payload[3];
    fin-> fault.comms = payload[4];
    if (payload[1] || payload[2] || payload[3] || payload[4])
        fault = true;

    return fault;
}

//------------------------------------------------------------------------------
// Name:        TWOG_CONFIGURE
// Description: Creates configuration messages for twog actuators
// Arguments:   - *fin: current actuator status structure pointer
// Returns:     - vector of configuration vectors
//------------------------------------------------------------------------------
std::vector<std::vector<uint8_t>> TWOG_CONFIGURE()
{
    std::vector<std::vector<uint8_t>> configure{};

    std::vector<uint8_t> item{};
    // Auto Info Config Packet
    item = { 178, 1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    configure.push_back(item);
    item.clear();

    //  Failsafe Config Packet
    item = { 146, 2, 0, 0, 0, 0, 0, 0, 0, 0 };
    configure.push_back(item);
    item.clear();

    //  Position Sampling Config Packet
    item = { 69, 0, 1, 0, 1, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0 };
    configure.push_back(item);
    item.clear();

    // System Config 1 Packet
    item = { 77, 66, 72, 0, 0, 0, 0, 0, 0, 61, 204, 204, 205, 65, 160, 0, 0, 68, 122, 0, 0, 17, 148, 16, 254, 0, 2, 191, 32, 255, 253, 64, 224, 1 };
    configure.push_back(item);
    item.clear();

    // System Config 2 Packet
    //item = { 68, 59, 68, 155, 166, 0, 0, 0, 0, 65, 32, 0, 0, 65, 32, 0, 0, 69, 156, 64, 0, 0, 2, 50, 128, 0, 21, 249, 0, 0, 0, 0, 0, 0, 54, 238, 128, 0, 0, 0, 199, 0 };
    item = { 68 };
    std::vector<uint8_t> vel_p_gain = (avl::to_bytes<float>(0.002));
    item.insert(item.end(), vel_p_gain.begin(), vel_p_gain.end());
    std::vector<uint8_t> rest ={ 0, 0, 0, 0, 65, 32, 0, 0, 65, 32, 0, 0, 69, 156, 64, 0, 0, 2, 50, 128, 0, 21, 249, 0, 0, 0, 0, 0, 0, 54, 238, 128, 0, 0, 0, 199, 0 };
    item.insert(item.end(), rest.begin(), rest.end());
    configure.push_back(item);
    item.clear();

    // System Config 3 Packet
    item = { 166, 1, 1, 0, 0, 93, 192, 0, 0, 117, 47, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    configure.push_back(item);
    item.clear();

    // Gain Scheduling Config Packet
    item = { 171, 0, 0, 0, 63, 128, 0, 0, 63, 128, 0, 0, 63, 128, 0, 0, 63, 128, 0, 0, 0, 0, 0, 0, 0, 0, 3, 232, 0, 0, 3, 232, 0, 0, 3, 232, 0, 0, 3, 232, 0, 0, 0, 0 };
    configure.push_back(item);
    item.clear();

    // Stall Detection Config Packet
    item = { 168, 0, 0, 0, 8, 52, 0, 0, 0, 150, 10, 90, 0, 0, 0, 100, 0, 10, 0, 0 };
    configure.push_back(item);
    item.clear();

    // Motion Profile Config packet
    item = { 154, 0, 0, 0, 0, 78, 32, 0, 0, 78, 32, 0, 0, 78, 32, 0, 0, 78, 32, 0, 0, 0, 200, 0, 0, 1, 144, 0, 0, 3, 232, 65, 112, 0, 0, 0, 0, 1, 244, 0, 0, 31, 64, 0, 0, 0, 0 };
    configure.push_back(item);
    item.clear();


    return configure;
}

#endif // TWOG_ACT_H
