//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a HEARTBEAT packet conforming to the AVL binary
//              packet protocol, providing functions to construct them and get
//              the information described by their fields.
//==============================================================================

#ifndef HEARTBEAT_PACKET_H
#define HEARTBEAT_PACKET_H

// Packet base class
#include "packet.h"

// included for std::nan
#include <cmath>

namespace avl
{

//==============================================================================
//                            PACKET FIELD MAPPING
//==============================================================================

// Enum listing a HEARTBEAT packet's fields in order starting at 0x00
enum HeartbeatField
{

    HEARTBEAT_STATUS,
    HEARTBEAT_WX,
    HEARTBEAT_WY,
    HEARTBEAT_WZ,
    HEARTBEAT_AX,
    HEARTBEAT_AY,
    HEARTBEAT_AZ,
    HEARTBEAT_ROLL,
    HEARTBEAT_PITCH,
    HEARTBEAT_YAW,
    HEARTBEAT_VN,
    HEARTBEAT_VE,
    HEARTBEAT_VD,
    HEARTBEAT_DEPTH,
    HEARTBEAT_HEIGHT,
    HEARTBEAT_RPM,
    HEARTBEAT_VOLTAGE,
    HEARTBEAT_UMODEM_SYNC,
    HEARTBEAT_IRIDIUM_STR,

    HEARTBEAT_MISSION_MODE,
    HEARTBEAT_FSD_MISSION_STATE,
    HEARTBEAT_FSD_CURRENT_ACTION,
    HEARTBEAT_FSD_TOTAL_ACTIONS,
    HEARTBEAT_FSD_ACTION_PERCENT,
    HEARTBEAT_BSD_MISSION_STATE,
    HEARTBEAT_BSD_CURRENT_ACTION,
    HEARTBEAT_BSD_TOTAL_ACTIONS,
    HEARTBEAT_BSD_ACTION_PERCENT,

    HEARTBEAT_GPS_SATS,
    HEARTBEAT_GPS_LAT,
    HEARTBEAT_GPS_LON,
    HEARTBEAT_GPS_ALT,

    HEARTBEAT_NAV_INITIALIZED,
    HEARTBEAT_NAV_LAT,
    HEARTBEAT_NAV_LON,
    HEARTBEAT_NAV_ALT,
    HEARTBEAT_NAV_YAW_STD,
    HEARTBEAT_NAV_AVG_POS_ERR

};

//==============================================================================
//                             ENUM DEFINITIONS
//==============================================================================

// Enum listing possible vehicle modes
typedef enum MissionMode
{
    MISSION_MODE_UNKNOWN,
    MISSION_MODE_DISABLED,
    MISSION_MODE_FSD,
    MISSION_MODE_BSD,
    MISSION_MODE_MANUAL,
} MissionMode;

static const char * mode_string[] = { "UNKNOWN", "DISABLED", "FSD", "BSD",
                                      "MANUAL" };
inline const char* mode_to_string(MissionMode mode)
{
    return mode_string[mode];
}

// Enum listing possible vehicle statuses
typedef enum SafetyStatus
{
    SAFETY_STATUS_UNKNOWN,
    SAFETY_STATUS_READY,
    SAFETY_STATUS_FAULT
} SafetyStatus;
static const char * status_string[] = { "UNKNOWN", "READY", "FAULT" };
inline const char* status_to_string(SafetyStatus status)
{
    return status_string[status];
}

// Enum listing possible micromodem synchronization statuses
typedef enum Sync
{
    SYNC_UNKNOWN,
    SYNC_UNSYNCED,
    SYNC_SYNCED
} Sync;
static const char * sync_string[] = { "UNKNOWN", "UNSYNCED", "SYNCED" };
inline const char* sync_to_string(Sync sync)
{
    return sync_string[sync];
}

// Enum listing FSD and BSD mission states
typedef enum State
{
    STATE_ACTIVE,
    STATE_PAUSED,
    STATE_INACTIVE
} State;
static const char * state_string[] = { "ACTIVE", "PAUSED", "INACTIVE" };
inline const char* state_to_string(State state)
{
    return state_string[state];
}

//==============================================================================
//                             STRUCT DEFINITIONS
//==============================================================================

// Struct containing information forming a full heartbeat packet
typedef struct Heartbeat
{

    SafetyStatus status        = SAFETY_STATUS_UNKNOWN;
    double wx                  = std::nan("");
    double wy                  = std::nan("");
    double wz                  = std::nan("");
    double ax                  = std::nan("");
    double ay                  = std::nan("");
    double az                  = std::nan("");
    double roll                = std::nan("");
    double pitch               = std::nan("");
    double yaw                 = std::nan("");
    double vn                  = std::nan("");
    double ve                  = std::nan("");
    double vd                  = std::nan("");
    double depth               = std::nan("");
    double height              = std::nan("");
    double rpm                 = std::nan("");
    double voltage             = std::nan("");
    Sync umodem_sync           = SYNC_UNKNOWN;
    uint8_t iridium_str        = 0;

    MissionMode mission_mode = MISSION_MODE_UNKNOWN;
    State   fsd_mission_state = STATE_INACTIVE;
    uint8_t fsd_current_action = 0;
    uint8_t fsd_total_actions  = 0;
    double  fsd_action_percent = 0.0;
    State   bsd_mission_state = STATE_INACTIVE;
    uint8_t bsd_current_action = 0;
    uint8_t bsd_total_actions  = 0;
    double  bsd_action_percent = 0.0;

    uint8_t gps_sats = 0;
    double gps_lat   = std::nan("");
    double gps_lon   = std::nan("");
    double gps_alt   = std::nan("");

    bool nav_initialized   = false;
    double nav_lat         = std::nan("");
    double nav_lon         = std::nan("");
    double nav_alt         = std::nan("");
    double nav_yaw_std     = std::nan("");
    double nav_avg_pos_err = std::nan("");

} Heartbeat;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class HeartbeatPacket : public Packet
{

public:

    //--------------------------------------------------------------------------
    // Name:        HeartbeatPacket constructor
    // Description: Constructs a HEARTBEAT packet with the given header data
    //              and heartbeat data.
    // Arguments:   - header: Packet header data.
    //              - heartbeat: Heartbeat information.
    //--------------------------------------------------------------------------
    HeartbeatPacket(PacketHeader header, Heartbeat heartbeat);

    //--------------------------------------------------------------------------
    // Name:        HeartbeatPacket constructor
    // Description: Constructs an HEARTBEAT packet from a base packet class.
    // Arguments:   - packet: Packet to construct the HEARTBEAT packet from.
    //--------------------------------------------------------------------------
    HeartbeatPacket(Packet packet);

    //--------------------------------------------------------------------------
    // Name:        HeartbeatPacket constructor
    // Description: Constructs the packet from a vector of bytes. The vector
    //              should contain all packet bytes including the packet
    //              start-of-frame and the checksum bytes. Throws a
    //              std::runtime_error if the bytes do not form a valid packet.
    // Arguments:   - bytes: vector of packet bytes
    //--------------------------------------------------------------------------
    HeartbeatPacket(std::vector<uint8_t> bytes);

    //--------------------------------------------------------------------------
    // Name:        get_heartbeat
    // Description: Gets the HEARTBEAT packet's field data as a heartbeat
    //              struct.
    // Returns:     Heartbeat struct containing the packet's field data.
    //--------------------------------------------------------------------------
    Heartbeat get_heartbeat();

};

}

#endif // HEARTBEAT_PACKET_H
