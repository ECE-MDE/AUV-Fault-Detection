//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a HEARTBEAT packet conforming to the AVL binary
//              packet protocol, providing functions to construct them and get
//              the information described by their fields.
//==============================================================================

#include "protocol/heartbeat_packet.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        HeartbeatPacket constructor
// Description: Constructs a HEARTBEAT packet with the given header data
//              and heartbeat data.
// Arguments:   - header: Packet header data.
//              - heartbeat: Heartbeat information.
//------------------------------------------------------------------------------
HeartbeatPacket::HeartbeatPacket(PacketHeader header, Heartbeat heartbeat) :
    Packet(header, HEARTBEAT_PACKET)
{

    add_field(HEARTBEAT_STATUS,             {static_cast<uint8_t>(heartbeat.status)});
    add_field(HEARTBEAT_WX,                 avl::to_bytes(heartbeat.wx));
    add_field(HEARTBEAT_WY,                 avl::to_bytes(heartbeat.wy));
    add_field(HEARTBEAT_WZ,                 avl::to_bytes(heartbeat.wz));
    add_field(HEARTBEAT_AX,                 avl::to_bytes(heartbeat.ax));
    add_field(HEARTBEAT_AY,                 avl::to_bytes(heartbeat.ay));
    add_field(HEARTBEAT_AZ,                 avl::to_bytes(heartbeat.az));
    add_field(HEARTBEAT_ROLL,               avl::to_bytes(heartbeat.roll));
    add_field(HEARTBEAT_PITCH,              avl::to_bytes(heartbeat.pitch));
    add_field(HEARTBEAT_YAW,                avl::to_bytes(heartbeat.yaw));
    add_field(HEARTBEAT_VN,                 avl::to_bytes(heartbeat.vn));
    add_field(HEARTBEAT_VE,                 avl::to_bytes(heartbeat.ve));
    add_field(HEARTBEAT_VD,                 avl::to_bytes(heartbeat.vd));
    add_field(HEARTBEAT_DEPTH,              avl::to_bytes(heartbeat.depth));
    add_field(HEARTBEAT_HEIGHT,             avl::to_bytes(heartbeat.height));
    add_field(HEARTBEAT_RPM,                avl::to_bytes(heartbeat.rpm));
    add_field(HEARTBEAT_VOLTAGE,            avl::to_bytes(heartbeat.voltage));
    add_field(HEARTBEAT_UMODEM_SYNC,        {static_cast<uint8_t>(heartbeat.umodem_sync)});
    add_field(HEARTBEAT_IRIDIUM_STR,        {heartbeat.iridium_str});

    add_field(HEARTBEAT_MISSION_MODE,       {static_cast<uint8_t>(heartbeat.mission_mode)});
    add_field(HEARTBEAT_FSD_MISSION_STATE,  {static_cast<uint8_t>(heartbeat.fsd_mission_state)});
    add_field(HEARTBEAT_FSD_CURRENT_ACTION, {heartbeat.fsd_current_action});
    add_field(HEARTBEAT_FSD_TOTAL_ACTIONS,  {heartbeat.fsd_total_actions});
    add_field(HEARTBEAT_FSD_ACTION_PERCENT, avl::to_bytes(heartbeat.fsd_action_percent));
    add_field(HEARTBEAT_BSD_MISSION_STATE,  {static_cast<uint8_t>(heartbeat.bsd_mission_state)});
    add_field(HEARTBEAT_BSD_CURRENT_ACTION, {heartbeat.bsd_current_action});
    add_field(HEARTBEAT_BSD_TOTAL_ACTIONS,  {heartbeat.bsd_total_actions});
    add_field(HEARTBEAT_BSD_ACTION_PERCENT, avl::to_bytes(heartbeat.bsd_action_percent));

    add_field(HEARTBEAT_GPS_SATS,           {heartbeat.gps_sats});
    add_field(HEARTBEAT_GPS_LAT,            avl::to_bytes(heartbeat.gps_lat));
    add_field(HEARTBEAT_GPS_LON,            avl::to_bytes(heartbeat.gps_lon));
    add_field(HEARTBEAT_GPS_ALT,            avl::to_bytes(heartbeat.gps_alt));

    add_field(HEARTBEAT_NAV_INITIALIZED,    avl::to_bytes(heartbeat.nav_initialized));
    add_field(HEARTBEAT_NAV_LAT,            avl::to_bytes(heartbeat.nav_lat));
    add_field(HEARTBEAT_NAV_LON,            avl::to_bytes(heartbeat.nav_lon));
    add_field(HEARTBEAT_NAV_ALT,            avl::to_bytes(heartbeat.nav_alt));
    add_field(HEARTBEAT_NAV_YAW_STD,        avl::to_bytes(heartbeat.nav_yaw_std));
    add_field(HEARTBEAT_NAV_AVG_POS_ERR,    avl::to_bytes(heartbeat.nav_avg_pos_err));

}

//------------------------------------------------------------------------------
// Name:        HeartbeatPacket constructor
// Description: Constructs an HEARTBEAT packet from a base packet class.
// Arguments:   - packet: Packet to construct the HEARTBEAT packet from.
//------------------------------------------------------------------------------
HeartbeatPacket::HeartbeatPacket(Packet packet) : Packet(packet.get_bytes())
{
    if (packet.get_descriptor() != HEARTBEAT_PACKET)
        throw std::runtime_error("packet is not a HEARTBEAT packet");
}

//------------------------------------------------------------------------------
// Name:        HeartbeatPacket constructor
// Description: Constructs the packet from a vector of bytes. The vector
//              should contain all packet bytes including the packet
//              start-of-frame and the checksum bytes. Throws a
//              std::runtime_error if the bytes do not form a valid packet.
// Arguments:   - bytes: vector of packet bytes
//------------------------------------------------------------------------------
HeartbeatPacket::HeartbeatPacket(std::vector<uint8_t> bytes) : Packet(bytes)
{

    // Ensure that the packet descriptor is correct and that the packet has
    // all of the required fields

    if (descriptor != HEARTBEAT_PACKET)
        throw std::runtime_error("incorrect packet descriptor for a "
            "HEARTBEAT packet");

}

//------------------------------------------------------------------------------
// Name:        get_heartbeat
// Description: Gets the HEARTBEAT packet's field data as a heartbeat
//              struct.
// Returns:     Heartbeat struct containing the packet's field data.
//------------------------------------------------------------------------------
Heartbeat HeartbeatPacket::get_heartbeat()
{

    Heartbeat heartbeat;

    if(has_field(HEARTBEAT_STATUS))
        heartbeat.status = static_cast<SafetyStatus>(
            avl::from_bytes<uint8_t>(
                get_field(HEARTBEAT_STATUS).get_data()));

    if(has_field(HEARTBEAT_WX))
        heartbeat.wx = avl::from_bytes<double>(
            get_field(HEARTBEAT_WX).get_data());

    if(has_field(HEARTBEAT_WY))
        heartbeat.wy = avl::from_bytes<double>(
            get_field(HEARTBEAT_WY).get_data());

    if(has_field(HEARTBEAT_WZ))
        heartbeat.wz = avl::from_bytes<double>(
            get_field(HEARTBEAT_WZ).get_data());

    if(has_field(HEARTBEAT_AX))
        heartbeat.ax = avl::from_bytes<double>(
            get_field(HEARTBEAT_AX).get_data());

    if(has_field(HEARTBEAT_AY))
        heartbeat.ay = avl::from_bytes<double>(
            get_field(HEARTBEAT_AY).get_data());

    if(has_field(HEARTBEAT_AZ))
        heartbeat.az = avl::from_bytes<double>(
            get_field(HEARTBEAT_AZ).get_data());

    if(has_field(HEARTBEAT_ROLL))
        heartbeat.roll = avl::from_bytes<double>(
            get_field(HEARTBEAT_ROLL).get_data());

    if(has_field(HEARTBEAT_PITCH))
        heartbeat.pitch = avl::from_bytes<double>(
            get_field(HEARTBEAT_PITCH).get_data());

    if(has_field(HEARTBEAT_YAW))
        heartbeat.yaw = avl::from_bytes<double>(
            get_field(HEARTBEAT_YAW).get_data());

    if(has_field(HEARTBEAT_VN))
        heartbeat.vn = avl::from_bytes<double>(
            get_field(HEARTBEAT_VN).get_data());

    if(has_field(HEARTBEAT_VE))
        heartbeat.ve = avl::from_bytes<double>(
            get_field(HEARTBEAT_VE).get_data());

    if(has_field(HEARTBEAT_VD))
        heartbeat.vd = avl::from_bytes<double>(
            get_field(HEARTBEAT_VD).get_data());

    if(has_field(HEARTBEAT_DEPTH))
        heartbeat.depth = avl::from_bytes<double>(
            get_field(HEARTBEAT_DEPTH).get_data());

    if(has_field(HEARTBEAT_HEIGHT))
        heartbeat.height = avl::from_bytes<double>(
            get_field(HEARTBEAT_HEIGHT).get_data());

    if(has_field(HEARTBEAT_RPM))
        heartbeat.rpm = avl::from_bytes<double>(
            get_field(HEARTBEAT_RPM).get_data());

    if(has_field(HEARTBEAT_VOLTAGE))
        heartbeat.voltage = avl::from_bytes<double>(
            get_field(HEARTBEAT_VOLTAGE).get_data());

    if(has_field(HEARTBEAT_UMODEM_SYNC))
        heartbeat.umodem_sync = static_cast<Sync>(
            avl::from_bytes<uint8_t>(
                get_field(HEARTBEAT_UMODEM_SYNC).get_data()));

    if(has_field(HEARTBEAT_IRIDIUM_STR))
        heartbeat.iridium_str = avl::from_bytes<uint8_t>(
            get_field(HEARTBEAT_IRIDIUM_STR).get_data());



    if(has_field(HEARTBEAT_MISSION_MODE))
        heartbeat.mission_mode = static_cast<MissionMode>(
            avl::from_bytes<uint8_t>(
                get_field(HEARTBEAT_MISSION_MODE).get_data()));

    if(has_field(HEARTBEAT_FSD_MISSION_STATE))
        heartbeat.fsd_mission_state = static_cast<State>(
            avl::from_bytes<uint8_t>(
                get_field(HEARTBEAT_FSD_MISSION_STATE).get_data()));

    if(has_field(HEARTBEAT_FSD_CURRENT_ACTION))
        heartbeat.fsd_current_action = avl::from_bytes<uint8_t>(
            get_field(HEARTBEAT_FSD_CURRENT_ACTION).get_data());

    if(has_field(HEARTBEAT_FSD_TOTAL_ACTIONS))
        heartbeat.fsd_total_actions = avl::from_bytes<uint8_t>(
            get_field(HEARTBEAT_FSD_TOTAL_ACTIONS).get_data());

    if(has_field(HEARTBEAT_FSD_ACTION_PERCENT))
        heartbeat.fsd_action_percent = avl::from_bytes<double>(
            get_field(HEARTBEAT_FSD_ACTION_PERCENT).get_data());

    if(has_field(HEARTBEAT_BSD_MISSION_STATE))
        heartbeat.bsd_mission_state = static_cast<State>(
            avl::from_bytes<uint8_t>(
                get_field(HEARTBEAT_BSD_MISSION_STATE).get_data()));

    if(has_field(HEARTBEAT_BSD_CURRENT_ACTION))
        heartbeat.bsd_current_action = avl::from_bytes<uint8_t>(
            get_field(HEARTBEAT_BSD_CURRENT_ACTION).get_data());

    if(has_field(HEARTBEAT_BSD_TOTAL_ACTIONS))
        heartbeat.bsd_total_actions = avl::from_bytes<uint8_t>(
            get_field(HEARTBEAT_BSD_TOTAL_ACTIONS).get_data());

    if(has_field(HEARTBEAT_BSD_ACTION_PERCENT))
        heartbeat.bsd_action_percent = avl::from_bytes<double>(
            get_field(HEARTBEAT_BSD_ACTION_PERCENT).get_data());



    if(has_field(HEARTBEAT_GPS_SATS))
        heartbeat.gps_sats = avl::from_bytes<uint8_t>(
            get_field(HEARTBEAT_GPS_SATS).get_data());

    if(has_field(HEARTBEAT_GPS_LAT))
        heartbeat.gps_lat = avl::from_bytes<double>(
            get_field(HEARTBEAT_GPS_LAT).get_data());

    if(has_field(HEARTBEAT_GPS_LON))
        heartbeat.gps_lon = avl::from_bytes<double>(
            get_field(HEARTBEAT_GPS_LON).get_data());

    if(has_field(HEARTBEAT_GPS_ALT))
        heartbeat.gps_alt = avl::from_bytes<double>(
            get_field(HEARTBEAT_GPS_ALT).get_data());



    if(has_field(HEARTBEAT_NAV_INITIALIZED))
        heartbeat.nav_initialized = static_cast<bool>(
            avl::from_bytes<uint8_t>(
                get_field(HEARTBEAT_NAV_INITIALIZED).get_data()));

    if(has_field(HEARTBEAT_NAV_LAT))
        heartbeat.nav_lat = avl::from_bytes<double>(
            get_field(HEARTBEAT_NAV_LAT).get_data());

    if(has_field(HEARTBEAT_NAV_LON))
        heartbeat.nav_lon = avl::from_bytes<double>(
            get_field(HEARTBEAT_NAV_LON).get_data());

    if(has_field(HEARTBEAT_NAV_ALT))
        heartbeat.nav_alt = avl::from_bytes<double>(
            get_field(HEARTBEAT_NAV_ALT).get_data());

    if(has_field(HEARTBEAT_NAV_YAW_STD))
        heartbeat.nav_yaw_std = avl::from_bytes<double>(
            get_field(HEARTBEAT_NAV_YAW_STD).get_data());

    if(has_field(HEARTBEAT_NAV_AVG_POS_ERR))
        heartbeat.nav_avg_pos_err = avl::from_bytes<double>(
            get_field(HEARTBEAT_NAV_AVG_POS_ERR).get_data());

    return heartbeat;

}

}
