//==============================================================================
// Autonomous Vehicle Library
//
// Description: The mission manager node manages the operation of missions from
//              the FSD and BSD interfaces and manages switching between mission
//              modes. The node contains one instance of the MissionManager
//              class for each interface and provides a service server and
//              packet handling for control of mission queues and mission modes
//
// Servers:     system/mission_control (avl_msgs/MissionControlSrv)
//
// Clients:     None
//
// Publishers:  system/mission_status (avl_msgs/MissionStatusMsg)
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>

// Packet handler class
#include <avl_comms/packet_handler.h>

// Mission queue class
#include <avl_system/mission_manager.h>

// ROS messages
#include <avl_msgs/MissionModeSrv.h>
#include <avl_msgs/MissionControlSrv.h>
#include <avl_msgs/MissionStatusMsg.h>
#include <avl_msgs/DataTxSrv.h>

//==============================================================================
//                              ENUM DEFINITION
//==============================================================================

// Enum listing mission commands
enum MissionCommand
{
    MISSION_START,
    MISSION_PAUSE,
    MISSION_STOP,
    MISSION_ADVANCE,
    MISSION_CLEAR,
    MISSION_READ
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class MissionManagerNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        MissionManagerNode constructor
    //--------------------------------------------------------------------------
    MissionManagerNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Current and previous mission mode
    MissionMode mode = MISSION_MODE_FSD;
    MissionMode prev_mode = MISSION_MODE_FSD;

    // Timer for switching mission modes
    ros::Timer mode_duration_timer;

    // Mission status publisher
    ros::Publisher mission_status_pub;

    // Service servers for control of mission execution via ROS
    ros::ServiceServer mission_mode_server;
    ros::ServiceServer mission_control_server;

    // Packet handler for handling incoming packets
    PacketHandler packet_handler;

    // Map containing the mission manager for each comms interface
    std::map<CommsInterface, MissionManager*> mission_manager;
    std::map<CommsInterface, MissionStatus> mission_status;

    // Ethernet channel TX client for sending status updates to the BSD
    ros::ServiceClient ethernet_tx_client;

private:

    //--------------------------------------------------------------------------
    // Name:        packet_callback
    // Description: Called when an AVL packet is received by the communication
    //              architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - packet: Received packet.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool packet_callback(CommsChannel channel, CommsInterface interface,
        Packet packet, bool& result, std::vector<uint8_t>& data)
    {

        PacketDescriptor desc = packet.get_descriptor();

        try
        {

            // Handle COMMAND packets, but only if the mission manager is in the
            // mode corresponding to the interface sending the command
            if (desc == COMMAND_PACKET)
            {

                // Parse the command packet
                CommandPacket command_packet(packet);
                Command command = command_packet.get_command();
                ParameterList params = command.parameters;

                // Handle FSD MODE command
                if (command.name == "FSD MODE")
                {
                    log_debug("handling FSD MODE command");
                    double duration = 0;
                    if (params.has("DURATION"))
                        duration = params.get("DURATION").to_double();
                    handle_mode_change(interface, MISSION_MODE_FSD,
                        duration);
                    result = true;
                    return true;
                }

                // Handle BSD MODE command
                if (command.name == "BSD MODE")
                {
                    log_debug("handling BSD MODE command");
                    double duration = 0;
                    if (params.has("DURATION"))
                        duration = params.get("DURATION").to_double();
                    handle_mode_change(interface, MISSION_MODE_BSD,
                        duration);
                    result = true;
                    return true;
                }

                // Handle MANUAL MODE command
                if (command.name == "MANUAL MODE")
                {
                    log_debug("handling MANUAL MODE command");
                    double duration = 0;
                    if (params.has("DURATION"))
                        duration = params.get("DURATION").to_double();
                    handle_mode_change(interface, MISSION_MODE_MANUAL,
                        duration);
                    result = true;
                    return true;
                }

                // Handle MISSION field
                if (command.name == "MISSION")
                {

                    log_debug("handling MISSION command");

                    // Get the command name
                    MissionCommand mission_command = params
                        .get("COMMAND").to_enum<MissionCommand>();

                    // Reject the MISSION command if we're not in the right mode
                    // and it's not a READ command
                    if (!in_mode(interface) && mission_command != MISSION_READ)
                    {
                        std::string message =
                            avl::interface_to_string(interface) +
                            " cannot command mission while in " +
                            avl::mode_to_string(mode) + " mode";
                        data = std::vector<uint8_t>(message.begin(),
                                                    message.end());
                        result = false;
                        return true;
                    }

                    // Event object to notify the BSD
                    Event event;

                    // Handle the various mission commands
                    if (mission_command == MISSION_START)
                    {
                        mission_manager[interface]->start();
                        event.type = EVENT_TYPE_MISSION_STARTED;
                    }
                    else if (mission_command == MISSION_PAUSE)
                    {
                        mission_manager[interface]->pause();
                        event.type = EVENT_TYPE_MISSION_PAUSED;
                    }
                    else if (mission_command == MISSION_STOP)
                    {
                        mission_manager[interface]->stop();
                        event.type = EVENT_TYPE_MISSION_STOPPED;
                    }
                    else if (mission_command == MISSION_ADVANCE)
                    {
                        mission_manager[interface]->advance();
                        event.type = EVENT_TYPE_MISSION_ADVANCED;
                    }
                    else if (mission_command == MISSION_CLEAR)
                    {
                        mission_manager[interface]->clear();
                        event.type = EVENT_TYPE_MISSION_CLEARED;
                    }
                    else if (mission_command == MISSION_READ)
                    {
                        auto actions = mission_manager[interface]->read();
                        for (auto action : actions)
                            avl::append(data, action.get_bytes());
                    }

                    // Notify the BSD of the mission event
                    event.result = true;
                    event_callback(event, INTERFACE_BSD);

                    result = true;
                    return true;

                } // MISSION Field

            } // COMMAND Packet

            // Handle action packets
            if (desc == ACTION_PACKET)
            {

                ActionPacket action_packet(packet);
                Action action = action_packet.get_action();
                std::string action_mode;

                switch (action.mode)
                {
                    case ACTION_MODE_APPEND:
                        mission_manager[interface]->append(packet);
                        action_mode = "appending";
                        break;
                    case ACTION_MODE_SET:
                        mission_manager[interface]->set(packet);
                        action_mode = "setting";
                        break;
                    case ACTION_MODE_EXECUTE:
                        if(in_mode(interface))
                        {
                            mission_manager[interface]->execute(packet);
                            action_mode = "executing";
                        }
                        else
                        {
                            log_warning("Received execute action from %s while "
                                    "in %s mode. Setting action.",
                                    interface_to_string(interface).c_str(),
                                    mode_to_string(mode).c_str());
                            mission_manager[interface]->set(packet);
                            action_mode = "setting";
                        }

                        break;
                }

                log_info("%s ACTION packet with action name %s and "
                    "parameters:", action_mode.c_str(), action.name.c_str());
                for (const Parameter& param : action.parameters)
                    log_info("name: %-12s   type: %d   value: %s",
                        param.name.c_str(),
                        param.type,
                        avl::byte_to_hex(param.value).c_str());

                result = true;
                return true;
            }

        }
        catch (const std::exception& ex)
        {
            std::string message(ex.what());
            data = std::vector<uint8_t>(message.begin(), message.end());
            log_warning("failed to handle COMMAND packet (%s)", ex.what());
            result = false;
            return true;
        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        mission_control_srv_callback
    // Description: Called when the mission_control service is called.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service call completed, false otherwise.
    //--------------------------------------------------------------------------
    bool mission_control_srv_callback(
        avl_msgs::MissionControlSrv::Request &request,
        avl_msgs::MissionControlSrv::Response &response)
    {

        CommsInterface interface = static_cast<CommsInterface>(
            request.interface);

        try
        {

            // Ensure that we are in the correct mode for controlling the
            // mission from the FSD interface
            if (!in_mode(INTERFACE_FSD))
                throw std::runtime_error(interface_to_string(interface) +
                    " interface cannot command missions in " +
                    mode_to_string(mode) + " mode");

            if (request.command == MISSION_START)
                mission_manager[interface]->start();
            else if (request.command == MISSION_PAUSE)
                mission_manager[interface]->pause();
            else if (request.command == MISSION_STOP)
                mission_manager[interface]->stop();
            else if (request.command == MISSION_ADVANCE)
                mission_manager[interface]->advance();
            else if (request.command == MISSION_CLEAR)
                mission_manager[interface]->clear();
            else if (request.command == MISSION_READ)
                log_warning("no read yet");
            response.success = true;
            response.message = "success";

        }
        catch (const std::exception& ex)
        {
            response.success = false;
            response.message = ex.what();
        }

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        mission_mode_srv_callback
    // Description: Called when the mission_mode service is called.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service call completed, false otherwise.
    //--------------------------------------------------------------------------
    bool mission_mode_srv_callback(
        avl_msgs::MissionModeSrv::Request &request,
        avl_msgs::MissionModeSrv::Response &response)
    {

        MissionMode mode = static_cast<MissionMode>(
            request.mode);

        try
        {
            handle_mode_change(INTERFACE_FSD, mode, request.duration);
            response.success = true;
            response.message = "success";
        }
        catch (const std::exception& ex)
        {
            response.success = false;
            response.message = ex.what();
        }

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        mode_duration_timer_callback
    // Description: Called when the mode duration timer expires.
    // Argument:    - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void mode_duration_timer_callback(const ros::TimerEvent& event)
    {

        log_debug("mode_duration_timer_callback called");
        log_warning("switching from %s mode back to %s mode",
            mode_to_string(mode).c_str(), mode_to_string(prev_mode).c_str());

        // Pause the current mission if in FSD or BSD mode
        if ((mode == MISSION_MODE_FSD || mode == MISSION_MODE_BSD) &&
            mission_manager[mode_to_interface(mode)]->is_active())
            mission_manager[mode_to_interface(mode)]->pause();

        // Switch back to the previous mode
        mode = prev_mode;

        // If the mode had a mission running when we switched out of it, start
        // the mission again
        if ((mode == MISSION_MODE_FSD || mode == MISSION_MODE_BSD) &&
            mission_manager[mode_to_interface(mode)]->is_paused())
        {
            log_warning("previous mode was paused, resuming");
            mission_manager[mode_to_interface(mode)]->start();
        }

        // Notify the BSD of mission mode changes
        Event mode_event;
        mode_event.type = EVENT_TYPE_MISSION_MODE_CHANGED;
        mode_event.result = true;
        mode_event.data = {static_cast<uint8_t>(mode)};
        event_callback(mode_event, INTERFACE_BSD);

        publish_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        status_changed_callback
    // Description: Called when a mission manager status changes.
    // Arguments:   - status: New mission status.
    //              - interface: Interface that the mission manager belongs to.
    //--------------------------------------------------------------------------
    void status_changed_callback(MissionStatus status, CommsInterface interface)
    {
        mission_status[interface] = status;
        publish_mission_status();
    }

    //--------------------------------------------------------------------------
    // Name:        handle_mode_change
    // Description: Handles changing mission modes indefinitely or for a
    //              specified period.
    // Arguments:   - interface: Interface calling the command.
    //              - new_mode: Mode to switch into.
    //              - duration: New mode duration in seconds.
    //--------------------------------------------------------------------------
    void handle_mode_change(CommsInterface interface, MissionMode new_mode,
        double duration)
    {

        log_debug("handle_mode_change called (%s to %s)",
            mode_to_string(mode).c_str(),
            mode_to_string(new_mode).c_str());

        // Don't handle a mode change if we are already in the new mode
        if (new_mode == mode)
            throw std::runtime_error("already in " +
                mode_to_string(mode) + " mode");

        // Pause the current mission if in FSD or BSD mode
        if (mode == MISSION_MODE_FSD || mode == MISSION_MODE_BSD)
            mission_manager[mode_to_interface(mode)]->pause();

        // Save the mode before switching so that we can switch
        // back when the duration expires
        prev_mode = mode;

        // If duration is zero, the mode is switched into indefinitely
        if (duration == 0)
        {
            log_warning("switching from %s mode to %s mode indefinitely",
                mode_to_string(mode).c_str(),
                mode_to_string(new_mode).c_str());
            mode_duration_timer.stop();
            mode = new_mode;
        }

        // If there is a duration, the mode is switched into for the
        // specified duration and then switched back afterwards
        else
        {
            log_warning("switching from %s mode to %s mode for %f seconds",
                mode_to_string(mode).c_str(),
                mode_to_string(new_mode).c_str(),
                duration);
            mode = new_mode;
            mode_duration_timer.setPeriod(ros::Duration(duration));
            mode_duration_timer.start();
        }

        // If the mode switches from BSD to FSD, resume mission
        if ((prev_mode == MISSION_MODE_BSD && mode == MISSION_MODE_FSD) &&
            mission_manager[mode_to_interface(mode)]->is_paused())
        {
            log_warning("Switching from BSD to FSD, resuming FSD mission");
            mission_manager[mode_to_interface(mode)]->start();
        }

        // If the mode we're switching into is the DISABLED mode, stop and clear
        // the mission queues
        if (new_mode == MISSION_MODE_DISABLED)
        {
            mission_manager[INTERFACE_FSD]->stop();
            mission_manager[INTERFACE_FSD]->clear();
            mission_manager[INTERFACE_BSD]->stop();
            mission_manager[INTERFACE_BSD]->clear();
        }

        // Notify the BSD of mission mode changes
        Event event;
        event.type = EVENT_TYPE_MISSION_MODE_CHANGED;
        event.result = true;
        event.data = {static_cast<uint8_t>(mode)};
        event_callback(event, INTERFACE_BSD);

        publish_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        publish_mission_status
    // Description: Publishes the latest mission status.
    //--------------------------------------------------------------------------
    void publish_mission_status()
    {
        MissionStatus fsd_status = mission_status[INTERFACE_FSD];
        MissionStatus bsd_status = mission_status[INTERFACE_BSD];
        avl_msgs::MissionStatusMsg msg;
        msg.mode = mode;
        msg.fsd_mission_state =  fsd_status.state;
        msg.fsd_current_action = fsd_status.current_action;
        msg.fsd_total_actions =  fsd_status.total_actions;
        msg.fsd_action_percent = fsd_status.action_percent;
        msg.bsd_mission_state =  bsd_status.state;
        msg.bsd_current_action = bsd_status.current_action;
        msg.bsd_total_actions =  bsd_status.total_actions;
        msg.bsd_action_percent = bsd_status.action_percent;
        mission_status_pub.publish(msg);

/*        log_debug("----------------------------------------------------------");
        log_debug("Mission status");
        log_debug("mode:               %s", mode_to_string(mode).c_str());
        log_debug("fsd_mission_state:  %s",   MissionManager::state_to_string(
            fsd_status.state).c_str());
        log_debug("fsd_current_action: %d",   fsd_status.current_action);
        log_debug("fsd_total_actions:  %d",   fsd_status.total_actions);
        log_debug("fsd_action_percent: %.2f", fsd_status.action_percent);
        log_debug("bsd_mission_state:  %s",   MissionManager::state_to_string(
            bsd_status.state).c_str());
        log_debug("bsd_current_action: %d",   bsd_status.current_action);
        log_debug("bsd_total_actions:  %d",   bsd_status.total_actions);
        log_debug("bsd_action_percent: %.2f", bsd_status.action_percent);
        log_debug("----------------------------------------------------------");
*/
        log_data("[MODE] %u", mode);
        log_data("[FSD] %u %d %d %.2f", fsd_status.state,
            fsd_status.current_action, fsd_status.total_actions,
            fsd_status.action_percent);
        log_data("[BSD] %u %d %d %.2f", bsd_status.state,
            bsd_status.current_action, bsd_status.total_actions,
            bsd_status.action_percent);
    }

    //--------------------------------------------------------------------------
    // Name:        in_mode
    // Description: Checks that the mission manager is in a mode that can be
    //              controlled by the given interface.
    // Arguments:   - interface: Interface to compare to mission mode.
    // Returns:     True if the current mission mode can be controlled by the
    //              given interface. False otherwise.
    //--------------------------------------------------------------------------
    bool in_mode(CommsInterface interface)
    {
        return (interface == INTERFACE_FSD && mode == MISSION_MODE_FSD) ||
               (interface == INTERFACE_BSD && mode == MISSION_MODE_BSD);
    }

    //--------------------------------------------------------------------------
    // Name:        mode_to_string
    // Description: Converts a mission mode enum to a string for printing.
    // Arguments:   - mode: Mode to convert to a string.
    // Returns:     String representing the mission mode.
    //--------------------------------------------------------------------------
    std::string mode_to_string(MissionMode mode)
    {
        switch (mode)
        {
            case MISSION_MODE_UNKNOWN:
                return "UNKNOWN";
            case MISSION_MODE_MANUAL:
                return "MANUAL";
            case MISSION_MODE_FSD:
                return "FSD";
            case MISSION_MODE_BSD:
                return "BSD";
            case MISSION_MODE_DISABLED:
                return "DISABLED";
        }
        throw std::runtime_error("mode_to_string: invalid mode");
    }

    //--------------------------------------------------------------------------
    // Name:        mode_to_interface
    // Description: Converts a mission mode enum to the interface enum that
    //              controls the mode.
    // Arguments:   - mode: Mode to convert to interface.
    // Returns:     Interface enum that controlls the given mode.
    //--------------------------------------------------------------------------
    CommsInterface mode_to_interface(MissionMode mode)
    {
        CommsInterface interface;
        if (mode == MISSION_MODE_FSD)
            interface = INTERFACE_FSD;
        else if (mode == MISSION_MODE_BSD)
            interface = INTERFACE_BSD;
        else
            throw std::runtime_error("mode doesn't have corresponding "
                "interface");
        return interface;
    }

    //--------------------------------------------------------------------------
    // Name:        event_callback
    // Description: Updates the BSD with events from the mission manager
    // Arguments:   - event: Event to send
    //--------------------------------------------------------------------------
    void event_callback(Event event, CommsInterface interface)
    {

        // Construct the packet header
        avl::PacketHeader header;
        header.timestamp = avl::get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        // Construct the notification packet
        EventPacket event_packet(header, event);

        // Create the DataTx service message
        DataTxSrv srv;
        srv.request.interface = interface;
        srv.request.data = event_packet.get_bytes();

        // Call the data transmit service
        ethernet_tx_client.call(srv);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log data headers
        add_data_header("[MODE] mode");
        add_data_header("[MODE] none");
        add_data_header("[FSD] state current\\_action total\\_actions"
            " action\\_percent");
        add_data_header("[FSD] none none none pct");
        add_data_header("[BSD] state current\\_action total\\_actions"
            " action\\_percent");
        add_data_header("[BSD] none none none pct");

        // Get the vector of packet descriptors from the config file
        std::vector<std::string> action_types =
            get_param<std::vector<std::string>>("~action_types");

        // Set the starting mode from the config file
        mode = static_cast<MissionMode>(get_param<int>("~starting_mode"));
        prev_mode = mode;

        // Set up the FSD and BSD mission managers
        mission_manager[INTERFACE_FSD] = new MissionManager(action_types);
        mission_manager[INTERFACE_FSD]->set_status_changed_callback(
            std::bind(&MissionManagerNode::status_changed_callback, this,
                std::placeholders::_1, INTERFACE_FSD));
        mission_manager[INTERFACE_BSD] = new MissionManager(action_types);
        mission_manager[INTERFACE_BSD]->set_status_changed_callback(
            std::bind(&MissionManagerNode::status_changed_callback, this,
                 std::placeholders::_1, INTERFACE_BSD));
         mission_manager[INTERFACE_BSD]->set_event_callback(
             std::bind(&MissionManagerNode::event_callback, this,
                  std::placeholders::_1, INTERFACE_BSD));

        // Configure the maximum mission queue durations from the config file
        mission_manager[INTERFACE_FSD]->set_max_queue_duration(
            get_param<double>("~max_fsd_queue_duration"));
        mission_manager[INTERFACE_BSD]->set_max_queue_duration(
            get_param<double>("~max_bsd_queue_duration"));

        // Set up the mission control services
        mission_mode_server = node_handle->advertiseService(
            "system/mission_mode",
            &MissionManagerNode::mission_mode_srv_callback, this);
        mission_control_server = node_handle->advertiseService(
            "system/mission_control",
            &MissionManagerNode::mission_control_srv_callback, this);

        // Set up the ethernet channel TX client and RX subscriber
        ethernet_tx_client =
            node_handle->serviceClient<DataTxSrv>("comms/ethernet_tx");

        // Configure the mode duration timer
        ros::Duration mode_duration = ros::Duration(1.0);
        mode_duration_timer = node_handle->createTimer(mode_duration,
            &MissionManagerNode::mode_duration_timer_callback, this, true);
        mode_duration_timer.stop();

        // Set the packet handler's callback to handle incoming packets
        packet_handler.set_callback(&MissionManagerNode::packet_callback, this);

        // Set up the mission status publisher
        using namespace avl_msgs;
        mission_status_pub = node_handle->advertise<MissionStatusMsg>
                ("system/mission_status", 1, true);
        publish_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function when the node is started.
    //--------------------------------------------------------------------------
    void shutdown()
    {

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    MissionManagerNode node(argc, argv);
    node.start();
    return 0;
}
