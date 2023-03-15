//==============================================================================
// Autonomous Vehicle Library
//
// Description: Controls the enabling and disabling of logging and creation of
//              new logging folders.
//
// Servers:     /logging/split_logs (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: system/mission_status (avl_msgs/MissionStatusMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Command handler class
#include <avl_comms/command_handler.h>

// C++ includes
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

// ROS messages
#include <avl_msgs/MissionStatusMsg.h>
#include <std_srvs/Trigger.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class LoggingControlNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        LoggingControlNode constructor
    //--------------------------------------------------------------------------
    LoggingControlNode(int argc, char **argv) : Node(argc, argv) { }

private:

    // Server for splitting the log folder
    ros::ServiceServer split_logs_server;

    // Subscriber for mission status data
    ros::Subscriber mission_status_sub;
    State prev_state = STATE_INACTIVE;

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

private:

    //--------------------------------------------------------------------------
    // Name:        get_timestamp
    // Description: Gets a string representing the current date and time.
    // Returns:     String representing the current date and time.
    //--------------------------------------------------------------------------
    std::string get_timestamp()
    {

        // Get the current time info struct
        time_t now = time(NULL);
        tm* time_info = localtime(&now);

        // Format the time as a string
        char buffer [256];
        strftime(buffer, 256, "%F_%H.%M.%S", time_info);
        return std::string(buffer);

    }

    //--------------------------------------------------------------------------
    // Name:        create_new_log_folder
    // Description: Creates a new timestamped log folder in /var/avl_logs,
    //              creates the log and config folders inside of it, copis all
    //              current config files, and sets the symlink from
    //              /var/avl_logs/current to the new timestamped log folder.
    //--------------------------------------------------------------------------
    void create_new_log_folder(std::string tag)
    {

        log_info("creating new log folder...");

        // Create the folder filepaths
        std::string timestamp = get_timestamp();
        std::string base_dir_path = "/var/avl_logs/" + timestamp + " [" + tag + "]";
        std::string log_dir_path = base_dir_path + "/log";
        std::string config_dir_path = base_dir_path + "/config";
        std::string current_dir_path = "/var/avl_logs/current";
        std::string new_current_dir_path = "/var/avl_logs/new_current";
        std::string config_files_path = "/var/avl_config";

        try
        {

            // Create the timestamped log folder and the folders in it
            fs::create_directories(log_dir_path);
            log_info("created directory " + log_dir_path);
            // fs::create_directories(config_dir_path);
            // log_info("created directory " + config_dir_path);

            // Copy all config files into the config folder
            fs::copy(config_files_path, config_dir_path);
            log_info("copied .config files from %s to %s",
                config_files_path.c_str(), config_dir_path.c_str());

            // Create a symbolic link that points from new_current to the log
            // folder .We cannot directly create a symbolic link to current
            // because it throws an error about current already existing and
            // there is no force option for this function. new_current can then
            // be renamed to current
            fs::create_directory_symlink(base_dir_path, new_current_dir_path);
            fs::rename(new_current_dir_path, current_dir_path);
            log_info("created symbolic link (%s -> %s)",
                current_dir_path.c_str(), base_dir_path.c_str());

        }
        catch (std::exception& ex)
        {
            log_error("log splitting process failed (%s)", ex.what());
        }

    }

    //--------------------------------------------------------------------------
    // Name:        mission_status_msg_callback
    // Description: Called when a mission status message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void mission_status_msg_callback(const avl_msgs::MissionStatusMsg& message)
    {

        if (get_param<bool>("~split_on_mission_transition"))
        {

            // Create a new logging folder when the mission state changes to
            // ACTIVE or INACTIVE
            State state = static_cast<State>(message.fsd_mission_state);
            if ((state != prev_state) &&
                (state == STATE_ACTIVE || state == STATE_INACTIVE))
            {
                log_info("mission state changed, splitting logs");
                create_new_log_folder(state == STATE_ACTIVE ? "A" : "I");
                prev_state = state;
            }

        }

    }

    //--------------------------------------------------------------------------
    // Name:        split_logs_srv_callback
    // Description: Called when the logging_control service is requested.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool split_logs_srv_callback(std_srvs::Trigger::Request& request,
                                 std_srvs::Trigger::Response& response)
    {
        create_new_log_folder("M");
        response.success = true;
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when a COMMAND packet is received by the
    //              communication architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - command_name: Command name of the received command.
    //              - params: Command parameter list.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool command_callback(CommsChannel channel, CommsInterface interface,
        std::string command_name, ParameterList params,
        bool& result, std::vector<uint8_t>& data)
    {

        // Handle SPLIT LOGS commands
        if (command_name == "SPLIT LOGS")
        {
            create_new_log_folder("M");
            result = true;
            return true;
        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Set up the service server for creating a new log folder
        split_logs_server = node_handle->advertiseService("logging/split_logs",
            &LoggingControlNode::split_logs_srv_callback, this);

        // Set up the subscriber for mission status data.
        mission_status_sub = node_handle->subscribe( "system/mission_status",
            10, &LoggingControlNode::mission_status_msg_callback, this);

        // Set the command handler's callback
        command_handler.set_callback(
            &LoggingControlNode::command_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    LoggingControlNode node(argc, argv);
    node.start();
    return 0;
}
