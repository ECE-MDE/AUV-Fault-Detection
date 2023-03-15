//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node abstraction base class. Allows for the easy
//              implementation of ROS nodes by deriving from this class. Three
//              virtual functions are provided:
//                  - init: called when the node is started. Place
//                    initialization code here
//                  - run: called after init. This is where the node's main
//                    functionality should be placed
//                  - shutdown: called after run, or when the node is shut down
//                    by the user or an interrupt
//              The node is started by calling the start function, and is
//              stopped when ROS is shutdown. Additionally, each node is
//              equipped with a logger and a command handler.
//==============================================================================

#include <avl_core/node.h>

// Standard ROS functions
#include "ros/ros.h"

// Logger class for logging messages to console and file
#include <avl_core/logger.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Node constructor
//------------------------------------------------------------------------------
Node::Node(int argc, char **argv) : Logger()
{

    // Initialize the node. The node name set here will be overwritten by
    // the node name set in the roslaunch file
    ros::init(argc, argv, "unnamed_node", ros::init_options::AnonymousName);

    // Create a NodeHandle object. This can only be done after the
    // ros::init function is called or ROS will give an error
    node_handle.reset(new ros::NodeHandle());

    // Set the log label to the node name that was either set by the
    // roslaunch file or left unchanged from unnamed_node
    set_log_name(ros::this_node::getName());

    // Console Logging settings to be configured from ROS parameters
    LoggerConfig console_config;

    // Check for global logging parameters for console logging that may be set
    // with a roslaunch file
    if(check_param<bool>("/logging/console/enable"))
        console_config.enable = get_param<bool>("/logging/console/enable");
    if(check_param<bool>("/logging/console/log_node"))
        console_config.log_node = get_param<bool>("/logging/console/log_node");
    if(check_param<bool>("/logging/console/log_data"))
        console_config.log_data = get_param<bool>("/logging/console/log_data");
    if(check_param<bool>("/logging/console/log_debug"))
        console_config.log_debug = get_param<bool>("/logging/console/log_debug");
    if(check_param<bool>("/logging/console/log_info"))
        console_config.log_info = get_param<bool>("/logging/console/log_info");
    if(check_param<bool>("/logging/console/log_warning"))
        console_config.log_warning = get_param<bool>("/logging/console/log_warning");
    if(check_param<bool>("/logging/console/log_error"))
        console_config.log_error = get_param<bool>("/logging/console/log_error");

    // Check for local logging parameters for console logging that may be set
    // with a node's config file. Since these are checked second, they will
    // override any global settings
    if(check_param<bool>("~/logging/console/enable"))
        console_config.enable = get_param<bool>("~/logging/console/enable");
    if(check_param<bool>("~/logging/console/log_node"))
        console_config.log_node = get_param<bool>("~/logging/console/log_node");
    if(check_param<bool>("~/logging/console/log_data"))
        console_config.log_data = get_param<bool>("~/logging/console/log_data");
    if(check_param<bool>("~/logging/console/log_debug"))
        console_config.log_debug = get_param<bool>("~/logging/console/log_debug");
    if(check_param<bool>("~/logging/console/log_info"))
        console_config.log_info = get_param<bool>("~/logging/console/log_info");
    if(check_param<bool>("~/logging/console/log_warning"))
        console_config.log_warning = get_param<bool>("~/logging/console/log_warning");
    if(check_param<bool>("~/logging/console/log_error"))
        console_config.log_error = get_param<bool>("~/logging/console/log_error");

    // Configure the console log
    configure_console_log(console_config);

    // File Logging settings to be configured from ROS parameters
    LoggerConfig file_config;

    // Check for global logging parameters for file logging that may be set
    // with a roslaunch file
    if(check_param<bool>("/logging/file/enable"))
        file_config.enable = get_param<bool>("/logging/file/enable");
    if(check_param<bool>("/logging/file/log_node"))
        file_config.log_node = get_param<bool>("/logging/file/log_node");
    if(check_param<bool>("/logging/file/log_data"))
        file_config.log_data = get_param<bool>("/logging/file/log_data");
    if(check_param<bool>("/logging/file/log_debug"))
        file_config.log_debug = get_param<bool>("/logging/file/log_debug");
    if(check_param<bool>("/logging/file/log_info"))
        file_config.log_info = get_param<bool>("/logging/file/log_info");
    if(check_param<bool>("/logging/file/log_warning"))
        file_config.log_warning = get_param<bool>("/logging/file/log_warning");
    if(check_param<bool>("/logging/file/log_error"))
        file_config.log_error = get_param<bool>("/logging/file/log_error");

    // Check for local logging parameters for file logging that may be set
    // with a node's config file. Since these are checked second, they will
    // override any global settings
    if(check_param<bool>("~/logging/file/enable"))
        file_config.enable = get_param<bool>("~/logging/file/enable");
    if(check_param<bool>("~/logging/file/log_node"))
        file_config.log_node = get_param<bool>("~/logging/file/log_node");
    if(check_param<bool>("~/logging/file/log_data"))
        file_config.log_data = get_param<bool>("~/logging/file/log_data");
    if(check_param<bool>("~/logging/file/log_debug"))
        file_config.log_debug = get_param<bool>("~/logging/file/log_debug");
    if(check_param<bool>("~/logging/file/log_info"))
        file_config.log_info = get_param<bool>("~/logging/file/log_info");
    if(check_param<bool>("~/logging/file/log_warning"))
        file_config.log_warning = get_param<bool>("~/logging/file/log_warning");
    if(check_param<bool>("~/logging/file/log_error"))
        file_config.log_error = get_param<bool>("~/logging/file/log_error");

    // Configure the file log
    set_log_filepath("/var/avl_logs/current/log");
    configure_file_log(file_config);

}

//------------------------------------------------------------------------------
// Name:        Node destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
Node::~Node()
{

}

//------------------------------------------------------------------------------
// Name:        start
// Description: Starts the node by running the init, run, and stop
//              functions.
//------------------------------------------------------------------------------
void Node::start()
{

    // Try running the node initialization function. Catch and report any
    // exceptions
    try
    {

        init_wrapper();

        // Try running the node run function. Catch and report any
        // exceptions
        try
        {
            run_wrapper();
        }
        catch (const std::exception& ex)
        {
            log_error("exception caught while node was running (%s)", ex.what());
        }
        catch (...)
        {
            log_error("exception caught while node was running (unknown non-standard exception)");
        }

    }
    catch (const std::exception& ex)
    {
        log_error("exception caught while node was initializing (%s)", ex.what());
    }
    catch (...)
    {
        log_error("exception caught while node was initializing (unknown non-standard exception)");
    }

    // Try running the node shutdown function. Catch and report any
    // exceptions
    try
    {
        shutdown_wrapper();
    }
    catch (const std::exception& ex)
    {
        log_error("exception caught while node was shutting down (%s)", ex.what());
    }
    catch (...)
    {
        log_error("exception caught while node was shutting down (unknown non-standard exception)");
    }

}

//------------------------------------------------------------------------------
// Name:        init
// Description: Called when the node is started. Can be overriden by a
//              derived node class.
//------------------------------------------------------------------------------
void Node::init()
{

}

//------------------------------------------------------------------------------
// Name:        run
// Description: Called after the init function when the node is started. Can
//              be overriden by a derived node class.
//------------------------------------------------------------------------------
void Node::run()
{

}

//------------------------------------------------------------------------------
// Name:        shutdown
// Description: Called after the run function when the node is started. Can
//              be overriden by a derived node class.
//------------------------------------------------------------------------------
void Node::shutdown()
{

}

//------------------------------------------------------------------------------
// Name:        init_wrapper
// Description: Wrapper for the derived class's init function. Prints debug
//              status messages for init start and stop.
//------------------------------------------------------------------------------
void Node::init_wrapper()
{

    log_node("initializing node...");
    init();
    log_node("finished node initialization");

}

//------------------------------------------------------------------------------
// Name:        run_wrapper
// Description: Wrapper for the derived class's run function. Prints debug
//              status messages for run start and stop.
//------------------------------------------------------------------------------
void Node::run_wrapper()
{

    log_node("running node...");
    run();
    log_node("finished node run");

}

//------------------------------------------------------------------------------
// Name:        shutdown_wrapper
// Description: Wrapper for the derived class's shutdown function. Prints
//              debug status messages for shutdown start and stop.
//------------------------------------------------------------------------------
void Node::shutdown_wrapper()
{

    log_node("shutting down node...");
    shutdown();
    log_node("finished node shutdown");

}
