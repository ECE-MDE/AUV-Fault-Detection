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

#ifndef NODE_H
#define NODE_H

// Standard ROS functions
#include "ros/ros.h"

// Logger class for logging messages to console and file
#include <avl_core/logger.h>

// Included for check_param and get_param
#include <avl_core/util/ros.h>
using namespace avl;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class Node : public Logger
{

public:

    // Pointer to the node's node handle. We want the node handle to be a member
    // of the class, but a node handle instance can only be created after
    // calling ros::init. Therefore, we use a pointer and point it to a new
    // instance of NodeHandle after we call ros::init in the constructor
    std::unique_ptr<ros::NodeHandle> node_handle;

public:

    //--------------------------------------------------------------------------
    // Name:        Node constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    Node(int argc, char **argv);

    //--------------------------------------------------------------------------
    // Name:        Node destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~Node();

    //--------------------------------------------------------------------------
    // Name:        start
    // Description: Starts the node by running the init, run, and stop
    //              functions.
    //--------------------------------------------------------------------------
    void start();

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Called when the node is started. Can be overriden by a
    //              derived node class.
    //--------------------------------------------------------------------------
    virtual void init();

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Called after the init function when the node is started. Can
    //              be overriden by a derived node class.
    //--------------------------------------------------------------------------
    virtual void run();

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function when the node is started. Can
    //              be overriden by a derived node class.
    //--------------------------------------------------------------------------
    virtual void shutdown();

private:

    //--------------------------------------------------------------------------
    // Name:        init_wrapper
    // Description: Wrapper for the derived class's init function. Prints debug
    //              status messages for init start and stop.
    //--------------------------------------------------------------------------
    void init_wrapper();

    //--------------------------------------------------------------------------
    // Name:        run_wrapper
    // Description: Wrapper for the derived class's run function. Prints debug
    //              status messages for run start and stop.
    //--------------------------------------------------------------------------
    void run_wrapper();

    //--------------------------------------------------------------------------
    // Name:        shutdown_wrapper
    // Description: Wrapper for the derived class's shutdown function. Prints
    //              debug status messages for shutdown start and stop.
    //--------------------------------------------------------------------------
    void shutdown_wrapper();

};

#endif // NODE_H
