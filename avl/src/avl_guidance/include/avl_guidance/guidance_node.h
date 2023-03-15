//==============================================================================
// Autonomous Vehicle Library
//
// Description: Contains a guidance node base class.
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Simple action server class. There is a pedantic warning to be ignored
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <actionlib/server/simple_action_server.h>
#pragma GCC diagnostic pop

// ROS actions
#include <avl_msgs/MissionAction.h>
using namespace avl_msgs;

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>

//==============================================================================
//                                 TYPEDEFS
//==============================================================================

typedef MissionGoalConstPtr GoalConstPtr;
typedef MissionFeedback     Feedback;
typedef MissionResult       Result;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class GuidanceNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    GuidanceNode(std::string server_name, int argc, char **argv) :
        Node(argc, argv)
    {

        // Set up the action server with its execute callback and name and set
        // auto-start to false
        action_server = new ActionServer(server_name,
            boost::bind(&GuidanceNode::execute_callback, this, _1), false);

        // Configure the action server's preempt callback
        action_server->registerPreemptCallback(
            boost::bind(&GuidanceNode::preempt_callback, this));

        // Start the action server
        action_server->start();

    }

protected:

    //--------------------------------------------------------------------------
    // Name:        start_new_action
    // Description: Called when a new action is received.
    //--------------------------------------------------------------------------
    virtual bool start_new_action(Action action) = 0;

    //--------------------------------------------------------------------------
    // Name:        iterate_action
    // Description: Called at the iteration rate while an action is
    //              executing. Main guidance node logic, such as publishing
    //              controller setpoints, should be executed here.
    //--------------------------------------------------------------------------
    virtual Feedback iterate_action() = 0;

    //--------------------------------------------------------------------------
    // Name:        stop_action
    // Description: Called when a action has been finished or has been canceled
    //              by the client.
    //--------------------------------------------------------------------------
    virtual void stop_action() = 0;

    //--------------------------------------------------------------------------
    // Name:        finish_action
    // Description: Declares the action finished either successfully or
    //              unsuccessfully and gives a result.
    // Arguments:   - success: true if the action completed successfully, false
    //                if it failed.
    //              - result: action result message
    //--------------------------------------------------------------------------
    void finish_action(bool success, Result result)
    {

        if (action_server->isActive())
        {
            if (success)
            {
                action_server->setSucceeded(result);
                log_debug("finish_action called (success)");
            }
            else
            {
                action_server->setAborted(result);
                log_debug("finish_action called (failed)");
            }
        }
        else
        {
            log_debug("finish_action called but action is no longer active");
        }

        action_complete = true;
        time_since_start = 0.0;

    }

    //--------------------------------------------------------------------------
    // Name:        finish_action
    // Description: Returns true of the current goal is active. False otherwise
    // Returns:     - True if the goal is active. False otherwise
    //--------------------------------------------------------------------------
    bool action_is_active()
    {
        return action_server->isActive();
    }

    //--------------------------------------------------------------------------
    // Name:        get_time_since_start
    // Description: Returns the number of seconds since the action was started
    //              or zero if there is no active action.
    // Returns:     Number of seconds since the action was started or zero if
    //              there is no active action.
    //--------------------------------------------------------------------------
    double get_time_since_start()
    {
        return time_since_start;
    }

    Action get_action()
    {
        return action;
    }

private:

    // Action server for action execution
    typedef actionlib::SimpleActionServer<MissionAction> ActionServer;
    ActionServer* action_server;

    // Action being executed
    Action action;

    // Flag indicating whether a action is completed or still in progress
    bool action_complete = true;

    // Timer and its duration for a action's duration. If a action reaches the
    // end of its duration before it is completed, it will be aborted
    ros::Duration duration_timer_duration;
    ros::Timer duration_timer;
    double time_since_start = 0.0;

    // Timer and its duration for iterating at a set rate when the action starts
    ros::Timer iterate_timer;
    ros::Duration iterate_timer_duration;

private:

    //--------------------------------------------------------------------------
    // Name:        preempt_callback
    // Description:
    //--------------------------------------------------------------------------
    void preempt_callback()
    {
        log_debug("preempt_callback called");
        action_complete = true;
        action_server->setPreempted();
        duration_timer.stop();
        iterate_timer.stop();
        time_since_start = 0.0;
    }

    //--------------------------------------------------------------------------
    // Name:        execute_callback
    // Description: Called when a new action goal is received.
    //--------------------------------------------------------------------------
    void execute_callback(const GoalConstPtr& goal)
    {

        // If the action server is not active, an action cannot be started
        if (!action_server->isActive())
        {
            log_error("execute_callback called on inactive server");
            return;
        }

        // Create an action packet from the bytes in the goal and get the
        // corresponding action struct
        ActionPacket packet(goal->packet);
        action = packet.get_action();

        // Log the action parameters
        log_info("attempting to execute action with parameters:");
        for (const Parameter& param : action.parameters)
            log_info("name: %-12s   type: %d   value: %s",
                param.name.c_str(),
                param.type,
                avl::byte_to_hex(param.value).c_str());

        // Get the DURATION parameter from the action if it has one in order to
        // set the duration timer. If an action does not have a duration,
        // assume a 1 second duration.
        if (action.parameters.has("DURATION"))
            duration_timer_duration = ros::Duration(
                action.parameters.get("DURATION").to_double());
        else
            duration_timer_duration = ros::Duration(1.0);

        // Call the start_new_action callback so that child classes can
        // accept or reject the action and initialize for it. Only continue with
        // the action if it is accepted, indicated by the start_new_action
        // return value
        action_complete = false;
        if (start_new_action(action))
        {

            log_info("new action accepted, starting iteration...");

            // Start the action duration timer
            duration_timer.setPeriod(duration_timer_duration);
            duration_timer.start();

            // Call the iterate action callback so that iteration starts
            // immediately, then start the iterate timer to continue action
            // iteration
            iterate_action();
            time_since_start = 0.0;
            iterate_timer.setPeriod(iterate_timer_duration);
            iterate_timer.start();

            // Spin to wait for the action to complete, which is when the user
            // calls the finish_action function
            ros::Rate spin_rate(1000);
            while(ros::ok() && action_server->isActive() && !action_complete)
            {
                ros::spinOnce();
                spin_rate.sleep();
            }

            log_info("action completed, stopping iteration");

            // Stop the iterate timer to stop action iteration
            iterate_timer.stop();

            // Stop the action duration timer since the action has completed
            duration_timer.stop();

        }
        else
        {

            // TODO: What should happen when an action is rejected?
            throw std::runtime_error("action was rejected by action server");

        }

    }

    //--------------------------------------------------------------------------
    // Name:        duration_timer_callback
    // Description: Called when the action duration timer expires.
    // Argument:    - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void duration_timer_callback(const ros::TimerEvent& event)
    {
        log_debug("action duration timer expired, ending action");
        finish_action(false, Result());
    }

    //--------------------------------------------------------------------------
    // Name:        iterate_timer_callback
    // Description: Iteration timer callback. Called when the iteration timer
    //              expires.
    // Arguments:   - event: timer event struct
    //--------------------------------------------------------------------------
    void iterate_timer_callback(const ros::TimerEvent& event)
    {
        Feedback feedback = iterate_action();
        action_server->publishFeedback(feedback);
        time_since_start += iterate_timer_duration.toSec();
    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run() final
    {

        // Configure the action duration timer
        duration_timer_duration = ros::Duration(1.0);
        duration_timer = node_handle->createTimer(
            duration_timer_duration,
            &GuidanceNode::duration_timer_callback, this, true);
        duration_timer.stop();

        // Configure the iterate timer
        double action_iterate_rate = get_param<double>("~iteration_rate");
        iterate_timer_duration = ros::Duration(1.0/action_iterate_rate);
        iterate_timer = node_handle->createTimer(iterate_timer_duration,
            &GuidanceNode::iterate_timer_callback, this);
        iterate_timer.stop();

        ros::spin();

    }

};
