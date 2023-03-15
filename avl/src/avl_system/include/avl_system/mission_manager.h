//==============================================================================
// Autonomous Vehicle Library
//
// Description: A claas manages a mission queue and the control of actions in
//              the mission queue. Provides functions to configure the mission
//              queue and functions to control the operation of the mission
//              queue. Contains action clients to execute actions when they are
//              started and allows the user to set a callback that is called
//              whenever the mission's state changes.
//==============================================================================

#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

// Simple action client class. There is a pedantic warning to be ignored
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <actionlib/client/simple_action_client.h>
#pragma GCC diagnostic pop
using namespace actionlib;

// Mission action message
#include <avl_msgs/MissionAction.h>
using namespace avl_msgs;

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

//==============================================================================
//                                  TYPEDEFS
//==============================================================================

// Mapping between a packet descriptor and a guidance node name and
// corresponding action client
typedef SimpleActionClient<MissionAction> MissionActionClient;
typedef std::map<std::string, std::unique_ptr<MissionActionClient>> ClientMap;

//==============================================================================
//                           MISSION STATUS STRUCT
//==============================================================================

// Enum listing possible states the mission can be in
enum MissionState
{
    MISSION_ACTIVE,
    MISSION_PAUSED,
    MISSION_INACTIVE
};

// Struct containing the status of the mission
struct MissionStatus
{

    // Current mission state
    MissionState state = MISSION_INACTIVE;

    // Total number of actions in the queue
    uint16_t total_actions = 0;

    // Current action that is/will be executed
    uint16_t current_action = 0;

    // Percent completion of the current action
    float action_percent = 0.0;

};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class MissionManager
{

public:

    //--------------------------------------------------------------------------
    // Name:        state_to_string
    // Description: Converts a mission state to a string for printing.
    // Arguments:   - state: Mission state to convert to a string.
    // Returns:     String representing the mission state.
    //--------------------------------------------------------------------------
    static std::string state_to_string(MissionState state)
    {
        if (state == MISSION_ACTIVE)
            return "ACTIVE";
        else if (state == MISSION_PAUSED)
            return "PAUSED";
        else
            return "INACTIVE";
    }

public:

    //--------------------------------------------------------------------------
    // Name:        MissionManager constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    MissionManager(std::vector<std::string> action_names)
    {

        // Initialize an action server in the client map for every action name
        for (const auto& name : action_names)
        {
            std::string server_name = "guidance/" + name;
            client_map[name] = std::unique_ptr<MissionActionClient>(
                new MissionActionClient(server_name, false));
        }

    }

    //--------------------------------------------------------------------------
    // Name:        MissionManager destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~MissionManager()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        set_max_queue_duration
    // Description: Sets the maximum allowable total duration of the action
    //              queue. The total action queue duration is calculated as
    //              the sum of action durations of all actions in the queue.
    //              Attempting to append actions that will break this limit
    //              will fail.
    // Arguments:   - action: action to be appended to the queue.
    //--------------------------------------------------------------------------
    void set_max_queue_duration(double duration)
    {
        max_queue_duration = duration;
    }

    //--------------------------------------------------------------------------
    // Name:        set_status_changed_callback
    // Description: Sets the callback to be called when the mission queue's
    //              status changes.
    // Arguments:   - callback: Mission status changed callback function.
    //--------------------------------------------------------------------------
    void set_status_changed_callback(std::function<void(MissionStatus)> callback)
    {
        status_changed_callback = callback;
    }

    //--------------------------------------------------------------------------
    // Name:        set_event_callback
    // Description: Sets the callback to be called for event notifications
    // Arguments:   - callback: Mission status changed callback function.
    //--------------------------------------------------------------------------
    void set_event_callback(std::function<void(Event)> callback)
    {
        event_callback = callback;
    }

    //--------------------------------------------------------------------------
    // Name:        is_empty
    // Description: Returns true if the mission queue is empty, and false if it
    //              contains one or more actions.
    // Returns:     True if the mission queue is empty, false if it contains one
    //              or more actions.
    //--------------------------------------------------------------------------
    bool is_empty()
    {
        return queue.empty();
    }

    //--------------------------------------------------------------------------
    // Name:        is_active
    // Description: Returns true if the mission is active.
    // Returns:     True if the mission queue is active.
    //--------------------------------------------------------------------------
    bool is_active()
    {
        return state == MISSION_ACTIVE;
    }

    //--------------------------------------------------------------------------
    // Name:        is_paused
    // Description: Returns true if the mission is active.
    // Returns:     True if the mission queue is active.
    //--------------------------------------------------------------------------
    bool is_paused()
    {
        return state == MISSION_PAUSED;
    }

    //--------------------------------------------------------------------------
    // Name:        is_inactive
    // Description: Returns true if the mission is active.
    // Returns:     True if the mission queue is active.
    //--------------------------------------------------------------------------
    bool is_inactive()
    {
        return state == MISSION_INACTIVE;
    }

    //--------------------------------------------------------------------------
    // Name:        size
    // Description: Returns the number of actions contained in the queue.
    // Returns:     Number of actions contained in the queue.
    //--------------------------------------------------------------------------
    size_t size()
    {
        return queue.size();
    }

    //--------------------------------------------------------------------------
    // Name:        get_actions
    // Description: Gets a vector containing all of the actions in the queue.
    // Returns:     Vector with all of the actions in the queue.
    //--------------------------------------------------------------------------
    std::vector<ActionPacket> read()
    {
        return queue;
    }

    //--------------------------------------------------------------------------
    // Name:        execute
    // Description: Sets the current action to the given action and automatically
    //              starts the mission. Any actions currently in the queue will
    //              be cleared. Used for real time control of the action.
    // Arguments:   - action: Action to set as current action in the queue.
    //--------------------------------------------------------------------------
    void execute(ActionPacket action)
    {

        // Check to see if there is a client for the action name
        std::string action_name = action.get_action().name;
        if (!map_has_client(action_name))
            throw std::runtime_error("no action server for action name "
                + action_name);

        // Check that the action's duration is not longer than the maximum
        // allowable mission queue duration
        if (get_action_duration(action) > max_queue_duration)
            throw std::runtime_error("action duration is greater than maximum "
                "allowable mission queue duration");

        // Cancel any active goals. Set the cancelled flag so that advance
        // is not called when the action_done_callback is called by the
        // canceling of an active action
        if (other_client_active(action_name))
        {
            cancelled = true;
            cancel_action_goals();
        }

        // Setting the mission queue clears all current actions from the queue
        // and puts the new action in the queue
        clear();
        append(action);

        // Immediately execute the new action
        execute_current_action();

        notify_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        set
    // Description: Sets the current action to the given action. Any actions
    //              currently in the queue will be cleared. Used for real time
    //              control of the action.
    // Arguments:   - action: Action to set as current action in the queue.
    //--------------------------------------------------------------------------
    void set(ActionPacket action)
    {

        // Check to see if there is a client for the action name
        std::string action_name = action.get_action().name;
        if (!map_has_client(action_name))
            throw std::runtime_error("no action server for action name "
                + action_name);

        // Check that the action's duration is not longer than the maximum
        // allowable mission queue duration
        if (get_action_duration(action) > max_queue_duration)
            throw std::runtime_error("action duration is greater than maximum "
                "allowable mission queue duration");

        // Cancel any active goals. Set the cancelled flag so that advance
        // is not called when the action_done_callback is called by the
        // canceling of an active action
        if (other_client_active(action_name))
        {
            cancelled = true;
            cancel_action_goals();
        }

        // Setting the mission queue clears all current actions from the queue
        // and puts the new action in the queue
        clear();
        append(action);

        // If the mission is active when set is called, we want to immediately
        // execute the new action
        if (state == MISSION_ACTIVE)
            execute_current_action();

        notify_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        append
    // Description: Appends an action to the end of the mission queue.
    // Arguments:   - action: Action to be appended to the end of the queue.
    //--------------------------------------------------------------------------
    void append(ActionPacket action)
    {

        // Check to see if there is a client for the action name
        std::string action_name = action.get_action().name;
        if (!map_has_client(action_name))
            throw std::runtime_error("no action server for action name "
                + action_name);

        // Check that adding the action would not make the total mission
        // queue duration longer than the maximum allowable duration
        if (get_action_duration(action) > max_queue_duration)
            throw std::runtime_error("appending action would cause mission "
                "queue to exceed maximum allowable duration");

        queue.push_back(action);
        notify_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        start
    // Description: Starts the mission queue.
    //--------------------------------------------------------------------------
    void start()
    {

        // The mission cannot be started if the mission queue is empty
        if (is_empty())
            throw std::runtime_error("start: mission queue is empty");

        // The mission cannot be started if it is already active
        if (state == MISSION_ACTIVE)
            throw std::runtime_error("start: mission is already active");

        execute_current_action();
        notify_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        pause
    // Description: Pauses the mission queue.
    //--------------------------------------------------------------------------
    void pause()
    {

        // The mission can only be paused if it is active
        if (state == MISSION_ACTIVE)
        {
            state = MISSION_PAUSED;
            action_percent = 0.0;
            cancel_action_goals();
            notify_mission_status();
        }

    }

    //--------------------------------------------------------------------------
    // Name:        stop
    // Description: Stops the mission queue.
    //--------------------------------------------------------------------------
    void stop()
    {

        // The mission can only be stopped if it is active
        if (state == MISSION_ACTIVE)
        {
            state = MISSION_INACTIVE;
            action_percent = 0.0;
            cancel_action_goals();
            notify_mission_status();
        }

    }

    //--------------------------------------------------------------------------
    // Name:        advance
    // Description: Advances to the next action by discarding the current
    //              action and moving to the next one.
    //--------------------------------------------------------------------------
    void advance()
    {

        current_action_num++;

        // If we have reached the end of the mission queue, set the mission as
        // inactive and clear the missions from the queue
        if (current_action_num >= queue.size())
        {
            state = MISSION_INACTIVE;
            clear();
        }

        // If we haven't reached the end of the mission queue and the mission
        // is active, execute the next action
        else if (state == MISSION_ACTIVE)
            execute_current_action();

        notify_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        clear
    // Description: Clears all actions from the action queue.
    //--------------------------------------------------------------------------
    void clear()
    {
        queue.clear();
        current_action_num = 0;
        action_percent = 0.0;
        notify_mission_status();
    }

private:

    // Mission queue holding action packets in execution order
    std::vector<ActionPacket> queue;

    // Action number that is currently up for execution
    size_t current_action_num = 0;

    // Current state of the mission
    MissionState state = MISSION_INACTIVE;

    // Most recent percent completion received from action server feedback
    float action_percent = 0.0;

    // Mapping between action name and action client so that we know
    // which action server to send the action to
    ClientMap client_map;

    // Callback set by the user to be called when the mission status changes
    std::function<void(MissionStatus)> status_changed_callback;

    // Callback set by the user to be called for event notifications
    std::function<void(Event)> event_callback;

    // the maximum allowable total duration of the action queue. The total
    // action queue duration is calculated as the sum of action durations of
    // all actions in the queue. Attempting to append actions that will break
    // this limit will fail.
    double max_queue_duration = 0.0;

    // Flag indicating that a set command is canceling actions and advance
    // should not be called when the cancelling causes action_done_callback to
    // be called
    bool cancelled = false;

private:

    //--------------------------------------------------------------------------
    // Name:        map_has_client
    // Description: Checks if there is a client corresponding to an action name
    //              in the client map.
    // Arguments:   - name: Action name.
    // Returns:     True if there is a corresponding client, false otherwise.
    //--------------------------------------------------------------------------
    bool map_has_client(std::string name)
    {
        for (const auto& entry : client_map)
            if (entry.first == name)
                return true;
        return false;
    }

    //--------------------------------------------------------------------------
    // Name:        get_action_duration
    // Description: Gets the duration of an action.
    // Returns:     Action duration in seconds.
    //--------------------------------------------------------------------------
    double get_action_duration(ActionPacket packet)
    {
        Action action = packet.get_action();
        if (action.parameters.has("DURATION"))
            return action.parameters.get("DURATION").to_double();
        else
            return 0.0;
    }

    //--------------------------------------------------------------------------
    // Name:        get_queue_duration
    // Description: Gets the total duration of all actions in the mission queue.
    // Returns:     Total mission queue duration in seconds.
    //--------------------------------------------------------------------------
    double get_queue_duration()
    {

        // In order to calculate the total mission qurur duration, we will loop
        // through every action packet in the mission queue and add up the
        // durations
        double queue_duration = 0.0;
        for (auto packet : queue)
            queue_duration += get_action_duration(packet);
        return queue_duration;

    }

    //--------------------------------------------------------------------------
    // Name:        notify_event
    // Description: Calls the user-defined event callback with the
    //              event notification.
    //--------------------------------------------------------------------------
    void notify_event(EventType type, bool result = true)
    {
        Event event;
        event.type = type;
        event.result = result;

        if(event_callback != nullptr)
            event_callback(event);
    }

    //--------------------------------------------------------------------------
    // Name:        notify_mission_status
    // Description: Calls the user-defined mission status callback with the
    //              current status of the mission.
    //--------------------------------------------------------------------------
    void notify_mission_status()
    {
        MissionStatus status;
        status.state = state;
        status.total_actions = queue.size();
        status.current_action = (queue.size() > 0) ? current_action_num + 1 : 0;
        status.action_percent = action_percent;
        status_changed_callback(status);
    }

    //--------------------------------------------------------------------------
    // Name:        action_active_callback
    // Description: Called when an action becomes active.
    //--------------------------------------------------------------------------
    void action_active_callback()
    {
        notify_mission_status();
    }

    //--------------------------------------------------------------------------
    // Name:        action_done_callback
    // Description: Called when an action is completed.
    // Argument:    - state: goal state upon completion
    //              - result: pointer to result message
    //--------------------------------------------------------------------------
    void action_done_callback(const SimpleClientGoalState& state,
        const MissionResultConstPtr& result)
    {

        // Only advance if this callback is not a result of canceling an
        // action via a set command
        if (!cancelled)
            advance();

        notify_event(EVENT_TYPE_ACTION_COMPLETE, result->success);

        // Reset the flag so that missions after the a set command can advance
        cancelled = false;

        notify_mission_status();

    }

    //--------------------------------------------------------------------------
    // Name:        action_feedback_callback
    // Description: Called when action feedback is received.
    // Argument:    - feedback: pointer to feedback message
    //--------------------------------------------------------------------------
    void action_feedback_callback(const MissionFeedbackConstPtr& feedback)
    {
        action_percent = feedback->percent;
        notify_mission_status();
    }

    //--------------------------------------------------------------------------
    // Name:        other_client_active
    // Description: Checks if any client, other than the one specified, has an
    //               active goal.
    // Returns:     True if there is an active client, false if there is not.
    //--------------------------------------------------------------------------
    bool other_client_active(const std::string &name )
    {
        for (const auto& element : client_map)
            if (element.second->getState() == SimpleClientGoalState::ACTIVE)
            {
                if (element.first != name)
                    return true;
            }
        return false;
    }

    //--------------------------------------------------------------------------
    // Name:        cancel_action_goals
    // Description: Cancels all action goals through the action clients.
    //--------------------------------------------------------------------------
    void cancel_action_goals()
    {
        for (const auto& element : client_map)
            element.second->cancelAllGoals();
    }

    //--------------------------------------------------------------------------
    // Name:        execute_current_action
    // Description: Executes the action at the top of the mission queue.
    //--------------------------------------------------------------------------
    void execute_current_action()
    {

        // Get the action packet that is up next for execution
        ActionPacket action_packet = queue.at(current_action_num);
        Action action = action_packet.get_action();

        // Check to see if the action server is connected
        if (!client_map[action.name]->isServerConnected())
            throw std::runtime_error("cannot connect to a guidance node with "
                "name /guidance/" + action.name);

        // Create the goal from the packet
        MissionGoal goal;
        goal.packet = action_packet.get_bytes();

        // Bind the action callbacks
        auto done_callback = boost::bind(
            &MissionManager::action_done_callback, this, _1, _2);
        auto active_callback = boost::bind(
            &MissionManager::action_active_callback, this);
        auto feedback_callback = boost::bind(
            &MissionManager::action_feedback_callback, this, _1);

        // Send the goal to the action client
        client_map[action.name]->sendGoal(goal, done_callback, active_callback,
            feedback_callback);

        state = MISSION_ACTIVE;

    }

};

#endif // MISSION_MANAGER_H
