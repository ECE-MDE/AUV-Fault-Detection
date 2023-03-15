//==============================================================================
// Autonomous Vehicle Library
//
// Description: This class is the base class for all control nodes. A control
//              node consists of a set of inputs, setpoints, and outputs. The
//              node monitors inputs and setpoints and, if setpoints are
//              received, calls an iterate function at a specified rate to
//              allow a child class to calculate and publlish outputs.
//
//              The control node handles inputs and setpoints as follows:
//              If setpoint messages are not received at the specified rate, the
//              setpoint will be disabled. If any of the added setpoints are
//              received, the control node will call the iterate function at the
//              specified rate. If no setpoints are received, iteration will be
//              stopped and the disable function will be called. If a setpoint
//              is active but input messages are not received at the specified
//              rate, the node will terminate.
//==============================================================================

#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H

// Node base class
#include <avl_core/node.h>

// Monitored subscriber class
#include <avl_core/monitored_subscriber.h>

// Safety abort
#include <avl_msgs/SafetyAbortSrv.h>

// ROS messages
#include <avl_msgs/Float64SetpointMsg.h>
#include <avl_msgs/LineSetpointMsg.h>
#include <avl_msgs/OrbitSetpointMsg.h>
using namespace avl_msgs;

// Boost any class
#include <boost/any.hpp>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ControlNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ControlNode constructor
    //--------------------------------------------------------------------------
    ControlNode(int argc, char **argv) : Node(argc, argv),
        iteration_duration(1.0)
    {

    }

    //--------------------------------------------------------------------------
    // Name:        set_iteration_rate
    // Description: Sets the control node's iteration rate.
    // Arguments:   - rate: Control node iteration rate in Hz.
    //--------------------------------------------------------------------------
    void set_iteration_rate(double rate)
    {
        iteration_duration = ros::Duration(1.0/rate);
        iteration_timer.setPeriod(iteration_duration);
    }

    //--------------------------------------------------------------------------
    // Name:        add_input
    // Description: Adds an input topic. If the topic stops receiving messages
    //              faster than the specified rate, a safety fault will be
    //              raised.
    // Arguments:   - topic_name: Input topic name.
    //              - min_rate: Minimum message rate in Hz before a safety fault
    //                is raised.
    //--------------------------------------------------------------------------
    template <typename T>
    void add_input(std::string topic_name, double min_rate)
    {

        // Create a new monitored subscriber and configure it to monitor the
        // message rate
        MonitoredSubscriber<T> new_sub;
        new_sub.set_message_rate(min_rate);
        new_sub.enable_message_rate_check(true);
        auto fault_callback = std::bind(&ControlNode::input_fault_callback<T>,
            this, std::placeholders::_1, topic_name);
        new_sub.subscribe(topic_name, 10, nullptr, fault_callback);

        // Add the new subscriber to the input map
        input_map[topic_name] = new_sub;

    }

    //--------------------------------------------------------------------------
    // Name:        add_setpoint
    // Description: Adds a setpoint topic. If setpoint messages are not received
    //              faster than the specified rate, the control node will
    //              disable control of that setpoint. If all setpoints are
    //              disabled, the control node will not iterate. A setpoint
    //              message must have a boolean member variable named enable.
    // Arguments:   - topic_name: Input topic name.
    //              - min_rate: Minimum message rate in Hz before a safety fault
    //                is raised.
    //--------------------------------------------------------------------------
    template <typename T>
    void add_setpoint(std::string topic_name, double min_rate)
    {

        // Create a new monitored subscriber and configure it to monitor the
        // message rate
        MonitoredSubscriber<T> new_sub;
        new_sub.set_message_rate(min_rate);
        new_sub.enable_message_rate_check(true);
        auto msg_cb = std::bind(&ControlNode::setpoint_msg_callback<T>,
            this, std::placeholders::_1, topic_name);
        auto fault_cb = std::bind(&ControlNode::setpoint_fault_callback<T>,
            this, std::placeholders::_1, topic_name);
        new_sub.subscribe(topic_name, 10, msg_cb, fault_cb);

        // Add the new subscriber to the setpoint map
        setpoint_map[topic_name] = {new_sub, false};

    }

    //--------------------------------------------------------------------------
    // Name:        get_input
    // Description: Gets the most recent input message from the specified topic.
    // Arguments:   - topic_name: Topic name from which to get the most recent
    //                input message.
    // Returns:     Most recent input message from the specified topic.
    //--------------------------------------------------------------------------
    template <typename T>
    T get_input(std::string topic_name)
    {

        // Check that the topic name exists
        if (!has_input(topic_name))
            throw std::runtime_error(std::string("control node has no input "
                "with topic name ") + topic_name);

        // Attempt to get the last message from the monitored subscriber
        T last_mesage;
        try
        {
            auto sub_ptr = boost::any_cast<MonitoredSubscriber<T>>(
                &input_map.at(topic_name));
            last_mesage = sub_ptr->get_last_message();
        }
        catch (const std::exception& ex)
        {
            throw std::runtime_error(std::string("unable to get input from "
                "topic " + topic_name + " (message type may be incorrect)"));
        }

        return last_mesage;

    }

    //--------------------------------------------------------------------------
    // Name:        get_setpoint
    // Description: Gets the most recent setpoint message from the specified
    //              topic.
    // Arguments:   - topic_name: Topic name from which to get the most recent
    //                setpoint message.
    // Returns:     Most recent setpoint message from the specified topic.
    //--------------------------------------------------------------------------
    template <typename T>
    T get_setpoint(std::string topic_name)
    {

        // Check that the topic name exists
        if (!has_setpoint(topic_name))
            throw std::runtime_error(std::string("control node has no setpoint"
                " with topic name ") + topic_name);

        // Attempt to get the last message from the monitored subscriber
        T last_mesage;
        try
        {
            auto sub_ptr = boost::any_cast<MonitoredSubscriber<T>>(
                &setpoint_map.at(topic_name).first);
            last_mesage = sub_ptr->get_last_message();
        }
        catch (const std::exception& ex)
        {
            throw std::runtime_error(std::string("unable to get setpoint from"
                " topic " + topic_name + " (message type may be incorrect)"));
        }

        return last_mesage;

    }

    //--------------------------------------------------------------------------
    // Name:        get_float64_setpoint
    // Description: Gets the most recent setpoint value from a setpoint with
    //              message type  avl_control/Float64SetpointMsg. If control of
    //              the setpoint is disabled, the function will return the
    //              specified disabled value.
    // Arguments:   - topic_name: Setpoint topic name.
    //              - disabled_value: Value that will be returned if the
    //                setpoint is disabled.
    // Returns:     The most recent float64 setpoint value received on the
    //              topic. Note that this is the value (msg.data) and not the
    //              message itself.
    //--------------------------------------------------------------------------
    double get_float64_setpoint(std::string topic_name,
        double disabled_value=NAN)
    {
        Float64SetpointMsg msg = get_setpoint<Float64SetpointMsg>(topic_name);
        if (setpoint_map[topic_name].second)
            return msg.data;
        else
            return disabled_value;
    }

private:

    // Maps linking the input and setpoint topic names to their monitored
    // subscriber
    std::map<std::string, std::pair<boost::any, bool>> setpoint_map;
    std::map<std::string, boost::any> input_map;

    // Timer for controller output iterations and its corresponding rate
    ros::Timer iteration_timer;
    ros::Duration iteration_duration;
    bool iterating = false;

    // Flag indicating whether the disable callback has been called. We do not
    // want to call disable repeatedly when a setpoint topic continues to
    // time out
    bool disable_called = true;

    // Service client for safety node aborts
    ros::ServiceClient abort_client;

private:

    //--------------------------------------------------------------------------
    // Name:        has_input
    // Description: Checks whether an input with the given topic name has been
    //              added to the input map.
    // Arguments:   - topic_name: Name of topic to check for.
    // Returns:     True if the topic is in the input list, false if it is not.
    //--------------------------------------------------------------------------
    bool has_input(std::string topic_name)
    {
        return input_map.find(topic_name) != input_map.end();
    }

    //--------------------------------------------------------------------------
    // Name:        has_setpoint
    // Description: Checks whether a setpoint with the given topic name has been
    //              added to the setpoint map.
    // Arguments:   - topic_name: Name of topic to check for.
    // Returns:     True if the topic is in the setpoint list, false if it is
    //              not.
    //--------------------------------------------------------------------------
    bool has_setpoint(std::string topic_name)
    {
        return setpoint_map.find(topic_name) != setpoint_map.end();
    }

    //--------------------------------------------------------------------------
    // Name:        all_setpoints_disabled
    // Description: Checks whether all setpoints are disabled.
    // Returns:     True if all setpoints are disabled, false if at least one
    //              setpoint is enabled.
    //--------------------------------------------------------------------------
    bool all_setpoints_disabled()
    {
        bool any_enabled = false;
        for (auto&& elem : setpoint_map)
            any_enabled = any_enabled || elem.second.second;
        return !any_enabled;
    }

    //--------------------------------------------------------------------------
    // Name:        assess_setpoints
    // Description: Checks whether iteration should be stopped or started based
    //              on the status of the control node's setpoints. If all
    //              setpoints are disabled and the node is still iterating, the
    //              function will stop iteration. If the node is not iterating
    //              and there is at least one enabled setpoint, the function
    //              will start iteration.
    // Arguments:   - msg: Setpoint message received on the topic.
    //              - topic_name: Topic name that the message was received from.
    //--------------------------------------------------------------------------
    void assess_setpoints()
    {

        // If the node is iterating but none of the setpoints are enabled,
        // iteration should be stopped
        if (iterating && all_setpoints_disabled() && !disable_called)
        {
            log_info("all setpoints disabled, stopping iteration");
            iteration_timer.stop();
            disable();
            disable_called = true;
            iterating = false;
        }

        // If the node is not iterating but at least one of the setpoints is
        // enabled, iteration should be started. The user-defined iterate
        // function will be called once and then the iteration timer will be
        // started so that iteration starts immediately. Otherwise, a full
        // period of the timer would have to elapse before iterate got called
        else if (!iterating && !all_setpoints_disabled())
        {
            log_info("at least one setpoint is enabled, starting iteration...");
            iterate();
            iteration_timer.start();
            disable_called = false;
            iterating = true;
        }

    }

    //--------------------------------------------------------------------------
    // Name:        setpoint_msg_callback
    // Description: Called when a messages is received on a setpoint topic.
    // Arguments:   - msg: Setpoint message received on the topic.
    //              - topic_name: Topic name that the message was received from.
    //--------------------------------------------------------------------------
    template <typename T>
    void setpoint_msg_callback(const T& msg, std::string topic_name)
    {
        setpoint_map[topic_name].second = msg.enable;
        assess_setpoints();
    }

    //--------------------------------------------------------------------------
    // Name:        input_fault_callback
    // Description: Called when a fault is detected by an input monitored
    //              subscriber. Throws an exception indicating
    //              that there is an input timeout fault.
    // Arguments:   - fault: fault event structure
    //--------------------------------------------------------------------------
    template <typename T>
    void input_fault_callback(const avl::SubscriberFault& fault,
        std::string topic_name)
    {

    	if (!disable_called)
    	{

            std::string message = "input on topic " + topic_name + " timed out";
            log_error(message);

            // Call the safety node abort service
            SafetyAbortSrv msg;
            msg.request.message = message;
            abort_client.call(msg);

            // If the safety node abort fails for some reason, throw exception
            if (!msg.response.success)
                throw std::runtime_error(message);

    	}

        // Reset the subscriber so that input messages can be received again
        boost::any_cast<MonitoredSubscriber<T>>(
            &input_map.at(topic_name))->reset();

    }

    //--------------------------------------------------------------------------
    // Name:        fault_callback
    // Description: Called when a fault is detected by the monitored subscriber.
    //              Throws an exception indicating that there is an input
    //              timeout fault.
    // Arguments:   - fault: fault event structure
    //--------------------------------------------------------------------------
    template <typename T>
    void setpoint_fault_callback(const avl::SubscriberFault& fault,
        std::string topic_name)
    {
        setpoint_map[topic_name].second = false;
        boost::any_cast<MonitoredSubscriber<T>>(
            &setpoint_map.at(topic_name).first)->reset();
        assess_setpoints();
    }

    //--------------------------------------------------------------------------
    // Name:        iteration_timer_callback
    // Description: Called at the iteration rate by the iteration timer. Calls
    //              the user-defined iterate function.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void iteration_timer_callback(const ros::TimerEvent& event)
    {
        iterate();
    }

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on every iteration of the controller. This function
    //              should be implemented by the child control node class and
    //              should contain the code that calculates the controller
    //              output values and publishes them to the relevant topics.
    //--------------------------------------------------------------------------
    virtual void iterate()
    {
        log_error("iterate_callback has not been correctly implemented in "
            "this node");
    }

    //--------------------------------------------------------------------------
    // Name:        disable
    // Description: Called when the controller is no longer active and enters a
    //              disabled state. This function should be implemented by the
    //              child control node class anbd should contain controller
    //              disabling logic such as clearing integral error.
    //--------------------------------------------------------------------------
    virtual void disable()
    {
        log_error("disable_controller has not been correctly implemented in "
            "this node");
    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run() final
    {

        // Create the iterate timer (which also starts it), and stop it
        iteration_timer = node_handle->createTimer(iteration_duration,
            &ControlNode::iteration_timer_callback, this);
        iteration_timer.stop();

        // Set up the safety abort client
        abort_client = node_handle->serviceClient
            <SafetyAbortSrv>("system/abort_safety_node");

        // Log information about the control node
        log_info("================================================================================");
        log_info("Iteration rate: %.3f Hz", 1.0/iteration_duration.toSec());
        log_info("Inputs:");
        for (auto&& elem : input_map)
            log_info("    %s", elem.first.c_str());
        log_info("Setpoints:");
        for (auto&& elem : setpoint_map)
            log_info("    %s", elem.first.c_str());
        log_info("================================================================================");

        ros::spin();

    }

};

#endif //CONTROL_NODE_H
