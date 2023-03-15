//==============================================================================
// Autonomous Vehicle Library
//
// Description: A ROS subscriber implementing various checks of the validity of
//              incoming messages. These checks include:
//
//                  - rate check: messages must be received at least as fast as
//                    a given rate in Hz
//
//                  - out of bounds check: messages are evaluated using a user
//                    specified function that determines whether a message is
//                    considered out of bounds
//
//                  - reptition check: messages are evaluated using a user
//                    specified funcction that determines whether a message is
//                    considered to be a repeat of the previous message
//
//              If a fault condition is detected, a structure containing
//              information about the fault is sent to the user defined fault
//              callback.
//==============================================================================

#ifndef MONITORED_SUBSCRIBER_H
#define MONITORED_SUBSCRIBER_H

// ROS includes
#include "ros/ros.h"

// Number of seconds to wait for the first message if rate checking is enabled.
// This allows monitored subscribers to not time out when ROS is starting up
// slowly
#define GRACE_PERIOD_DURATION 5.0

//==============================================================================
//                              FAULT STRUCTURES
//==============================================================================

namespace avl
{

// Enum describing the types of faults that a monitored subscriber can detect
typedef enum SubscriberFaultType
{

    TIMEOUT_FAULT,
    OUT_OF_BOUNDS_FAULT,
    REPEATED_MESSAGE_FAULT

} SubscriberFaultType;

// Structure containing information about a fault detected by the monitored
// subscriber
typedef struct SubscriberFault
{

    SubscriberFaultType type;
    std::string topic;

} SubscriberFault;

}

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================
template<typename MessageType>
class MonitoredSubscriber
{

public:

    //--------------------------------------------------------------------------
    // Name:        MonitoredSubscriber constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    MonitoredSubscriber()
    {

        // Default out of bounds and repeated message functions that always
        // return true
        out_of_bounds_function = [](MessageType) { return true; };
        repeated_message_function =
            [](MessageType, MessageType) { return true; };

    }

    //--------------------------------------------------------------------------
    // Name:        MonitoredSubscriber copy constructor
    // Description: Copy constructor.
    //--------------------------------------------------------------------------
    MonitoredSubscriber(const MonitoredSubscriber &sub)
    {

        // Copy all member variables except the timer and subscriber
        nh = sub.nh;
        topic_name = sub.topic_name;
        queue_size = sub.queue_size;
        timeout_timer_duration = sub.timeout_timer_duration;
        message_rate_check_enabled = sub.message_rate_check_enabled;
        out_of_bounds_check_enabled = sub.out_of_bounds_check_enabled;
        repeated_message_check_enabled = sub.repeated_message_check_enabled;
        message_callback = sub.message_callback;
        fault_callback = sub.fault_callback;
        out_of_bounds_function = sub.out_of_bounds_function;
        repeated_message_function = sub.repeated_message_function;
        previous_message = sub.previous_message;
        fault_detected = sub.fault_detected;

        // Call subscribe again to generate the timer and subscriber. This is
        // necessary because if we simply copy the timer and subscriber from
        // the other instance then the timer callback and message callback will
        // still be pointing to the other instance's callback functions. If
        // the other instance is then destructed, we now have callbacks
        // that no longer exist. This appears to be a quirk of ROS's Timer and
        // Subscriber class copying.
        subscribe(topic_name, queue_size, message_callback, fault_callback);

    }

    //--------------------------------------------------------------------------
    // Name:        MonitoredSubscriber destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~MonitoredSubscriber()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        set_message_rate
    // Description: Sets the message rate in Hz. Messages must be received at
    //              faster than this rate to prevent a message rate fault.
    // Arguments:   - rate: message rate in Hz
    //--------------------------------------------------------------------------
    void set_message_rate(double rate)
    {
        timeout_timer_duration = ros::Duration(1.0/rate);
    }

    //--------------------------------------------------------------------------
    // Name:        set_out_of_bounds_function
    // Description: Sets the function used to determine whether a message is
    //              out of bounds. The function must have the following
    //              signature:
    //                  bool function_name(const MessageType& message);
    //              The function should return true if the message is considered
    //              out of bounds, and false if it is not.
    // Arguments:   - function: pointer to function that evaluates a message for
    //                an out of bounds condition
    //--------------------------------------------------------------------------
    void set_out_of_bounds_function(std::function<bool(MessageType)> function)
    {
        out_of_bounds_function = function;
    }

    //--------------------------------------------------------------------------
    // Name:        set_repeated_message_function
    // Description: Sets the function used to determine whether a message is
    //              repeated. The function must have the following signature:
    //                  bool function_name(const MessageType& previous_message,
    //                      const MessageType& message);
    //              The function should return true if message is considered
    //              a repeat of previous_message, and false if it is not.
    // Arguments:   - function: pointer to function that evaluates a message for
    //                a repeat condition
    //--------------------------------------------------------------------------
    void set_repeated_message_function(std::function<bool(const MessageType&,
        const MessageType&)> function)
    {
        repeated_message_function = function;
    }

    //--------------------------------------------------------------------------
    // Name:        enable_message_rate_check
    // Description: Enables or disables message rate checking.
    // Arguments:   - enable: true to enable, false to disable
    //--------------------------------------------------------------------------
    void enable_message_rate_check(bool enable)
    {
        message_rate_check_enabled = enable;
    }

    //--------------------------------------------------------------------------
    // Name:        enable_out_of_bounds_check
    // Description: Enables or disables message out of bounds checking.
    // Arguments:   - enable: true to enable, false to disable
    //--------------------------------------------------------------------------
    void enable_out_of_bounds_check(bool enable)
    {
        out_of_bounds_check_enabled = enable;
    }

    //--------------------------------------------------------------------------
    // Name:        enable_repitition_check
    // Description: Enables or disables message repitition checking.
    // Arguments:   - enable: true to enable, false to disable
    //--------------------------------------------------------------------------
    void enable_repitition_check(bool enable)
    {
        repeated_message_check_enabled = enable;
    }

    //--------------------------------------------------------------------------
    // Name:        get_last_message
    // Description: Gets the most recent message that was received on the topic.
    // Returns:     Most recently received message.
    //--------------------------------------------------------------------------
    MessageType get_last_message()
    {
        return previous_message;
    }

    //--------------------------------------------------------------------------
    // Name:        reset
    // Description: Resets the monitored subscriber's fault status, allowing
    //              new messages to be evaluated for faults and passed to the
    //              message callback.
    //--------------------------------------------------------------------------
    void reset()
    {

        fault_detected = false;
        timeout_timer.setPeriod(timeout_timer_duration);
        timeout_timer.start();

    }

    //--------------------------------------------------------------------------
    // Name:        subscribe
    // Description: Subscribes to a topic to be monitored.
    // Arguments:   - topic: Topic name to subscribe to.
    //              - queue_size: number of messages to queue.
    //              - message_cb: Pointer to callback to be called when a
    //                message is received.
    //              - fault_fp: Pointer to callback to be called when a fault
    //                is detected.
    //--------------------------------------------------------------------------
    void subscribe(const std::string &topic, uint32_t queue_size,
        std::function<void(const MessageType&)> message_cb,
        std::function<void(const avl::SubscriberFault&)> fault_cb)
    {

        topic_name = topic;
        message_callback = message_cb;
        fault_callback = fault_cb;

        // Create the timer for message rate checking, which also starts
        // the timer
        timeout_timer = nh.createTimer(ros::Duration(GRACE_PERIOD_DURATION),
            &MonitoredSubscriber::timer_callback, this);

        // Set up the subscriber to call the intermediate message callback that
        // will check the message for faults, and then either call the fault
        // callback or forward the message to the user's message callback
        subscriber = nh.subscribe(topic, queue_size,
            &MonitoredSubscriber::intermediate_msg_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        subscribe
    // Description: Subscribes to a topic to be monitored. Uses class member
    //              functions as the message and fault callbacks.
    // Arguments:   - topic: Topic name to subscribe to.
    //              - queue_size: Number of messages to queue.
    //              - message_fp: Pointer to callback member function to pass
    //                received messaged to.
    //              - fault_fp: Pointer to callback member function to pass
    //                fault information to.
    //              - obj: Instance of class in which the member function is
    //                located.
    //--------------------------------------------------------------------------
    template<class T>
    void subscribe(const std::string &topic, uint32_t queue_size,
        void(T::*message_fp)(const MessageType&),
        void(T::*fault_fp)(const avl::SubscriberFault&), T *obj)
    {
        auto message_cb = std::bind(message_fp, obj, std::placeholders::_1);
        auto fault_cb = std::bind(fault_fp, obj, std::placeholders::_1);
        subscribe(topic, queue_size, message_cb, fault_cb);
    }

    //--------------------------------------------------------------------------
    // Name:        subscribe
    // Description: Subscribes to a topic to be monitored when a message
    //              callback is not needed. Uses a class member function as the
    //              fault callback.
    // Arguments:   - topic: Topic name to subscribe to.
    //              - queue_size: Number of messages to queue.
    //              - fault_fp: Pointer to callback member function to pass
    //                fault information to.
    //              - obj: Instance of class in which the member function is
    //                located.
    //--------------------------------------------------------------------------
    template<class T>
    void subscribe(const std::string &topic, uint32_t queue_size,
        void(T::*fault_fp)(const avl::SubscriberFault&), T *obj)
    {
        auto fault_cb = std::bind(fault_fp, obj, std::placeholders::_1);
        subscribe(topic, queue_size, nullptr, fault_cb);
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Shutdown a monitored subscription
    // Arguments:   - None
    //--------------------------------------------------------------------------
    void shutdown()
    {
        subscriber.shutdown();
    }

private:

    // Node handle for subscribing
    ros::NodeHandle nh;

    // Regular ROS subscriber used for subscribing to the topic
    ros::Subscriber subscriber;
    std::string topic_name;
    uint32_t queue_size;

    // Timeout timer and its associated duration. If the timer times out, then
    // that means messages we're not received faster than the specified rate
    ros::Timer timeout_timer;
    ros::Duration timeout_timer_duration = ros::Duration(1.0);

    // Flags for enablingdisabling fault checking types
    // bool message_rate_check_enabled = false;
    // bool out_of_bounds_check_enabled = false;
    // bool repeated_message_check_enabled = false;
    bool message_rate_check_enabled = false;
    bool out_of_bounds_check_enabled = false;
    bool repeated_message_check_enabled = false;

    // User specified message and fault callbacks
    std::function<void(MessageType)> message_callback;
    std::function<void(const avl::SubscriberFault&)> fault_callback;

    // User defined functions that return a bool indicating whether the message
    // is out of bounds or is a repeat of the previous message. We need these
    // because the message types can be anything, and this class has no way of
    // knowing what it means for a message to be out of bounds, or what a
    // repeated message looks like. They must be user defined
    std::function<bool(MessageType)> out_of_bounds_function;
    std::function<bool(MessageType, MessageType)> repeated_message_function;

    // A copy of the last message received to it can be passed to the user to
    // compare to the newest message for the repeated message check
    MessageType previous_message;

    // Flag indicating whether a fault was detected and the fault callback was
    // called. If this is true, further fault callbacks will not be generated
    // and messages will not be passed to the user's message callback
    volatile bool fault_detected = false;

private:

    //--------------------------------------------------------------------------
    // Name:        timer_callback
    // Description: Called when the timeout timer expires.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void timer_callback(const ros::TimerEvent& event)
    {

        if (message_rate_check_enabled && !fault_detected)
        {
            fault_detected = true;
            timeout_timer.stop();
            avl::SubscriberFault fault;
            fault.type = avl::TIMEOUT_FAULT;
            fault.topic = topic_name;
            fault_callback(fault);
        }

    }

    //--------------------------------------------------------------------------
    // Name:        intermediate_msg_callback
    // Description: Called when a message is received by the subscriber.
    // Arguments:   - message: message received by the subscriber
    //--------------------------------------------------------------------------
    void intermediate_msg_callback(const MessageType& message)
    {

        // If a fault was already detected, we don't need to check for more or
        // pass the message to the user
        if (!fault_detected)
        {

            // Check the messages against the fault criteria
            if (out_of_bounds_function(message) &&
                out_of_bounds_check_enabled)
            {

                timeout_timer.stop();
                fault_detected = true;
                avl::SubscriberFault fault;
                fault.type = avl::OUT_OF_BOUNDS_FAULT;
                fault.topic = topic_name;
                fault_callback(fault);

            }
            else if (repeated_message_function(previous_message, message) &&
                     repeated_message_check_enabled)
            {

                timeout_timer.stop();
                fault_detected = true;
                previous_message = MessageType();
                avl::SubscriberFault fault;
                fault.type = avl::REPEATED_MESSAGE_FAULT;
                fault.topic = topic_name;
                fault_callback(fault);

            }
            else
            {

                // Reset the timeout timer by setting its duration again. This
                // cancels the timer, so it must be started again
                timeout_timer.setPeriod(timeout_timer_duration);
                timeout_timer.start();

                // Save the new message as the previous message for the next
                // iteration of fault checks
                previous_message = message;

                // Pass the message to the callback specified by the user
                if (message_callback != nullptr)
                    message_callback(message);

            }

        }

    }

};

#endif // MONITORED_SUBSCRIBER_H
