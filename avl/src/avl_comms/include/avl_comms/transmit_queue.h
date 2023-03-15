//==============================================================================
// Autonomous Vehicle Library
//
// Description:
//==============================================================================

#ifndef TRANSMIT_QUEUE_H
#define TRANSMIT_QUEUE_H

// C++ includes
#include <queue>
#include <utility>

// Byte array and message typedefs
typedef std::vector<uint8_t> bytes_t;
typedef std::pair<int, bytes_t> message_t;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

struct MessageCompare
{

    //--------------------------------------------------------------------------
    // Name:        operator()
    // Description: Comparison operator for comparison of message priority.
    // Arguments:   - msg1: first mesasge
    //              - msg2: second message
    // Returns:     True if the task queue is empty, false if it
    //              contains one or more message.
    //--------------------------------------------------------------------------
    bool operator()(const message_t& msg1, const message_t& msg2)
    {
        return msg1.first > msg2.first;
    }

};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class TransmitQueue
{

public:

    //--------------------------------------------------------------------------
    // Name:        TransmitQueue constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    TransmitQueue()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        TransmitQueue destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~TransmitQueue()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        top
    // Description: Accesses the top message in the queue. Does not remove it
    //              from the queue.
    // Returns:     Top message from the queue.
    //--------------------------------------------------------------------------
    bytes_t top()
    {
        if (!heartbeat.empty())
            return heartbeat;
        else
            return queue.top().second;
    }

    //--------------------------------------------------------------------------
    // Name:        empty
    // Description: Returns true if the queue is empty, and false if it
    //              contains one or more message.
    // Returns:     True if the task queue is empty, false if it
    //              contains one or more message.
    //--------------------------------------------------------------------------
    bool empty()
    {
        return queue.empty() && heartbeat.empty();
    }

    //--------------------------------------------------------------------------
    // Name:        has_heartbeat
    // Description: Returns true if the queue has heartbeat data, and false if
    //              it does not.
    // Returns:     True if the queue has heartbeat data, and false if
    //              it does not.
    //--------------------------------------------------------------------------
    bool has_heartbeat()
    {
        return !heartbeat.empty();
    }

    //--------------------------------------------------------------------------
    // Name:        size
    // Description: Returns the number of messages contained in the queue.
    // Returns:     The size of the queue.
    //--------------------------------------------------------------------------
    size_t size()
    {
        if (!heartbeat.empty())
            return queue.size() + 1;
        else
            return queue.size();
    }

    //--------------------------------------------------------------------------
    // Name:        set_heartbeat
    // Description: Sets the heartbeat data that is always at the top of the
    //              queue. If there is already heartbeat data in the queue, it
    //              will be overwritten.
    // Arguments:   - data: Heartbeat data.
    //--------------------------------------------------------------------------
    void set_heartbeat(bytes_t data)
    {
        heartbeat = data;
    }

    //--------------------------------------------------------------------------
    // Name:        push
    // Description: Pushes a message with a given priority into the queue.
    // Arguments:   - priority: message priority level
    //              - message: mesasge to be pushed into the queue
    //--------------------------------------------------------------------------
    void push(int priority, bytes_t message)
    {
        queue.push(std::make_pair(priority, message));
    }

    //--------------------------------------------------------------------------
    // Name:        pop
    // Description: Removes the top message from the queue.
    //--------------------------------------------------------------------------
    void pop()
    {
        if (!heartbeat.empty())
            heartbeat.clear();
        else
            queue.pop();
    }

private:

    // The priority queue holding all messages to be transmitted
    std::priority_queue<message_t, std::vector<message_t>, MessageCompare> queue;

    // Contains a heartbeat message that will always be returned first when
    // reading from the queue
    bytes_t heartbeat;

};

#endif // TRANSMIT_QUEUE_H
