//==============================================================================
// Autonomous Vehicle Library
//
// Description: Class for the parsing of data messages by putting it into a ring
//              buffer and checking the buffer against one or more match
//              condition. When a match condition is met, the data up to and
//              including the match condition are returned to the match
//              condition's user defined callback function.
//==============================================================================

#ifndef MATCH_PARSER_H
#define MATCH_PARSER_H

// Match condition class and utility to check whether a circular buffer matches
// a match condition
#include <avl_asio/match.h>
#include <avl_asio/match_util.h>

// AVL Packet class for matching on packets
#include <avl_core/protocol/packet.h>
using namespace avl;

// Boost circular buffer for storing read bytes
#include <boost/circular_buffer.hpp>

//==============================================================================
//                            CLASS DECLARATION
//==============================================================================

class MatchParser
{

public:

    //--------------------------------------------------------------------------
    // Name:        MatchParser constructor
    // Description: Default constructor.
    // Arguments:   - buffer_size: Match parser buffer size in bytes (optional).
    //--------------------------------------------------------------------------
    MatchParser(uint64_t buffer_size=65536) : buffer(buffer_size)
    {
        match_list.clear();
    }

    //--------------------------------------------------------------------------
    // Name:        MatchParser destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~MatchParser()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        set_match
    // Description: Sets the match condition of the parser to a single match
    //              condition.
    // Arguments:   - match: Match condition.
    //--------------------------------------------------------------------------
    void set_match(const Match& match)
    {
        match_list.clear();
        match_list.push_back(match);
    }

    //--------------------------------------------------------------------------
    // Name:        add_match
    // Description: Adds the specified match condition to the list of conditions
    //              to be checked.
    // Arguments:   - match: Match condition.
    //--------------------------------------------------------------------------
    void add_match(const Match& match)
    {
        match_list.push_back(match);
    }

    //--------------------------------------------------------------------------
    // Name:        set_packet_callback
    // Description: Sets a packet callback function to be called when an AVL
    //              packet is read.
    // Arguments:   - callback: Pointer to packet callback function.
    //--------------------------------------------------------------------------
    void set_packet_callback(std::function<void(Packet)> callback)
    {
        packet_callback = callback;
    }

    //--------------------------------------------------------------------------
    // Name:        set_packet_callback
    // Description: Sets a packet callback function to be called when an AVL
    //              packet is read.
    // Arguments:   - callback: Pointer to packet callback function.
    //--------------------------------------------------------------------------
    template<class T>
    void set_packet_callback(void (T::*callback)(Packet), T* obj)
    {
        packet_callback = std::bind(callback, obj, std::placeholders::_1);
    }

    //--------------------------------------------------------------------------
    // Name:        set_match
    // Description: Sets the match condition of the parser to a list of match
    //              conditions.
    // Arguments:   - matches: Vector of match conditions.
    //--------------------------------------------------------------------------
    void set_match(const std::vector<Match>& matches)
    {
        match_list = matches;
    }

    //--------------------------------------------------------------------------
    // Name:        clear_matches
    // Description: Deletes all matches from the list of match conditions.
    //--------------------------------------------------------------------------
    void clear_matches()
    {
        match_list.clear();
    }

    //--------------------------------------------------------------------------
    // Name:        process_bytes
    // Description: Pushes multiple bytes into the circular buffer and checks
    //              whether the buffer matches any of the match conditions.
    //              If it does, the data is sent to the match callback, and
    //              the matching bytes are removed from the buffer.
    // Arguments:   - bytes: Vector of bytes to add to the buffer.
    //--------------------------------------------------------------------------
    void process_bytes(std::vector<uint8_t> bytes)
    {

        // Add the given number of the bytes to the circular buffer
        buffer.insert(buffer.end(), bytes.begin(),
            bytes.begin() + bytes.size());

        // Look through the buffer for AVL packets. Finish when there are no
        // more packets found
        if (packet_callback != nullptr)
        {

            bool found_packet = false;
            do
            {

                // Convert the circular buffer to a vector of bytes for use with
                // the packet class's contains_packet function
                std::vector<uint8_t> bytes =
                    std::vector<uint8_t>(buffer.begin(), buffer.end());

                // Locate any packets in the buffer
                size_t i_start, i_end;
                found_packet = Packet::contains_packet(bytes, i_start, i_end);
                if (found_packet)
                {

                    // Get the bytes forming the packet
                    std::vector<uint8_t> data(buffer.begin() + i_start,
                                              buffer.begin() + i_end);

                    // Erase the used bytes from the buffer
                    buffer.erase(buffer.begin(), buffer.begin() + i_end);

                    // Call the user-defined read callback and pass it the
                    // packet
                    packet_callback(Packet(data));

                }

            }
            while (found_packet);

        }

        // Look through the buffer for matching match conditions from our list
        // of match conditions. Finish when there are no more matches found
        bool found_match = false;
        do
        {

            // Loop through and check the match conditions in the match list
            for (size_t i = 0; i < match_list.size(); i++)
            {

                // If the buffer matches the delimiter
                size_t match_end;
                found_match = matches(buffer, match_list.at(i).delimiter,
                    match_end);

                if (found_match)
                {

                    // Get the bytes up to the end of the match
                    std::vector<uint8_t> data(buffer.begin(),
                                              buffer.begin() + match_end);

                    // Erase the used bytes from the buffer
                    buffer.erase(buffer.begin(),
                                 buffer.begin() + match_end);

                    // Call the user-defined read callback from the match with
                    // the read data
                    match_list.at(i).callback(data);

                }

            }

        }
        while (found_match);

    }

private:

    // Buffer to store read data
    boost::circular_buffer<uint8_t> buffer;

    // List of match conditions to match the buffer against
    std::vector<Match> match_list;

    // Handler for AVL packets
    std::function<void(Packet)> packet_callback = nullptr;

};

#endif // MATCH_PARSER_H
