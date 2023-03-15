//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides utility functions for match parsing, including checking
//              whether a circular buffer matches a match condition.
//==============================================================================

#ifndef MATCH_UTIL_H
#define MATCH_UTIL_H

// C++ includes
#include <string>
#include <vector>

// Boost includes
#include <boost/circular_buffer.hpp>
#include <boost/asio.hpp>
#include <boost/any.hpp>

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        buffer_to_string
// Description: Reads a specified number of characters from a streambuf as a
//              string. Does not remove the characters from the streambuf.
// Arguments:   - buffer: streambuf to read from
//              - size: number of characters to read from buffer
// Returns:     String from streambuf data.
//------------------------------------------------------------------------------
inline std::string streambuf_to_string(const boost::asio::streambuf &buffer, size_t size)
{

    return std::string(boost::asio::buffers_begin(buffer.data()),
                       boost::asio::buffers_begin(buffer.data()) + size);

}

//------------------------------------------------------------------------------
// Name:        is_string
// Description: Checks whether a boost::any is a std::string.
// Arguments:   - operand: boost::any to check
// Returns:     True if the boost::any is a std::string, false otherwise.
//------------------------------------------------------------------------------
inline bool is_string(const boost::any & operand)
{
    return boost::any_cast<std::string>(&operand);
}

//------------------------------------------------------------------------------
// Name:        is_chars
// Description: Checks whether a boost::any is a char const *.
// Arguments:   - operand: boost::any to check
// Returns:     True if the boost::any is a char const *, false otherwise.
//------------------------------------------------------------------------------
inline bool is_chars(const boost::any & operand)
{
    return boost::any_cast<char const *>(&operand);
}

//------------------------------------------------------------------------------
// Name:        is_size
// Description: Checks whether a boost::any is a size_t
// Arguments:   - operand: boost::any to check
// Returns:     True if the boost::any is a size_t, false otherwise.
//------------------------------------------------------------------------------
inline bool is_size(const boost::any & operand)
{
    return boost::any_cast<size_t>(&operand);
}

//------------------------------------------------------------------------------
// Name:        is_int
// Description: Checks whether a boost::any is an int
// Arguments:   - operand: boost::any to check
// Returns:     True if the boost::any is an int, false otherwise.
//------------------------------------------------------------------------------
inline bool is_int(const boost::any & operand)
{
    return boost::any_cast<int>(&operand);
}

//------------------------------------------------------------------------------
// Name:        is_bytes
// Description: Checks whether a boost::any is a std::vector<uint8_t>
// Arguments:   - operand: boost::any to check
// Returns:     True if the boost::any is a std::vector<uint8_t>, false
//              otherwise.
//------------------------------------------------------------------------------
inline bool is_bytes(const boost::any & operand)
{
    return boost::any_cast<std::vector<uint8_t>>(&operand);
}

//------------------------------------------------------------------------------
// Name:        ends_with
// Description: Checks whether the most recently added bytes in a circular
//              buffer (the end of the buffer) match a specified delimiter.
// Arguments:   - buffer: circular buffer to check
//              - delim: delimiter to check for
// Returns:     True if the end of the buffer matches the delimiter. False
//              otherwise.
//------------------------------------------------------------------------------
inline bool ends_with(boost::circular_buffer<uint8_t> buffer, std::string delim)
{

    bool found = false;

    // Lengths of the buffer and delimiter
    size_t buff_len = buffer.size();
    size_t delim_len = delim.size();

    // If the buffer is long enough to contain the delimiter
    if (buff_len >= delim.size())
    {

        // Compare the delimiter to the end (most recently added characters) of
        // the buffer. If compare returns 0, the end of the buffer and the
        // delimiter match
        std::string buff_end = std::string(buffer.end() - delim_len, buffer.end());
        found = (buff_end.compare(delim) == 0);

    }

    return found;

}

//------------------------------------------------------------------------------
// Name:        ends_with
// Description: Checks whether the most recently added bytes in a circular
//              buffer (the end of the buffer) match a specified delimiter.
// Arguments:   - buffer: circular buffer to check
//              - delim: delimiter to check for
// Returns:     True if the end of the buffer matches the delimiter. False
//              otherwise.
//------------------------------------------------------------------------------
inline bool ends_with(boost::circular_buffer<uint8_t> buffer, std::vector<uint8_t> delim)
{

    bool found = false;

    // Lengths of the buffer and delimiter
    size_t buff_len = buffer.size();
    size_t delim_len = delim.size();

    // If the buffer is long enough to contain the delimiter
    if (buff_len >= delim.size())
    {

        // Compare the delimiter to the end (most recently added characters) of
        // the buffer. If compare returns 0, the end of the buffer and the
        // delimiter match
        std::vector<uint8_t> buff_end = std::vector<uint8_t>(buffer.end() - delim_len, buffer.end());
        found = buff_end == delim;

    }

    return found;

}

//------------------------------------------------------------------------------
// Name:        contains
// Description: Checks whether the circular buffer contains a specified
//              delimiter.
// Arguments:   - buffer: circular buffer to check
//              - delim: delimiter to check for
//              - position: reference to size_t to store the position of the
//                delimiter in the buffer.
// Returns:     True if the circular buffer contains the delimiter. False
//              otherwise.
//------------------------------------------------------------------------------
inline bool contains(boost::circular_buffer<uint8_t> buffer, std::vector<uint8_t> delim,
                     size_t& position)
{

    // Search for the delimiter in the buffer
    auto result = std::search(buffer.begin(), buffer.end(),
                              delim.begin(),  delim.end());
    position = std::distance(buffer.begin(), result) + delim.size();
    return (result != buffer.end());

}

//------------------------------------------------------------------------------
// Name:        contains
// Description: Checks whether the circular buffer contains a specified
//              delimiter.
// Arguments:   - buffer: circular buffer to check
//              - delim: delimiter to check for
//              - position: reference to size_t to store the position of the
//                delimiter in the buffer.
// Returns:     True if the circular buffer contains the delimiter. False
//              otherwise.
//------------------------------------------------------------------------------
inline bool contains(boost::circular_buffer<uint8_t> buffer, std::string delim,
                     size_t& position)
{


    // Convert the string delimiter into an array of bytes, then use the other
    // overload of this function
    std::vector<uint8_t> delim_byte_vector(delim.begin(), delim.end());
    return ::contains(buffer, delim_byte_vector, position);

}

//------------------------------------------------------------------------------
// Name:        matches
// Description: Checks whether the buffer matches a match delimiter consisting
//              of a buffer size (int or size_t) or ending in a string delimiter
//              (std::string or char const*).
// Arguments:   - buffer: circular buffer to check for a match
//              - match_delim: match delimiter to check for (int, size_t,
//                std::string, or char const*)
//              - match_end_pos: position of the last byte that matches the
//                match delimiter
// Returns:     True if the buffer contains a match, false otherwise.
//------------------------------------------------------------------------------
inline bool matches(boost::circular_buffer<uint8_t> buffer, boost::any match_delim,
                    size_t& match_end_pos)
{

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Character array delimiter
    if (is_chars(match_delim))
    {
        std::string delim = std::string(boost::any_cast<char const*>(match_delim));
        return contains(buffer, delim, match_end_pos);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // String delimiter
    else if (is_string(match_delim))
    {
        std::string delim = boost::any_cast<std::string>(match_delim);
        return contains(buffer, delim, match_end_pos);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Integer buffer size
    else if (is_int(match_delim))
    {
        size_t size = (size_t)boost::any_cast<int>(match_delim);
        if (buffer.size() >= size)
        {
            match_end_pos = size;
            return true;
        }
        else
        {
            return false;
        }
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // size_t buffer size
    else if (is_size(match_delim))
    {
        size_t size = boost::any_cast<size_t>(match_delim);
        if (buffer.size() >= size)
        {
            match_end_pos = size;
            return true;
        }
        else
        {
            return false;
        }
        return buffer.size() >= size;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Byte array delimiter
    else if (is_bytes(match_delim))
    {
        std::vector<uint8_t> delim = boost::any_cast<std::vector<uint8_t>>(match_delim);
        return contains(buffer, delim, match_end_pos);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Unsupported delimiter
    throw std::runtime_error("matches: unsupported match delimiter type");

}

#endif // MATCH_UTIL_H
