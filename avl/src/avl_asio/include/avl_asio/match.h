//==============================================================================
// Autonomous Vehicle Library
//
// Description: Class representing a match condition. A match condition consists
//              of a delimiter to match against, and the read callback to be
//              called when the match is successful.
//
//          Examples:
//
//          Match({0xFF, 0x65}, callback) <-- match against a series of bytes
//          Match("\n", callback)         <-- Match against characters
//          Match("abcd", callback)       <-- Match against characters
//          Match(5, callback)            <-- Match against a number of bytes
//==============================================================================

#ifndef MATCH_H
#define MATCH_H

// Boost any type to store multiple types of match conditions
#include <boost/any.hpp>

// C++ includes
#include <functional>
#include <vector>
#include <string>

typedef std::vector<uint8_t> bytes_t;
typedef std::function<void(bytes_t)> match_callback_t;

namespace ph = std::placeholders;

//==============================================================================
//                            CLASS DECLARATION
//==============================================================================

class Match
{

public:

    // Delimiter to match against
    boost::any delimiter;

    // Read callback to be called when the match is successful
    //std::function<void(std::string)> callback;
    std::function<void(std::vector<uint8_t>)> callback;

public:

    //--------------------------------------------------------------------------
    // Name:        Match constructors
    // Description: Constuctors for match conditions with a standard callback
    //              function.
    //--------------------------------------------------------------------------

    // Delimiter is vector of bytes
    Match(bytes_t delim, match_callback_t cb) :
        delimiter(delim), callback(cb) { }

    // Delimiter is initializer list of bytes
    Match(std::initializer_list<uint8_t> delim, match_callback_t cb) :
        delimiter(delim), callback(cb) { }

    // Delimiter is char literal
    Match(const char delim, match_callback_t cb) :
        delimiter(delim), callback(cb) { }

    // Delimiter is string literal
    Match(const char* delim, match_callback_t cb) :
        delimiter(delim), callback(cb) { }

    // Delimiter is number of bytes as int
    Match(int size, match_callback_t cb) :
        delimiter(size), callback(cb) { }

    // Delimiter is number of bytes as size_t
    Match(size_t size, match_callback_t cb) :
        delimiter(size), callback(cb) { }

    //--------------------------------------------------------------------------
    // Name:        Match constructors
    // Description: Constuctors for match conditions with a class member
    //              function callback.
    //--------------------------------------------------------------------------

    // Delimiter is vector of bytes
    template<class T>
    Match(bytes_t delim, void (T::*cb)(bytes_t), T* obj) :
        delimiter(delim), callback(std::bind(cb, obj, ph::_1)) { }

    // Delimiter is initializer list of bytes
    template<class T>
    Match(std::initializer_list<uint8_t> delim, void (T::*cb)(bytes_t), T* obj) :
        delimiter(std::vector<uint8_t>(delim)), callback(std::bind(cb, obj, ph::_1)) { }

    // Delimiter is char literal
    template<class T>
    Match(const char delim, void (T::*cb)(bytes_t), T* obj) :
        delimiter(std::string(1, delim)), callback(std::bind(cb, obj, ph::_1)) { }

    // Delimiter is string literal
    template<class T>
    Match(const char* delim, void (T::*cb)(bytes_t), T* obj) :
        delimiter(delim), callback(std::bind(cb, obj, ph::_1)) { }

    // Delimiter is number of bytes as int
    template<class T>
    Match(int size, void (T::*cb)(bytes_t), T* obj) :
        delimiter(size), callback(std::bind(cb, obj, ph::_1)) { }

    // Delimiter is number of bytes as size_t
    template<class T>
    Match(size_t size, void (T::*cb)(bytes_t), T* obj) :
        delimiter(size), callback(std::bind(cb, obj, ph::_1)) { }

};

#endif // MATCH_H
