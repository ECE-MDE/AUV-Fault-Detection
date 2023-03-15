//==============================================================================
// Autonomous Vehicle Library
//
// PURPOSE: Simple connection closed exception class derived from
//          std::exception. Used when some sort of connection, such as a socket
//          or serial connection, is closed either intentionally or
//          unintentionally.
//
// AUTHOR:  stkrauss
//
// REVIEWED: stkrauss, babiggs
//==============================================================================

#ifndef CONNECTION_CLOSED_EXCEPTION_H
#define CONNECTION_CLOSED_EXCEPTION_H

// Standard exception base class
#include <stdexcept>

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class ConnectionClosedException : public std::exception
{

public:

    virtual const char* what() const throw()
    {
        return "connection closed";
    }

};

#endif  // CONNECTION_CLOSED_EXCEPTION_H
