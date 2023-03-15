//==============================================================================
// Autonomous Vehicle Library
//
// PURPOSE: Simple timeout exception class derived from std::exception. Used
//          when an operation, such as a read/write or a connection, times out.
//
// AUTHOR:  stkrauss
//
// REVIEWED: stkrauss, babiggs
//==============================================================================

#ifndef TIMEOUT_EXCEPTION_H
#define TIMEOUT_EXCEPTION_H

// Standard exception base class
#include <stdexcept>

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class TimeoutException : public std::exception
{

public:

    virtual const char* what() const throw()
    {
        return "operation timed out";
    }

};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class ConnectTimeoutException : public TimeoutException
{

public:

    virtual const char* what() const throw()
    {
        return "connect timed out";
    }

};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class ReadTimeoutException : public TimeoutException
{

public:

    virtual const char* what() const throw()
    {
        return "read timed out";
    }

};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class WriteTimeoutException : public TimeoutException
{

public:

    virtual const char* what() const throw()
    {
        return "write timed out";
    }

};

#endif  // TIMEOUT_EXCEPTION_H
