//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides basic TCP server functionality. Listens for and accepts
//              one connection at a time on a specified port. When a connection
//              is accepted, a user defined TCP session callback is called with
//              a reference to the TCP socket for reading and writing. The
//              server will not accept another connection until the previos
//              connection ends.
//==============================================================================

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <avl_asio/tcp_socket.h>

// Boost functions
#include <boost/asio.hpp>
#include <boost/bind.hpp>

// Custom exception types
#include <avl_asio/timeout_exception.h>
#include <avl_asio/connection_closed_exception.h>

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class TcpServer : public TcpSocket
{

public:

    //--------------------------------------------------------------------------
    // Name:        TcpServer constructor
    // Description: Default constructor.
    // Arguments:   - buffer_size: Read buffer size in bytes (optional).
    //--------------------------------------------------------------------------
    TcpServer(uint64_t buffer_size=65536);

    //--------------------------------------------------------------------------
    // Name:        TcpServer destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~TcpServer();

    //--------------------------------------------------------------------------
    // Name:        open
    // Description: Opens the server on localhost at the specified port. Does
    //              nothing if it already opened.
    // Arguments:   - port: TCP port to open
    //--------------------------------------------------------------------------
    void open(unsigned int port);

    //--------------------------------------------------------------------------
    // Name:        is_open
    // Description: Returns TCP server status. True if open, false if closed.
    // Returns:     True if open, false otherwise.
    //--------------------------------------------------------------------------
    bool is_open() const;

    //--------------------------------------------------------------------------
    // Name:        is_connected
    // Description: Returns the status of the TCP server's connection to a
    //              client. True if connected to a client, false if not
    //              connected.
    // Returns:     True if connected, false otherwise.
    //--------------------------------------------------------------------------
    bool is_connected() const;

    //--------------------------------------------------------------------------
    // Name:        close
    // Description: Closes the TCP server.
    //--------------------------------------------------------------------------
    void close();

    //--------------------------------------------------------------------------
    // Name:        set_session_callback
    // Description: Sets the session callback function that is called when a new
    //              connection is accepted. The callback must have the following
    //              signature:
    //                  void session_callback(const TcpSocket& socket)
    //              where socket is a reference to the newly accepted TCP socket
    //              connection.
    // Arguments:   - callback: pointer to session callback function
    //--------------------------------------------------------------------------
    void set_session_callback(std::function<void(void)> callback);

    //--------------------------------------------------------------------------
    // Name:        set_session_callback
    // Description: Sets the session callback function that is called when a new
    //              connection is accepted. The callback must have the following
    //              signature:
    //                  void session_callback(const TcpSocket& socket)
    //              where socket is a reference to the newly accepted TCP socket
    //              connection. This overload is used for class member callback
    //              functions.
    // Arguments:   - handler: pointer to session callback member function
    //              - obj: instance of the object
    //--------------------------------------------------------------------------
    template<class T>
    void set_session_callback(void (T::*handler)(void), T* obj)
    {
        session_callback = std::bind(handler, obj);
    }

    //--------------------------------------------------------------------------
    // Name:        spin
    // Description: Processes asynchronous IO operations. This function blocks
    //              forever. Use the spin_once function in a loop to do the same
    //              without blocking.
    //--------------------------------------------------------------------------
    void spin();

    //--------------------------------------------------------------------------
    // Name:        spin_once
    // Description: Processes asynchronous IO operations. This function handles
    //              only a small set of operations at a time, and must therefore
    //              be called in a loop.
    //--------------------------------------------------------------------------
    void spin_once();

private:

    // Port that server will be opened on
    unsigned int server_port;

    // Acceptor used to listen for and accept incoming connections
    boost::asio::ip::tcp::acceptor acceptor;

    // The callback called by each session to handle session functionality
    std::function<void(void)> session_callback;

private:

    //--------------------------------------------------------------------------
    // Name:        start_accept
    // Description: Starts an asynchronous connection accept that calls
    //              accept_complete_callback when a connection is accepted.
    //--------------------------------------------------------------------------
    void start_accept();

    //--------------------------------------------------------------------------
    // Name:        accept_complete_callback
    // Description: Callback called by the async_accept function when a
    //              connection was successfully connected. Calls the TCP session
    //              callback function.
    //--------------------------------------------------------------------------
    void accept_complete_callback(const boost::system::error_code& e);

};

#endif // TCP_SERVER_H
