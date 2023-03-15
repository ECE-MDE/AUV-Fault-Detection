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

#include <avl_asio/tcp_server.h>

// Boost functions
#include <boost/asio.hpp>
#include <boost/bind.hpp>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        TcpServer constructor
// Description: Default constructor.
// Arguments:   - buffer_size: Read buffer size in bytes (optional).
//------------------------------------------------------------------------------
TcpServer::TcpServer(uint64_t buffer_size) : TcpSocket(buffer_size),
    acceptor(io_service)
{

}

//------------------------------------------------------------------------------
// Name:        TcpServer destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
TcpServer::~TcpServer()
{

}

//------------------------------------------------------------------------------
// Name:        open
// Description: Opens the server on localhost at the specified port. Does
//              nothing if it already opened.
// Arguments:   - port: TCP port to open
//------------------------------------------------------------------------------
void TcpServer::open(unsigned int port)
{

    server_port = port;

    // Close the acceptor if it already open
    socket.close();

    try
    {

        // Open the acceptor with the reuse address option (SO_REUSEADDR)
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port);
        acceptor.open(endpoint.protocol());
        acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
        acceptor.bind(endpoint);
        acceptor.listen();

    }
    catch (const std::exception& ex)
    {
        acceptor.close();
        throw std::runtime_error(std::string("open: failed to open TCP server (") + ex.what() + ")");
    }

    // Accept an incoming connection
    start_accept();

}

//------------------------------------------------------------------------------
// Name:        is_open
// Description: Returns TCP server status. True if open, false if closed.
// Returns:     True if serial port open, false otherwise.
//------------------------------------------------------------------------------
bool TcpServer::is_open() const
{
    return acceptor.is_open();
}

//------------------------------------------------------------------------------
// Name:        is_connected
// Description: Returns the status of the TCP server's connection to a
//              client. True if connected to a client, false if not
//              connected.
// Returns:     True if connected, false otherwise.
//------------------------------------------------------------------------------
bool TcpServer::is_connected() const
{
    return socket.is_open();
}

//------------------------------------------------------------------------------
// Name:        close
// Description: Closes the TCP server.
//------------------------------------------------------------------------------
void TcpServer::close()
{
    acceptor.close();
    socket.close();
}

//------------------------------------------------------------------------------
// Name:        set_session_callback
// Description: Sets the session callback function that is called when a new
//              connection is accepted. The callback must have the following
//              signature:
//                  void session_callback(const TcpSocket& socket)
//              where socket is a reference to the newly accepted TCP socket
//              connection.
// Arguments:   - callback: pointer to session callback function
//------------------------------------------------------------------------------
void TcpServer::set_session_callback(std::function<void(void)> callback)
{
    session_callback = callback;
}

//------------------------------------------------------------------------------
// Name:        spin
// Description: Processes asynchronous IO operations. This function blocks
//              forever. Use the spin_once function in a loop to do the same
//              without blocking.
//------------------------------------------------------------------------------
void TcpServer::spin()
{
    io_service.run();
}

//------------------------------------------------------------------------------
// Name:        spin_once
// Description: Processes asynchronous IO operations. This function handles
//              only a small set of operations at a time, and must therefore
//              be called in a loop.
//------------------------------------------------------------------------------
void TcpServer::spin_once()
{
    io_service.poll_one();
}

//------------------------------------------------------------------------------
// Name:        start_accept
// Description: Starts an asynchronous connection accept that calls
//              accept_complete_callback when a connection is accepted.
//------------------------------------------------------------------------------
void TcpServer::start_accept()
{

    // Bind the accept complete calback
    auto accept_callback = boost::bind(&TcpServer::accept_complete_callback, this,
                                       boost::asio::placeholders::error);

    // Asynchronously accept a new connection into the socket
    acceptor.async_accept(socket, accept_callback);

}

//------------------------------------------------------------------------------
// Name:        accept_complete_callback
// Description: Callback called by the async_accept function when a
//              connection was successfully connected. Calls the TCP session
//              callback function.
//------------------------------------------------------------------------------
void TcpServer::accept_complete_callback(const boost::system::error_code& error)
{

    if (!error)
    {

        // Close the acceptor so that no more conenctions are accepted
        acceptor.close();

        // Start the socket's asynchronous read cycle. This is normally done by
        // the TcpSocket's open function, but in this case the acceptor is
        // handling the opening of the socket and does not call the open
        // function.
        start_async_read();

        // Call the session callback if one has been set
        if (session_callback != nullptr)
            session_callback();

        // When the session callback returns, the session is over and another
        // connection can be accepted

        // Re-open the acceptor to accept a new connection
        open(server_port);

    }

}
