//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides basic functions to configure, read from, and write to a
//              TCP socket.
//
//              Reading is done asynchronously by setting a match condition and
//              a corresponding read callback function. Writing is done
//              synchronously, and blocks until the message is written.
//
//              Either the spin or spin_once function must be called to enable
//              functionality. The spin function blocks indefinitely processing
//              operations, while the spin_once function will not block.
//              However, the spin_once function will only process a small set of
//              operations and must therefore be called in a loop.
//
//              A read timeout can also be set, causing the spin and spin_once
//              functions to throw a TimeoutException if no data is read within
//              the timeout duration.
//==============================================================================

#include <avl_asio/tcp_socket.h>

// Match parser base class
#include <avl_asio/match_parser.h>

// Boost functions
#include <boost/asio.hpp>
#include <boost/bind.hpp>

// Custom exception types
#include <avl_asio/timeout_exception.h>
#include <avl_asio/connection_closed_exception.h>

#include <iostream>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        TcpSocket constructor
// Description: Default constructor.
// Arguments:   - buffer_size: Read buffer size in bytes.
//------------------------------------------------------------------------------
TcpSocket::TcpSocket(uint64_t buffer_size) : MatchParser(buffer_size),
	io_service(), socket(io_service), connect_timer(io_service),
    read_timer(io_service), write_timer(io_service)
{

    // No deadline is required until the first socket operation is started. We
    // set the deadline to positive infinity so that the actor takes no action
    // until a specific deadline is set.
    connect_timer.expires_at(boost::posix_time::pos_infin);
    read_timer.expires_at(boost::posix_time::pos_infin);
    write_timer.expires_at(boost::posix_time::pos_infin);

    // Start the persistent actor that checks for deadline expiry.
    check_connect_deadline();
    check_read_deadline();
    check_write_deadline();

}

//------------------------------------------------------------------------------
// Name:        TcpSocket destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
TcpSocket::~TcpSocket()
{
    socket.close();
}

//------------------------------------------------------------------------------
// Name:        connect
// Description: Connects to the specified address and port. If the TCP
//              socket is already connected, it will be closed and
//              re-connected with the specified address and port.
// Arguments:   - address: TCP address to connect to.
//              - port: TCP port to connect to.
//------------------------------------------------------------------------------
void TcpSocket::connect(const std::string& address, unsigned int port,
    unsigned int timeout)
{

	// Close the socket if it's already open
	socket.close();

    // Resolve the address with DNS in case it is a hostname and not an
    // IP address. This allows for connections to addresses like www.google.com
    using namespace boost::asio::ip;
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(address.c_str(), std::to_string(port));
    tcp::resolver::iterator iter = resolver.resolve(query);

    // Set a deadline for the asynchronous operation. As a host name may
    // resolve to multiple endpoints, this function uses the composed operation
    // async_connect. The deadline applies to the entire operation, rather than
    // individual connection attempts.
    if (timeout == 0)
        connect_timer.expires_from_now(boost::posix_time::pos_infin);
    else
        connect_timer.expires_from_now(boost::posix_time::milliseconds(timeout));

    // Set up the variable that receives the result of the asynchronous
    // operation. The error code is set to would_block to signal that the
    // operation is incomplete. Asio guarantees that its asynchronous
    // operations will never fail with would_block, so any other value in
    // ec indicates completion.
    boost::system::error_code error = boost::asio::error::would_block;

    // Start the asynchronous operation itself. The boost::lambda function
    // object is used as a callback and will update the ec variable when the
    // operation completes. The blocking_udp_client.cpp example shows how you
    // can use boost::bind rather than boost::lambda.
    boost::asio::async_connect(socket, iter,
        boost::lambda::var(error) = boost::lambda::_1);

    // Block until the asynchronous operation has completed.
    do io_service.run_one(); while (error == boost::asio::error::would_block);

    // Determine whether a connection was successfully established. The
    // deadline actor may have had a chance to run and close our socket, even
    // though the connect operation notionally succeeded. Therefore we must
    // check whether the socket is still open before deciding if we succeeded
    // or failed.
    if (error || !socket.is_open())
        throw std::runtime_error("connect: failed to connect TCP socket (" +
          error.message() + ")");

    // Cancel the connect timeout timer
    connect_timer.expires_from_now(boost::posix_time::pos_infin);

    // Start the asynchronous reading loop that reads bytes into the match
    // parser
    start_async_read();

}

//------------------------------------------------------------------------------
// Name:        is_connected
// Description: Returns TCP socket status. True if connected, false if
//              not connected.
// Returns:     True if open, false otherwise.
//------------------------------------------------------------------------------
bool TcpSocket::is_connected() const
{
    return socket.is_open();
}

//------------------------------------------------------------------------------
// Name:        close
// Description: Closes the TCP socket.
//------------------------------------------------------------------------------
void TcpSocket::close()
{
    connect_timer.cancel();
    read_timer.cancel();
    write_timer.cancel();
    socket.close();
}

//------------------------------------------------------------------------------
// Name:        set_read_timeout
// Description: Sets the read timeout duration. If the time between each
//              data byte read (if any) exceeds the timeout duration, a
//              TimeoutException will be thrown by the spin or spin_once
//              functions. A timeout of zero disables the read timeout. The
//              serial port must be re-opened after a timeout in order to be
//              used again.
// Arguments:   - timeout: Timeout duration in milliseconds.
//------------------------------------------------------------------------------
void TcpSocket::set_read_timeout(int timeout)
{

	// A timeout of zero correspondes to no timeout
    if (timeout == 0)
        read_timeout_duration = boost::posix_time::pos_infin;
    else
        read_timeout_duration = boost::posix_time::milliseconds(timeout);

    // Start the read timer
    read_timer.expires_from_now(read_timeout_duration);

}

//------------------------------------------------------------------------------
// Name:        write
// Description: Writes an array of bytes to the TCP socket with an optional
//              timeout.
// Arguments:   - data: Pointer to bytes to be written.
//              - num_bytes: Number of bytes to write.
//              - timeout: Timeout duration in milliseconds. A timeout of
//                zero disables the read timeout.
//------------------------------------------------------------------------------
void TcpSocket::write(const uint8_t* data, size_t num_bytes,
    unsigned int timeout)
{

    // Check that the socket is connected
    if (!socket.is_open())
        throw std::runtime_error("write: socket is not connected ");

    // Create a buffer from the with the bytes
	auto buf = boost::asio::buffer(data, num_bytes);

    // Set a deadline for the asynchronous operation. Since this function uses
    // a composed operation (async_write), the deadline applies to the entire
    // operation, rather than individual writes to the socket.
    if (timeout == 0)
        write_timer.expires_from_now(boost::posix_time::pos_infin);
    else
        write_timer.expires_from_now(boost::posix_time::milliseconds(timeout));

    // Set up the variable that receives the result of the asynchronous
    // operation. The error code is set to would_block to signal that the
    // operation is incomplete. Asio guarantees that its asynchronous
    // operations will never fail with would_block, so any other value in
    // ec indicates completion.
    boost::system::error_code error = boost::asio::error::would_block;

    // Start the asynchronous operation itself. The boost::lambda function
    // object is used as a callback and will update the ec variable when the
    // operation completes. The blocking_udp_client.cpp example shows how you
    // can use boost::bind rather than boost::lambda.
    boost::asio::async_write(socket, buf,
        boost::lambda::var(error) = boost::lambda::_1);

    // Block until the asynchronous operation has completed.
    do io_service.run_one(); while (error == boost::asio::error::would_block);

    // Stop the timer
    write_timer.expires_from_now(boost::posix_time::pos_infin);

    // Throw an exception if the write fails
    if (error)
        throw std::runtime_error("write: failed to write (" +
          error.message() + ")");

}

//------------------------------------------------------------------------------
// Name:        write
// Description: Writes a vector of bytes to the TCP socket with an optional
//              timeout.
// Arguments:   - data: Vector of bytes to be written.
//              - timeout: Timeout duration in milliseconds. A timeout of
//                zero disables the read timeout.
//------------------------------------------------------------------------------
void TcpSocket::write(const std::vector<uint8_t>& data, unsigned int timeout)
{
	write(&data[0], data.size(), timeout);
}

//------------------------------------------------------------------------------
// Name:        write
// Description: Writes a string to the TCP socket with an optional timeout.
// Arguments:   - data: ASCII data string to be written.
//              - timeout: Timeout duration in milliseconds. A timeout of
//                zero disables the read timeout.
//------------------------------------------------------------------------------
void TcpSocket::write(const std::string& data, unsigned int timeout)
{
	write((uint8_t*)data.c_str(), data.size(), timeout);
}

//------------------------------------------------------------------------------
// Name:        spin
// Description: Processes asynchronous IO operations. This function blocks
//              forever. Use the spin_once function in a loop to do the same
//              without blocking.
//------------------------------------------------------------------------------
void TcpSocket::spin()
{
    io_service.run();
}

//------------------------------------------------------------------------------
// Name:        spin_once
// Description: Processes asynchronous IO operations. This function handles
//              only a small set of operations at a time, and must therefore
//              be called in a loop.
//------------------------------------------------------------------------------
void TcpSocket::spin_once()
{
    io_service.poll_one();
}

//------------------------------------------------------------------------------
// Name:        check_connect_deadline
// Description: starts continual checks of the deadline timers to enforce
//              timeouts.
//------------------------------------------------------------------------------
void TcpSocket::check_connect_deadline()
{

    boost::system::error_code ignored_error;

    // Check whether any of the deadlines have passed. If any have, close the
    // socket to stop all async operations and stop all timers.
    auto time_now = boost::asio::deadline_timer::traits_type::now();
    if (connect_timer.expires_at() <= time_now )
    {
        socket.close(ignored_error);
        connect_timer.expires_at(boost::posix_time::pos_infin);
        throw ConnectTimeoutException();
    }

    // Put the actors back to sleep
    connect_timer.async_wait(boost::lambda::bind(
        &TcpSocket::check_connect_deadline, this));

}

//------------------------------------------------------------------------------
// Name:        check_read_deadline
// Description: starts continual checks of the deadline timers to enforce
//              timeouts.
//------------------------------------------------------------------------------
void TcpSocket::check_read_deadline()
{

    boost::system::error_code ignored_error;

    // Check whether any of the deadlines have passed. If any have, close the
    // socket to stop all async operations and stop all timers.
    auto time_now = boost::asio::deadline_timer::traits_type::now();
    if (read_timer.expires_at() <= time_now )
    {
        socket.close(ignored_error);
        read_timer.expires_at(boost::posix_time::pos_infin);
        throw ReadTimeoutException();
    }

    // Put the actors back to sleep
    read_timer.async_wait(boost::lambda::bind(
        &TcpSocket::check_read_deadline, this));

}

//------------------------------------------------------------------------------
// Name:        check_write_deadline
// Description: starts continual checks of the deadline timers to enforce
//              timeouts.
//------------------------------------------------------------------------------
void TcpSocket::check_write_deadline()
{

    boost::system::error_code ignored_error;

    // Check whether any of the deadlines have passed. If any have, close the
    // socket to stop all async operations and stop all timers.
    auto time_now = boost::asio::deadline_timer::traits_type::now();
    if (write_timer.expires_at() <= time_now )
    {
        socket.close(ignored_error);
        write_timer.expires_at(boost::posix_time::pos_infin);
        throw WriteTimeoutException();
    }

    // Put the actors back to sleep
    write_timer.async_wait(boost::lambda::bind(
        &TcpSocket::check_write_deadline, this));

}

//------------------------------------------------------------------------------
// Name:        start_async_read
// Description: Starts an asynchronous read of a single byte, which calls
//              the read complete callback when finished. The read complete
//              callback will process the byte with the match parser, then
//              call this function again so that reading is done
//              continuously.
//------------------------------------------------------------------------------
void TcpSocket::start_async_read()
{

	if (socket.is_open())
    {

        // Bind the read handler
        auto read_callback = boost::bind(
            &TcpSocket::read_complete_callback, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred);

        // Start the async read with a null buffer so that we can read in all
        // available bytes later in the read callback without knowing the
        // required buffer size ahead of time.
        socket.async_receive(boost::asio::null_buffers(), read_callback);

        // Start the read timer
        read_timer.expires_from_now(read_timeout_duration);

    }

}

//------------------------------------------------------------------------------
// Name:        read_complete_callback
// Description: Called when an asynchronous read finishes reading or has
//              an error during the read. Passes data to the match parser if
//              read was successful, or throws a TimeoutException or
//              ConnectionClosedException if read was not sucessful.
// Arguments:   - error: Error code indicating success or cause of failure.
//              - bytes_transferred: Number of bytes received.
//------------------------------------------------------------------------------
void TcpSocket::read_complete_callback(const boost::system::error_code& error,
                                       const size_t bytes_transferred)
{

    // If the read completed successfully
    if (error == boost::system::errc::success)
    {

		// If there are bytes to be read, read and process them
		if(socket.available() > 0)
		{

			// Cancel the read timeout timer
	        read_timer.expires_from_now(boost::posix_time::pos_infin);

	        // Get the number of bytes available to be read. We can't use
	        // bytes_transferred since it will always be zero because we called
	        // async_receive with a null buffer. This was done so that we could
			// size the buffer here after we know how many bytes were read
	        size_t num_bytes = socket.available();

	        // Create a buffer to store all available bytes, and synchronously
			// read all of the bytes into it
	        std::vector<uint8_t> bytes(num_bytes);
	        socket.receive(boost::asio::buffer(bytes));

	        // Process the received bytes into the match parser
	        process_bytes(bytes);

	        // Start the asynchronous read process again to read more bytes
	        start_async_read();

		}

		// If there are no bytes to be read, the client has closed the
		// connection and a connection close exception should be thrown
		else
		{

			connect_timer.expires_from_now(boost::posix_time::pos_infin);
            read_timer.expires_from_now(boost::posix_time::pos_infin);
            write_timer.expires_from_now(boost::posix_time::pos_infin);
	        socket.close();
	        throw ConnectionClosedException();

		}

    }

    // If the error value shows that the read was canceled by the timer, do
    // nothing. These are the Linux, Windows, and OSX error values for the
    // read being canceled apparently
    else if (error.value() == 125 ||
             error.value() == 995 ||
             error.value() == 45)
    {

    }

    // If the error code was unexpected, throw an exception
    else
    {

        connect_timer.expires_from_now(boost::posix_time::pos_infin);
        read_timer.expires_from_now(boost::posix_time::pos_infin);
        write_timer.expires_from_now(boost::posix_time::pos_infin);
        socket.close();
        throw std::runtime_error("read_complete_callback: unexpected error (" +
          error.message() + ")");

    }

}
