//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides basic functions to configure, read from, and send with
//              a UDP socket, including multicast.
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

#include <avl_asio/udp_socket.h>

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
// Name:        UdpSocket constructor
// Description: Default constructor.
// Arguments:   - buffer_size: Read buffer size in bytes (optional).
//------------------------------------------------------------------------------
UdpSocket::UdpSocket(uint64_t buffer_size) : MatchParser(buffer_size),
    io_service(), socket(io_service), read_timer(io_service),
    write_timer(io_service)
{

    // Open the socket
    socket.open(boost::asio::ip::udp::v4());

    // No deadline is required until the first socket operation is started. We
    // set the deadline to positive infinity so that the actor takes no action
    // until a specific deadline is set.
    read_timer.expires_at(boost::posix_time::pos_infin);
    write_timer.expires_at(boost::posix_time::pos_infin);

    // Start the persistent actor that checks for deadline expiry.
    check_read_deadline();
    check_write_deadline();

}

//------------------------------------------------------------------------------
// Name:        UdpSocket destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
UdpSocket::~UdpSocket()
{
    socket.close();
}

//------------------------------------------------------------------------------
// Name:        open
// Description: Opens the UDP socket to receive data on a port. Optionally,
//              a multicast address can be specified to receive data
//              broadcast to the multicast address. A valid multicast
//              address must be used. See:
//                  https://en.wikipedia.org/wiki/Multicast_address
//              If the UDP socket is already open, it will be closed and
//              re-opened.
// Arguments:   - port: Port to receive data on.
//              - multicast_address: Optional multicast group address.
//------------------------------------------------------------------------------
void UdpSocket::open(unsigned int port, const std::string& multicast_address)
{

    // Close the socket if it already open
    socket.close();

    try
    {

        // Open and bind the UDP socket
        using namespace boost::asio::ip;
        udp::endpoint listen_endpoint(udp::v4(), port);
        socket.open(listen_endpoint.protocol());
        socket.set_option(udp::socket::reuse_address(true));
        socket.bind(listen_endpoint);

        // If a multicast address is specified, join the multicast group
        if (!multicast_address.empty())
        {
            const address multicast_ip = address::from_string(multicast_address);
            socket.set_option(multicast::join_group(multicast_ip));
        }

        // Set the socket to non-blocking to prevent synchronous read functions
        // from blocking if there is not enough data to be read. A synchronous
        // read is used in the asynchronous read callback, but we don't want
        // it to block if there is not enough data
        socket.non_blocking(true);

    }
    catch (const std::exception& ex)
    {
        socket.close();
        throw std::runtime_error(
            std::string("connect: failed to open UDP socket (")
            + ex.what()
            + ")");
    }

    // Start the asynchronous read process
    start_async_read();

}

//------------------------------------------------------------------------------
// Name:        is_open
// Description: Returns UDP socket status. True if open, false if closed.
// Returns:     True if open, false otherwise.
//------------------------------------------------------------------------------
bool UdpSocket::is_open() const
{
    return socket.is_open();
}

//------------------------------------------------------------------------------
// Name:        close
// Description: Closes the UDP multicast server.
//------------------------------------------------------------------------------
void UdpSocket::close()
{
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
void UdpSocket::set_read_timeout(int timeout)
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
// Name:        send_to
// Description: Sends data to the specified destination.
// Arguments:   - data: Pointer to data to be sent.
//              - num_bytes: Number of bytes to send.
//              - timeout: Timeout duration in milliseconds. A timeout of
//                zero disables the read timeout.
//------------------------------------------------------------------------------
void UdpSocket::send_to(const std::string& address, unsigned int port,
    const uint8_t* data, size_t num_bytes, unsigned int timeout)
{

    // Check that the socket is open
    if (!socket.is_open())
        throw std::runtime_error("send_to: socket is not open ");

    // Create a buffer from the with the bytes
    auto buf = boost::asio::buffer(data, num_bytes);

    // Set a deadline for the asynchronous operation. Since this function uses
    // a composed operation (async_write), the deadline applies to the entire
    // operation, rather than individual writes to the socket.
    if (timeout == 0)
        write_timer.expires_from_now(boost::posix_time::pos_infin);
    else
        write_timer.expires_from_now(boost::posix_time::milliseconds(timeout));

    // Set up the endpoint to send to
    using namespace boost::asio::ip;
    udp::endpoint send_endpoint(address::from_string(address), port);

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
    socket.async_send_to(buf, send_endpoint,
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
// Name:        send_to
// Description: Sends data to the specified destination.
// Arguments:   - data: Vector of data to send.
//              - timeout: Timeout duration in milliseconds. A timeout of
//                zero disables the read timeout.
//------------------------------------------------------------------------------
void UdpSocket::send_to(const std::string& address, unsigned int port,
    const std::vector<uint8_t>& data, unsigned int timeout)
{
    send_to(address, port, &data[0], data.size(), timeout);
}

//------------------------------------------------------------------------------
// Name:        send_to
// Description: Sends data to the specified destination.
// Arguments:   - data: ASCII data string to send.
//              - timeout: Timeout duration in milliseconds. A timeout of
//                zero disables the read timeout.
//------------------------------------------------------------------------------
void UdpSocket::send_to(const std::string& address, unsigned int port,
    const std::string& data, unsigned int timeout)
{
    send_to(address, port, (uint8_t*)data.c_str(), data.size(), timeout);
}

//------------------------------------------------------------------------------
// Name:        spin
// Description: Processes asynchronous IO operations. This function blocks
//              forever. Use the spin_once function in a loop to do the same
//              without blocking.
//------------------------------------------------------------------------------
void UdpSocket::spin()
{
    io_service.run();
}

//------------------------------------------------------------------------------
// Name:        spin_once
// Description: Processes asynchronous IO operations. This function handles
//              only a small set of operations at a time, and must therefore
//              be called in a loop.
//------------------------------------------------------------------------------
void UdpSocket::spin_once()
{
    io_service.poll_one();
}

//------------------------------------------------------------------------------
// Name:        check_read_deadline
// Description: starts continual checks of the deadline timers to enforce
//              timeouts.
//------------------------------------------------------------------------------
void UdpSocket::check_read_deadline()
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
        &UdpSocket::check_read_deadline, this));

}

//------------------------------------------------------------------------------
// Name:        check_write_deadline
// Description: starts continual checks of the deadline timers to enforce
//              timeouts.
//------------------------------------------------------------------------------
void UdpSocket::check_write_deadline()
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
        &UdpSocket::check_write_deadline, this));

}

//------------------------------------------------------------------------------
// Name:        start_async_read
// Description: Starts an asynchronous read of a single byte, which calls
//              the read complete callback when finished. The read complete
//              callback will process the byte with the match parser, then
//              call this function again so that reading is done
//              continuously.
//------------------------------------------------------------------------------
void UdpSocket::start_async_read()
{

	if (socket.is_open())
    {

        // Bind the read handler
        auto read_callback = boost::bind(
            &UdpSocket::read_complete_callback, this,
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
void UdpSocket::read_complete_callback(const boost::system::error_code& error,
                                       const size_t bytes_transferred)
{

    // If the read completed successfully
    if (error == boost::system::errc::success)
    {

        // Cancel the read timeout timer
        read_timer.expires_from_now(boost::posix_time::pos_infin);

        // Get the number of bytes available to be read. We can't use
        // bytes_transferred since it will always be zero because we called
        // async_receive with a null buffer. This was done so that we could size
        // the buffer here after we know how many bytes were read
        size_t num_bytes = socket.available();

        // Create a buffer to store all available bytes, and synchronously read
        // all of the bytes into it
        std::vector<uint8_t> bytes(num_bytes);
        socket.receive(boost::asio::buffer(bytes));

        // Process the received bytes into the match parser
        process_bytes(bytes);

        // Start the asynchronous read process again to read more bytes
        start_async_read();

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

        read_timer.expires_from_now(boost::posix_time::pos_infin);
        write_timer.expires_from_now(boost::posix_time::pos_infin);
        socket.close();
        throw std::runtime_error("read_complete_callback: unexpected error (" +
          error.message() + ")");

    }

}
