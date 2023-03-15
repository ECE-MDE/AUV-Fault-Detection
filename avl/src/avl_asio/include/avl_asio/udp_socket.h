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

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

// Match parser base class
#include <avl_asio/match_parser.h>

// Boost functions
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

// Custom exception types
#include <avl_asio/timeout_exception.h>
#include <avl_asio/connection_closed_exception.h>

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class UdpSocket : public MatchParser
{

public:

    //--------------------------------------------------------------------------
    // Name:        UdpSocket constructor
    // Description: Default constructor.
    // Arguments:   - buffer_size: Read buffer size in bytes (optional).
    //--------------------------------------------------------------------------
    UdpSocket(uint64_t buffer_size=65536);

    //--------------------------------------------------------------------------
    // Name:        UdpSocket destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~UdpSocket();

    //--------------------------------------------------------------------------
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
    //--------------------------------------------------------------------------
    void open(unsigned int port, const std::string& multicast_address="");

    //--------------------------------------------------------------------------
    // Name:        is_open
    // Description: Returns UDP socket status. True if open, false if closed.
    // Returns:     True if open, false otherwise.
    //--------------------------------------------------------------------------
    bool is_open() const;

    //--------------------------------------------------------------------------
    // Name:        close
    // Description: Closes the UDP socket.
    //--------------------------------------------------------------------------
    void close();

    //--------------------------------------------------------------------------
    // Name:        set_read_timeout
    // Description: Sets the read timeout duration. If the time between each
    //              data byte read (if any) exceeds the timeout duration, a
    //              TimeoutException will be thrown by the spin or spin_once
    //              functions. A timeout of zero disables the read timeout. The
    //              serial port must be re-opened after a timeout in order to be
    //              used again.
    // Arguments:   - timeout: Timeout duration in milliseconds. A timeout of
    //                zero disables the read timeout.
    //--------------------------------------------------------------------------
    void set_read_timeout(int timeout);

    //--------------------------------------------------------------------------
    // Name:        send_to
    // Description: Sends data to the specified destination.
    // Arguments:   - data: Pointer to data to be sent.
    //              - num_bytes: Number of bytes to send.
    //              - timeout: Timeout duration in milliseconds. A timeout of
    //                zero disables the read timeout.
    //--------------------------------------------------------------------------
    void send_to(const std::string& address, unsigned int port,
        const uint8_t* data, size_t num_bytes, unsigned int timeout=100);

    //--------------------------------------------------------------------------
    // Name:        send_to
    // Description: Sends data to the specified destination.
    // Arguments:   - data: Vector of data to send.
    //              - timeout: Timeout duration in milliseconds. A timeout of
    //                zero disables the read timeout.
    //--------------------------------------------------------------------------
    void send_to(const std::string& address, unsigned int port,
        const std::vector<uint8_t>& data, unsigned int timeout=100);

    //--------------------------------------------------------------------------
    // Name:        send_to
    // Description: Sends data to the specified destination.
    // Arguments:   - data: ASCII data string to send.
    //              - timeout: Timeout duration in milliseconds. A timeout of
    //                zero disables the read timeout.
    //--------------------------------------------------------------------------
    void send_to(const std::string& address, unsigned int port,
        const std::string& data, unsigned int timeout=100);

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

    // Boost ASIO IO service used to run async io
    boost::asio::io_service io_service;

    // Boost ASIO UDP socket instance
    boost::asio::ip::udp::socket socket;

    // Deadline timer for read and write
    boost::asio::deadline_timer read_timer;
    boost::asio::deadline_timer write_timer;

    // Read timeout duration
    boost::posix_time::time_duration read_timeout_duration =
        boost::posix_time::pos_infin;

private:

    //--------------------------------------------------------------------------
    // Name:        check_read_deadline
    // Description: starts continual checks of the read deadline timer to
    //              enforce the read timeout.
    //--------------------------------------------------------------------------
    void check_read_deadline();

    //--------------------------------------------------------------------------
    // Name:        check_write_deadline
    // Description: starts continual checks of the write deadline timer to
    //              enforce the write timeout.
    //--------------------------------------------------------------------------
    void check_write_deadline();

    //--------------------------------------------------------------------------
    // Name:        start_async_read
    // Description: Starts the asynchronous reading process. Sets up the async
    //              reads so that all available bytes are read and passed
    //              into the match parser one at a time, and then starts another
    //              read. Allows reading to be done continuously.
    //--------------------------------------------------------------------------
    void start_async_read();

    //--------------------------------------------------------------------------
    // Name:        read_complete_callback
    // Description: Called when an asynchronous read finishes reading or has
    //              an error during the read. Passes data to the match parser if
    //              read was successful, or throws a TimeoutException or
    //              ConnectionClosedException if read was not sucessful.
    // Arguments:   - error: Error code indicating success or cause of failure.
    //              - bytes_transferred: Number of bytes received.
	//--------------------------------------------------------------------------
	void read_complete_callback(const boost::system::error_code& error,
                                const size_t bytes_transferred);

};

#endif  // UDP_SOCKET_H
