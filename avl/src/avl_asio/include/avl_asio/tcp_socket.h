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

#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

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

class TcpSocket : public MatchParser
{

public:

    //--------------------------------------------------------------------------
    // Name:        TcpSocket constructor
    // Description: Default constructor.
    // Arguments:   - buffer_size: Read buffer size in bytes (optional).
    //--------------------------------------------------------------------------
    TcpSocket(uint64_t buffer_size=65536);

    //--------------------------------------------------------------------------
    // Name:        TcpSocket destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~TcpSocket();

    //--------------------------------------------------------------------------
    // Name:        connect
    // Description: Connects to the specified address and port. If the TCP
    //              socket is already connected, it will be closed and
    //              re-connected with the specified address and port.
    // Arguments:   - address: TCP address to connect to.
    //              - port: TCP port to connect to.
    //--------------------------------------------------------------------------
    void connect(const std::string& address, unsigned int port,
        unsigned int timeout=5000);

    //--------------------------------------------------------------------------
    // Name:        is_connected
    // Description: Returns TCP socket status. True if connected, false if
    //              not connected.
    // Returns:     True if open, false otherwise.
    //--------------------------------------------------------------------------
    bool is_connected() const;

    //--------------------------------------------------------------------------
    // Name:        close
    // Description: Closes the TCP socket.
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
  	// Name:        write
  	// Description: Writes an array of bytes to the TCP socket with an optional
    //              timeout.
  	// Arguments:   - data: Pointer to bytes to be written.
  	//              - num_bytes: Number of bytes to write.
    //              - timeout: Timeout duration in milliseconds. A timeout of
    //                zero disables the read timeout.
  	//--------------------------------------------------------------------------
  	void write(const uint8_t* data, size_t num_bytes, unsigned int timeout=100);

  	//--------------------------------------------------------------------------
  	// Name:        write
  	// Description: Writes a vector of bytes to the TCP socket with an optional
    //              timeout.
  	// Arguments:   - data: Vector of bytes to be written.
    //              - timeout: Timeout duration in milliseconds. A timeout of
    //                zero disables the read timeout.
  	//--------------------------------------------------------------------------
  	void write(const std::vector<uint8_t>& data, unsigned int timeout=100);

  	//--------------------------------------------------------------------------
  	// Name:        write
  	// Description: Writes a string to the TCP socket with an optional timeout.
  	// Arguments:   - data: ASCII data string to be written.
    //              - timeout: Timeout duration in milliseconds. A timeout of
    //                zero disables the read timeout.
  	//--------------------------------------------------------------------------
  	void write(const std::string& data, unsigned int timeout=100);

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

protected:

    // Boost ASIO IO service used to run async io
    boost::asio::io_service io_service;

    // Boost ASIO TCP socket instance
    boost::asio::ip::tcp::socket socket;

    // Deadline timer for connect, read, and write
    boost::asio::deadline_timer connect_timer;
    boost::asio::deadline_timer read_timer;
    boost::asio::deadline_timer write_timer;

    // Read timeout duration
    boost::posix_time::time_duration read_timeout_duration =
        boost::posix_time::pos_infin;

protected:

    //--------------------------------------------------------------------------
    // Name:        check_connect_deadline
    // Description: starts continual checks of the connect deadline timer to
    //              enforce the connect timeout.
    //--------------------------------------------------------------------------
    void check_connect_deadline();

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
    // Description: Starts an asynchronous read of a single byte, which calls
    //              the read complete callback when finished. The read complete
    //              callback will process the byte with the match parser, then
    //              call this function again so that reading is done
    //              continuously.
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

#endif  // TCP_SOCKET_H
