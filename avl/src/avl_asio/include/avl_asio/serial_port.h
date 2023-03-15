//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides basic functions to configure, read from, and write to a
//              serial port.
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

#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

// Match parser base class
#include <avl_asio/match_parser.h>

// Boost functions
#include <boost/asio.hpp>
#include <boost/bind.hpp>

// Custom exception types
#include <avl_asio/timeout_exception.h>
#include <avl_asio/connection_closed_exception.h>

// Total size of the read buffer
#define READ_BUFFER_SIZE 1024

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class SerialPort : public MatchParser
{

// Simplify the Boost ASIO serial port namespace
typedef boost::asio::serial_port_base serial;

public:

    //--------------------------------------------------------------------------
    // Name:        SerialPort constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    SerialPort(uint64_t buffer_size=65536);

    //--------------------------------------------------------------------------
    // Name:        SerialPort destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~SerialPort();

    //--------------------------------------------------------------------------
    // Name:        open
    // Description: Opens the serial port with the specified parameters. If the
    //              serial port is already open, it will be closed and re-opened
    //              with the specified settings.
    // Arguments:   - port_name: Name of port to be opened e.g. "/dev/ttyUSB0".
    //              - baud_rate: Baud rate in bits per second.
    //              - parity: Parity option (default: none).
    //              - char_size: Character size in bits (default: 8).
    //              - flow_ctrl: Flow control option (default: none).
    //              - stop_bits: Number of stop bits (default: one).
    //--------------------------------------------------------------------------
    void open(const std::string& port_name, unsigned int baud_rate,
        serial::parity parity = serial::parity(serial::parity::none),
        serial::character_size char_size = serial::character_size(8),
        serial::flow_control flow_ctrl = serial::flow_control(serial::flow_control::none),
        serial::stop_bits opt_stop = serial::stop_bits(serial::stop_bits::one));

    //--------------------------------------------------------------------------
    // Name:        set_rs485
    // Description: Enables or disables serial port RS485.
    // Arguments:   - enable: True to enable RS485, false to disable.
    //--------------------------------------------------------------------------
    void set_rs485(bool enable);

    //--------------------------------------------------------------------------
    // Name:        is_open
    // Description: Returns serial port status. True if open, false if closed.
    // Returns:     True if open, false otherwise.
    //--------------------------------------------------------------------------
    bool is_open() const;

    //--------------------------------------------------------------------------
    // Name:        close
    // Description: Closes the serial port.
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
    // Arguments:   - timeout: Timeout duration in milliseconds.
    //--------------------------------------------------------------------------
    void set_read_timeout(int timeout);

    //--------------------------------------------------------------------------
    // Name:        write
    // Description: Writes data to the serial port.
    // Arguments:   - data: pointer to data to be written.
    //              - num_bytes: number of bytes to write.
    //--------------------------------------------------------------------------
    void write(const uint8_t* data, size_t num_bytes);

    //--------------------------------------------------------------------------
    // Name:        write
    // Description: Writes data to the serial port.
    // Arguments:   - data: Vector of data to be written.
    //--------------------------------------------------------------------------
    void write(const std::vector<uint8_t>& data);

    //--------------------------------------------------------------------------
    // Name:        write
    // Description: Writes data to the serial port.
    // Arguments:   - data: ASCII data string to be written.
    //--------------------------------------------------------------------------
    void write(const std::string& data);

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

    // Boost serial port instance
    boost::asio::serial_port port;

    // Deadline timer for byte read timeout
    boost::asio::deadline_timer timer;

    // Byte read timeout duration
    boost::posix_time::time_duration timeout_duration;

    // Buffer for read bytes
    std::vector<uint8_t> read_buffer;

private:

    //--------------------------------------------------------------------------
    // Name:        start_timeout_timer
    // Description: Starts the boost deadline timer that handles the read
    //              timeout. Uses timeout_duration as the duration and calls
    //              timeout_expired_callback when the timer expires. This
    //              function will cancel and restart the timer if it is called
    //              while the timer is already running.
    //--------------------------------------------------------------------------
    void start_timeout_timer();

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
    // Name:        timeout_expired_callback
    // Description: Called when the timeout timer expires. Cancels async reads
    //              if the timer was not cancelled first by a successful read.
    // Arguments:   - error: Boost error code, cause of timer expiration.
    //--------------------------------------------------------------------------
    void timeout_expired_callback(const boost::system::error_code& error);

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

#endif  // SERIAL_PORT_H
