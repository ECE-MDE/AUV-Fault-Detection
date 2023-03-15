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

#include <avl_asio/serial_port.h>

// Match parser base class
#include <avl_asio/match_parser.h>

// Boost functions
#include <boost/asio.hpp>
#include <boost/bind.hpp>

// Custom exception types
#include <avl_asio/timeout_exception.h>
#include <avl_asio/connection_closed_exception.h>

// Millisecond sleep
#include <chrono>
#include <thread>

// Linux-specific serial includes
#include <linux/serial.h>
#include <linux/ioctl.h>
#include <asm/ioctls.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        SerialPort constructor
// Description: Default constructor.
//------------------------------------------------------------------------------
SerialPort::SerialPort(uint64_t buffer_size) : io_service(), port(io_service),
    timer(io_service), read_buffer(buffer_size)
{

	// Default to no read timeout
	set_read_timeout(0);

}

//------------------------------------------------------------------------------
// Name:        SerialPort destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
SerialPort::~SerialPort()
{
	port.close();
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
void SerialPort::open(const std::string& port_name, unsigned int baud_rate,
					  serial::parity parity,
					  serial::character_size char_size,
					  serial::flow_control flow_ctrl,
					  serial::stop_bits stop_bits)
{

    // Close the port and sleep to allow it to close. If this sleep is removed,
    // trying to re-open a port immediately after closing it will cause an error
    // with message "open: access denied"
    port.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(125));

    // Attempt to open the serial port and check for errors
    boost::system::error_code ec;
    port.open(port_name, ec);
    if (ec)
    {
        throw std::runtime_error(std::string("open: failed to open "
            "serial port (") + ec.message() + ")");
    }

    // Set the serial port options
	port.set_option(serial::baud_rate(baud_rate));
	port.set_option(parity);
	port.set_option(char_size);
	port.set_option(flow_ctrl);
	port.set_option(stop_bits);

	// Set the serial port to low latency mode. This is Linux only
	int fd = port.native_handle();
	struct serial_struct ser_info;
    ioctl(fd, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &ser_info);

    // Start the asynchronous reading loop that reads bytes into the match
    // parser
	start_async_read();

}

//------------------------------------------------------------------------------
// Name:        set_rs485
// Description: Enables or disables serial port RS485.
// Arguments:   - enable: True to enable RS485, false to disable.
//------------------------------------------------------------------------------
void SerialPort::set_rs485( bool enable )
{
	int fd = port.native_handle();
	struct serial_rs485 rs485conf;

	ioctl(fd, TIOCGRS485, &rs485conf);

	if (enable)
	{
	/* Enable RS485 mode: */
	rs485conf.flags |= SER_RS485_ENABLED;

	/* Set logical level for RTS pin equal to 1 when sending: */
	//rs485conf.flags |= SER_RS485_RTS_ON_SEND;

	/* Set this flag if you want to receive data even while sending */
	//rs485conf.flags &= ~(SER_RS485_RX_DURING_TX);

	/* set logical level for RTS pin equal to 0 after sending: */
	//rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

	/* Set rts delay before send, if needed: */
	//rs485conf.delay_rts_before_send = 1;

	/* Set rts delay after send, if needed: */
	//rs485conf.delay_rts_after_send = 10;
	}
	else rs485conf.flags &= ~(SER_RS485_ENABLED);

	ioctl(fd, TIOCSRS485, &rs485conf);
}

//------------------------------------------------------------------------------
// Name:        is_open
// Description: Returns serial port status. True if open, false if closed.
// Returns:     True if open, false otherwise.
//------------------------------------------------------------------------------
bool SerialPort::is_open() const
{
	return port.is_open();
}

//------------------------------------------------------------------------------
// Name:        close
// Description: Closes the serial port.
//------------------------------------------------------------------------------
void SerialPort::close()
{
	port.close();
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
void SerialPort::set_read_timeout(int timeout)
{

	// A timeout of zero correspondes to no timeout
    if (timeout == 0)
    {
        timeout_duration = boost::posix_time::pos_infin;
    }
    else
    {
        timeout_duration = boost::posix_time::milliseconds(timeout);
    }

	// Start the read timeout timer
	start_timeout_timer();

}

//------------------------------------------------------------------------------
// Name:        write
// Description: Writes data to the serial port.
// Arguments:   - data: pointer to data to be written.
//              - num_bytes: number of bytes to write.
//------------------------------------------------------------------------------
void SerialPort::write(const uint8_t* data, size_t num_bytes)
{

	boost::system::error_code error;
	boost::asio::write(port, boost::asio::buffer(data, num_bytes), error);

	if (error)
	{
		throw ConnectionClosedException();
	}

}

//------------------------------------------------------------------------------
// Name:        write
// Description: Writes data to the serial port.
// Arguments:   - data: Vector of data to be written.
//------------------------------------------------------------------------------
void SerialPort::write(const std::vector<uint8_t>& data)
{

	boost::system::error_code error;
	boost::asio::write(port, boost::asio::buffer(&data[0], data.size()));

	if (error)
	{
		throw ConnectionClosedException();
	}

}

//------------------------------------------------------------------------------
// Name:        write
// Description: Writes data to the serial port.
// Arguments:   - data: ASCII data string to be written.
//------------------------------------------------------------------------------
void SerialPort::write(const std::string& data)
{

	boost::system::error_code error;
	boost::asio::write(port, boost::asio::buffer(data.c_str(), data.size()));

	if (error)
	{
		throw ConnectionClosedException();
	}

}

//------------------------------------------------------------------------------
// Name:        spin
// Description: Processes asynchronous IO operations. This function blocks
//              forever. Use the spin_once function in a loop to do the same
//              without blocking.
//------------------------------------------------------------------------------
void SerialPort::spin()
{
    io_service.run();
}

//------------------------------------------------------------------------------
// Name:        spin_once
// Description: Processes asynchronous IO operations. This function handles
//              only a small set of operations at a time, and must therefore
//              be called in a loop.
//------------------------------------------------------------------------------
void SerialPort::spin_once()
{
    io_service.poll_one();
}

//------------------------------------------------------------------------------
// Name:        start_timeout_timer
// Description: Starts the boost deadline timer that handles the read
//              timeout. Uses timeout_duration as the duration and calls
//              timeout_expired_callback when the timer expires. This
//              function will cancel and restart the timer if it is called
//              while the timer is already running.
//------------------------------------------------------------------------------
void SerialPort::start_timeout_timer()
{

	// Set the timer to expire after the timeout duration elapses
	timer.expires_from_now(timeout_duration);

	// Bind the timer expired callback
	auto timer_callback = boost::bind(&SerialPort::timeout_expired_callback,
        this, boost::asio::placeholders::error);

	// Start the asyncronous wait for the timer to expire
	timer.async_wait(timer_callback);

}

//------------------------------------------------------------------------------
// Name:        start_async_read
// Description: Starts an asynchronous read of a single byte, which calls
//              the read complete callback when finished. The read complete
//              callback will process the byte with the match parser, then
//              call this function again so that reading is done
//              continuously.
//------------------------------------------------------------------------------
void SerialPort::start_async_read()
{

    if (port.is_open())
	{

		// Bind the read complete callback function
		auto read_callback = boost::bind(&SerialPort::read_complete_callback,
            this, boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred);

		// Start the asynchronous read
		port.async_read_some(boost::asio::buffer(read_buffer), read_callback);

		// Start the read timeout timer
		start_timeout_timer();

	}


}

//------------------------------------------------------------------------------
// Name:        timeout_expired_callback
// Description: Called when the timeout timer expires. Cancels async reads
//              if the timer was not cancelled first by a successful read.
// Arguments:   - error: Boost error code, cause of timer expiration.
//------------------------------------------------------------------------------
void SerialPort::timeout_expired_callback(const boost::system::error_code& error)
{

    // If the timer was not canceled by the read completion, close since we have
    // timed out, and throw a timeout exception
    if (!error)
    {
        port.close();
        throw TimeoutException();
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
void SerialPort::read_complete_callback(const boost::system::error_code& error,
	                                    const size_t bytes_transferred)
{

    if (!error)
	{

        // Cancel the read timeout timer
		timer.cancel();

		// Process the bytes that were read with the match parser
        process_bytes(avl::subvector(read_buffer, 0, bytes_transferred));

        // Start the asynchronous read process again to read another byte
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

		timer.cancel();
		port.close();
		throw ConnectionClosedException();

	}


}
