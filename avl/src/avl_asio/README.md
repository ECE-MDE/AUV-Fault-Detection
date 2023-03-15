<div align="center">

# AVL ASIO Package
</div>

The `avl_asio` package contains classes providing classes for interfacing with and parsing data from serial ports and TCP/UDP sockets.

### Match Parsing

All data from the serial port, TCP socket, and UDP socket classes in the `avl_asio` package is read using match conditions and a match parser. A match parser is used to parse an incoming stream of data into logical chunks that can be decoded, such as a stream of sensor data or status messages. 

The `MatchParser` class consists of a ring buffer to store incoming data, and one or more match conditions against which the buffer is checked. A byte or a vector of bytes can be added to the match parser ring buffer using the `process_byte` or the `process_bytes` function. When this function is called, the byte or bytes are added to the ring buffer, and the ring buffer contents are checked to determine if the buffer meets any of the match parser's match conditions. If any of the match conditions are met, the user defined match condition callback function is called in order to pass the buffer data to the user. The ring buffer is then cleared for the next match.

A match condition is a condition which, when detected, returns all read data up to and including the match condition. A match condition consists of either a string, a vector of bytes, or a number of bytes, and a match condition callback function defined by the user. 
The following match condition examples illustrate the construction of a match condition:
```c++
Match({0xFF, 0x65}, match_callback); // Match against a vector of bytes
Match("\n", match_callback);         // Match against a character
Match("abcd", match_callback);       // Match against a string
Match(5, match_callback);            // Match against a number of bytes
```
The callback function is called when the match condition is met, and has the following function signature:
```c++
void match_callback(std::vector<uint8_t> data)
{
    // Process data here
}
```
where `data` is the vector of read bytes up to and including the match condition. 

## Interfaces

The `avl_asio` package provides the `SerialPort`, `UdpSocket`, `TcpSocket` and `TcpServer` classes for asynchronous reading and synchronous writing to their respective hardware interfaces. Each of these classes inherits the `MatchParser` classes for reading data and provide functions to write data. Additionally, these classes require that the user calls one of the following functions in order to process operations:
```c++
void spin();
void spin_once();
```
These functions are similar to the ROS functions [`ros::spin()` and `ros::spinOnce()`](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning). The first function, `spin()`, will block indefinitely, processing all serial port reads and writes. The second function, `spin_once()`, will not block, but will only process one set of operations, and must therefore be called in a loop.

The following subsections describe the setup each of the classes in more detail.

### Serial Port

The `SerialPort` class can be used to read from and write to a serial port. A serial port can be opened with the default no parity, 8 bit character size, no flow control, and one stop bit using the `open()` function. The serial port name and baud rate must be specified. Different serial port settings may also be passed to the `open` function, but are optional. If the serial port is RS485, the `set_rs485` function should be called.

In Ubuntu, the user must be added to the group `dialout` in order to open a serial port, otherwise opening will fail with a `Permission denied` error.

### UDP Socket

The `UdpSocket` class can be used to read from and write to a UDP socket. Data can be written to other UDp sockets using the `send_to` functions. To receive data, the `open` function must be called and a port specified. Optionally, a UDP multicast address can be joined.

### TCP Socket

The `TcpSocket` class can be used to read from and write to a TCP socket. A TCP socket must be connected to a TCP server with the `connect` command where the remote address and port are specified. Data can be written to the  connected server with the `write` functions.

### TCP Server

The `TcpSocket` class can be used to set up a TCP server that accepts incoming connection requests from TCP sockets. Currently, the class only supports a single connection. To configure a TCP server, a session callback with the signature
```c++
void session_callback()
```
must be defined and set with the `set_session_callback` function. Note that the `spin` or `spin_once` function must be called if reading or writing in the session callback. Then, the server can be opened with the `open` function where the port is specified.