//==============================================================================
// Autonomous Vehicle Library
//
// Description: Handles interfacing with an Iridium SBD modem to provide read
//              and write functionality over the Iridium network.
//
// Servers:     device/iridium/sbd_tx (avl_msgs/ByteArraySrv)
//
// Clients:     None
//
// Publishers:  device/iridium/sbd_rx (avl_msgs/ByteArrayMsg)
//              device/iridium/signal (avl_msgs/IridiumSignalMsg)
//
// Subscribers: None
//==============================================================================

// Base node class
#include <avl_core/node.h>

// ROS messages
#include <avl_msgs/ByteArraySrv.h>
#include <avl_msgs/ByteArrayMsg.h>
#include <avl_msgs/IridiumSignalMsg.h>
using namespace avl_msgs;

// Serial Port class
#include <avl_asio/serial_port.h>

// Iridium modem AT command protocol
#include <avl_devices/protocol/iridium_modem.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class IridiumModemNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        IridiumModemNode constructor
    //--------------------------------------------------------------------------
    IridiumModemNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Serial port instance
    SerialPort serial;

    // Buffer for received messages. The modem sends a command echo, sometimes
    // followed by response data, and then an ack string
    std::vector<std::string> rx_buffer;

    // Timer used to wait for an acknowledge response to commands
    ros::Timer response_timer;
    ros::Duration response_timer_duration;

    // Iridium channel TX server and RX publisher
    ros::ServiceServer iridium_tx_server;
    ros::Publisher iridium_rx_pub;
    ros::Publisher iridium_signal_pub;

    // Iridium service availability and signal strength reported by the modem
    bool service_available = false;
    int signal_strength = 0;
    int min_signal_strengh;

    // Flag indicating whether an SBD session is currently active. A new SBD
    // session cannot be started if there is already one in progress
    bool sbd_session_active = false;

    // Flag indicating that the mailbox has been checked. On startup, the node
    // will attempt to initiate sessions to check the mailbox until it is
    // successfully checked once
    bool mailbox_checked = false;

    // Flag indicating that there is a message in the mailbox to be received
    bool mailbox_has_message = false;

    // Flag indicating that there is data in the receive buffer to be read
    bool received_data = false;

    // Flag indicating that a ring alert was received
    bool ring_alert_received = false;

private:

    //--------------------------------------------------------------------------
    // Name:        iridium_tx_srv_callback
    // Description: Service callback called when the iridium interface transmit
    //              service is requested by a client.
    // Arguments:   - request: Request message received from the client.
    //              - response: Response message to be returned to the client.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool iridium_tx_srv_callback(ByteArraySrv::Request& request,
                                 ByteArraySrv::Response& response)
    {

        // Return failure if an SBD session is already active
        if (sbd_session_active)
        {
            response.success = false;
            response.message = "SBD session already active";
            return true;
        }

        // Return failure if signal strength is not sufficient
        if (!service_available || signal_strength < min_signal_strengh)
        {
            response.success = false;
            response.message = "insufficient Iridium signal strength";
            return true;
        }

        // Get the bytes to be transmitted from the service
        std::vector<uint8_t> data = request.data;

        // Indicate that we are ready to transfer bytes into the modem TX
        // buffer. The modem will respond with READY
        write_command(IRIDIUM_WRITE_BYTES_TO_BUFFER(data.size()), "READY");

        // Write the bytes to be put in the TX buffer. The modem will respond
        // with 0 for success,
        std::string res = write_bytes(IRIDIUM_BINARY_TRANSFER(data));
        if (res == "0")
        {

            // Write the command to start an SBD session
            sbd_session_active = true;
            res = write_command(IRIDIUM_START_SBD_SESSION(ring_alert_received));
            sbd_session_active = false;

            // If the MO status indicates that the SBD session was a
            // success, clear the TX buffer and return a success response
            SbdResult sbd_result = parse_sbd_result(res);

            // Clear the modem's TX buffer
            write_command(IRIDIUM_CLEAR_WRITE_BUFFER());

            // Format the service response
            response.success = mo_status_map[sbd_result.mo_status].second;
            response.message = mo_status_map[sbd_result.mo_status].first;
            return true;

        }

        // Return an error if loading the TX buffer failed
        log_error("failed to load iridium TX buffer (error code %s)",
            res.c_str());
        response.success = false;
        response.message = "failed to load iridium TX buffer";
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        read_callback
    // Description: Called when the read match condition is met.
    // Arguments:   - data: data read by match condition
    //--------------------------------------------------------------------------
    void read_callback(std::vector<uint8_t> data)
    {

        // Format the data as a string, and remove any \r\n characters
        std::string message = std::string(data.begin(), data.end());
        avl::strip(message, '\r');
        avl::strip(message, '\n');

        // No need to handle an empty message
        if (!message.empty())
        {

            // Intercept event reports. These can arrive at any time and do
            // not need to be added to the RX buffer
            if (avl::starts_with(message, "+CIEV:"))
            {

                log_debug("received indicator event report (%s)",
                    message.c_str());

                // Determine if the message is indicating service availability
                // or signal strength and parse it
                // Split the message by the comma delimiters and parse the data
                int value = std::stoi(avl::split(message, ",").at(1));
                if (message.at(6) == '0')
                    signal_strength = value;
                else
                    service_available = value;

                // Format and publish an Iridium signal message
                IridiumSignalMsg iridium_signal_msg;
                iridium_signal_msg.service_available = service_available;
                iridium_signal_msg.signal_strength = signal_strength;
                iridium_signal_pub.publish(iridium_signal_msg);

                log_data("[status] %d %d", service_available, signal_strength);

            }

            // Intercept SBD ring alerts. These can arrive at any time and do
            // not need to be added to the RX buffer
            else if (avl::starts_with(message, "SBDRING"))
            {
                log_info("received SBD ring");
                ring_alert_received = true;
            }

            // All other messages can be added to the RX buffer
            else
            {
                rx_buffer.push_back(message);
            }

            // If the message is an SBD sesion result, we will check it and
            // set some flags based on the result
            if (avl::starts_with(message, "+SBDIX"))
            {

                log_debug("received SBD session result");
                SbdResult sbd_result = parse_sbd_result(message);

                log_info("==================================================================");
                log_info("SBD session result");
                log_info("mo_status: %s", mo_status_map[sbd_result.mo_status].first.c_str());
                log_info("mo_msn:    %d", sbd_result.mo_msn);
                log_info("mt_status: %s", mt_status_map[sbd_result.mt_status].first.c_str());
                log_info("mt_msn:    %d", sbd_result.mt_msn);
                log_info("mt_length: %d", sbd_result.mt_length);
                log_info("mt_queued: %d", sbd_result.mt_queued);
                log_info("==================================================================");
                log_data("[sbd] %d %d %d %d %d %d",
                    sbd_result.mo_status,
                    sbd_result.mo_msn,
                    sbd_result.mt_status,
                    sbd_result.mt_msn,
                    sbd_result.mt_length,
                    sbd_result.mt_queued);

                // If the MO status indicates that the SBD session was a
                // success, check for data to be received
                if (mo_status_map[sbd_result.mo_status].second)
                {
                    received_data = sbd_result.mt_length > 0;
                    mailbox_has_message = sbd_result.mt_queued > 0;
                    ring_alert_received = false;
                    mailbox_checked = true;
                }

            }

            // Log the received message
            log_info("[serial_rx] %s", message.c_str());

        }

        return;

    }

    //--------------------------------------------------------------------------
    // Name:        response_timer_callback
    // Description: Called when the response timer expires.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void response_timer_callback(const ros::TimerEvent& event)
    {
        throw std::runtime_error("timed out waiting for modem response");
    }

    //--------------------------------------------------------------------------
    // Name:        write_command
    // Description: Writes a command string to the modem and waits for the modem
    //              to acknowledge.
    // Arguments:   - cmd: Command string to write to the modem.
    //              - ack_str: Acknowledgement string to wait for.
    // Returns:     Modem response data if there was any, empty if not.
    //--------------------------------------------------------------------------
    std::string write_command(std::string cmd, std::string ack_str="OK")
    {

        // Clear the receive buffer
        rx_buffer.clear();

        // Write the command string
        std::vector<uint8_t> bytes(cmd.begin(), cmd.end());
        serial.write(bytes);
        avl::strip(cmd, '\r');
        avl::strip(cmd, '\n');
        log_info("[serial_tx] %s", cmd.c_str());

        // Wait for the command echo from the modem
        log_debug("waiting for echo (%s)", cmd.c_str());
        wait_for_string(cmd);
        log_debug("received echo", cmd.c_str());

        // Wait for the ACK string from the modem
        log_debug("waiting for ack (%s)", ack_str.c_str());
        wait_for_string(ack_str);
        log_debug("received ack");

        // If the command had response data, return the response
        return (rx_buffer.size() >= 3) ? rx_buffer.at(1) : "";

    }

    //--------------------------------------------------------------------------
    // Name:        write_command
    // Description: Writes a bytes to the modem and waits for the modem to
    //              acknowledge.
    // Arguments:   - bytes: Bytes to write to the modem.
    //              - ack_str: Acknowledgement string to wait for.
    // Returns:     Modem response data if there was any, empty if not.
    //--------------------------------------------------------------------------
    std::string write_bytes(std::vector<uint8_t> bytes,
                            std::string ack_str="OK")
    {

        // Clear the receive buffer
        rx_buffer.clear();

        // Write the bytes
        serial.write(bytes);
        log_info("[serial_tx] %s", avl::byte_to_hex(bytes).c_str());

        // Wait for the ACK string from the modem
        log_debug("waiting for ack (%s)", ack_str.c_str());
        wait_for_string(ack_str);
        log_debug("received ack");

        // If the command had response data, return the response
        return (rx_buffer.size() == 2) ? rx_buffer.at(0) : "";

    }

    //--------------------------------------------------------------------------
    // Name:        wait_for_string
    // Description: Waits until the specified string is received or until the
    //              response timer times out.
    // Arguments:   - str: String to wait for.
    //--------------------------------------------------------------------------
    void wait_for_string(std::string str)
    {

        // Start the response timer
        response_timer.start();

        // Spin wile waiting for the string to enter the rx buffer
        while (ros::ok())
        {
            if (rx_buffer.size() > 0 && rx_buffer.back() == str)
                break;
            serial.spin_once();
        };

        // If we have left the loop, the string was received

        // Reset the timer by setting a new duration
        response_timer.setPeriod(response_timer_duration, true);
        response_timer.stop();

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log data headers
        add_data_header("[status] has_service, signal_str");
        add_data_header("[status] bool bar");
        add_data_header("[sbd] mo_status mo_msn mt_status mt_msn mt_length mt_queued");
        add_data_header("[sbd] NA NA NA NA NA NA");

        // Set up the response timer as a one-shot timer. Creating the timer
        // also starts it, so stop it.
        response_timer_duration = ros::Duration(
            get_param<double>("~serial/read_timeout"));
        response_timer = node_handle->createTimer(response_timer_duration,
            &IridiumModemNode::response_timer_callback, this, true);
        response_timer.stop();

        // Configure and open the serial port
        serial.set_read_timeout(0);
        serial.set_match(Match("\r\n", &IridiumModemNode::read_callback, this));
        serial.open(get_param<std::string>("~serial/port_name"),
                    get_param<int>("~serial/baudrate"));

        // Set up the iridium channel TX server and RX publisher.
        iridium_tx_server = node_handle->
            advertiseService("device/iridium/sbd_tx",
            &IridiumModemNode::iridium_tx_srv_callback, this);
        iridium_rx_pub = node_handle->
            advertise<ByteArrayMsg>("device/iridium/sbd_rx", 32);
        iridium_signal_pub = node_handle->
            advertise<IridiumSignalMsg>("device/iridium/signal", 1, true);

        // Publish an initial status message for the latched topic
        IridiumSignalMsg iridium_signal_msg;
        iridium_signal_msg.service_available = false;
        iridium_signal_msg.signal_strength = 0;
        iridium_signal_pub.publish(iridium_signal_msg);

        // Get the minimum signal strength from the config file
        min_signal_strengh = get_param<double>("~min_signal_strengh");

        // Give the modem some time to boot up
        ros::Duration(3.0).sleep();

        log_info("configuring Iridium modem...");

        // Reset all modem settings
        log_info("resetting modem settings");
        write_command(IRIDIUM_RESET_SETTINGS());

        // Enable event reporting
        log_info("enabling event reporting");
        write_command(IRIDIUM_SET_EVENT_REPORTING(true, true));

        // Enable SBD ring notifications
        log_info("enabling SBD ring notifications");
        write_command(IRIDIUM_SET_RING_NOTIFICATIONS(true));

        // Clear read and write buffers
        log_info("Clearing read and write buffers");
        write_command(IRIDIUM_CLEAR_READ_BUFFER());
        write_command(IRIDIUM_CLEAR_WRITE_BUFFER());

        log_info("Iridium modem configuration complete");

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {

        ros::Rate spin_rate(1000);
        while (ros::ok())
        {

            // If the modem received data from the network, write the command to
            // read the data from the MT buffer. The modem response will be the
            // received binary bytes
            if (received_data)
            {

                log_debug("receive buffer contains data, reading data");
                auto res = write_command(IRIDIUM_READ_BYTES_FROM_BUFFER());
                std::vector<uint8_t> data(res.begin(), res.end());

                log_debug("received %d bytes from read buffer (%s)",
                    data.size(), avl::byte_to_hex(data).c_str());

                // Received data is formatted as two data length bytes, the
                // data itself, and then two checksum bytes. Do not try to
                // process if the read buffer data is not at least this long
                if (data.size() > 4)
                {

                    // Remove the first two data length bytes and the last two
                    // checksum bytes
                    data.erase(data.begin(), data.begin() + 2);
                    data.erase(data.end() - 2, data.end());

                    // Create and publish a data receive message
                    ByteArrayMsg iridium_rx_msg;
                    iridium_rx_msg.data = data;
                    iridium_rx_pub.publish(iridium_rx_msg);

                    // Log the received bytes
                    log_data("[rx] %s", avl::byte_to_hex(data).c_str());

                }
                else
                {
                    log_warning("read buffer empty, expected data");
                }

                // Clear the modem's RX buffer
                write_command(IRIDIUM_CLEAR_READ_BUFFER());

                // Reset the flag since we have finished reading the data
                received_data = false;

            }

            // Check if starting an SBD sesion is required to transmit or
            // receive data and the signal is sufficient
            bool session_required = !mailbox_checked ||
                                     ring_alert_received ||
                                     mailbox_has_message;
            bool signal_sufficient = service_available &&
                                     signal_strength >= min_signal_strengh;

            // We will only attempt to transmit/receive when there is network
            // service and we haven't already started an SBD session
            if (!sbd_session_active && signal_sufficient && session_required)
            {

                log_debug("starting SBD session to read messages");
                sbd_session_active = true;
                write_command(IRIDIUM_START_SBD_SESSION(ring_alert_received));
                sbd_session_active = false;

            }

            serial.spin_once();
            ros::spinOnce();
            spin_rate.sleep();

        }

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================

int main(int argc, char **argv)
{
    IridiumModemNode node(argc, argv);
    node.start();
    return 0;
}
