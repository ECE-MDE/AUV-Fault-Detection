//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to control a WHOI micromodem. Handles the sending and
//              receiving of acoustic data to and from the modem. A modem can
//              take  the following actions:
//
// Servers:     device/whoi/downlink_tx (avl_msgs/ByteArraySrv)
//              device/whoi/ping_transponders (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  device/whoi/status (avl_msgs/WhoiStatusMsg)
//              device/whoi/downlink_rx (avl_msgs/WhoiDataMsg)
//              device/whoi/transponders_rx (avl_msgs/WhoiTranspondersMsg)
//
// Subscribers: None
//==============================================================================


// Base node class
#include <avl_core/node.h>

// Utility functions
#include <avl_core/util/string.h>
#include <avl_core/util/time.h>

// ROS messages
#include <std_srvs/Trigger.h>
#include <avl_msgs/WhoiDownlinkSrv.h>
#include <avl_msgs/WhoiStatusMsg.h>
#include <avl_msgs/WhoiDataMsg.h>
#include <avl_msgs/WhoiTranspondersMsg.h>
using namespace std_srvs;
using namespace avl_msgs;

// Serial Port
#include <avl_asio/serial_port.h>

// WHOI Micromodem command protocol
#include <avl_devices/protocol/whoi_micromodem.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class WhoiMicromodemNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        WhoiMicromodemNode constructor
    //--------------------------------------------------------------------------
    WhoiMicromodemNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // WHOI micromodem ID
    const int MODEM_ID = 0;

    // Serial port instance
    SerialPort serial;

    // Service servers and publishers
    ros::ServiceServer downlink_tx_server;
    ros::ServiceServer ping_transponders_server;
    ros::Publisher status_pub;
    ros::Publisher downlink_rx_pub;
    ros::Publisher transponders_rx_pub;

    // Most recent time of arrival measurement
    double time_of_arrival;

    // Flag indicating that an acoustic transmission is in progress. The modem
    // cannot start a new transmission while already transmitting.
    bool transmit_in_progress = false;
    bool lbl_ping_in_progress = false;

    // Maximum number of bytes for each modulation type. Follows rate index in
    // micromodem manual
    std::vector<size_t> modulation_max_bytes = {32, 64, 64, 256, 256, 256, 32};

    // LBL ping settings
    int lbl_timeout;
    int lbl_channel;

    // Flag indicating whether the modem has synced its clock to the GPS time
    // and GPS PPS signal
    bool synced_to_gps = false;

    // Transmit buffer to hold bytes that will be sent to the modem when it
    // requests data to be transmitted
    std::vector<uint8_t> tx_buffer;

private:

    //--------------------------------------------------------------------------
    // Name:        acoustic_tx_srv_callback
    // Description: Service callback called when the micromodem downlink service
    //              is called to transmit data.
    // Arguments:   - request: Request message received from the client.
    //              - response: Response message to be returned to the client.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool downlink_tx_srv_callback(WhoiDownlinkSrv::Request& request,
                                  WhoiDownlinkSrv::Response& response)
    {

        // Check message size against modulation type. Four bytes are taken by
        // time of departure, so subtract those
        if (request.data.size() > (modulation_max_bytes.at(request.modulation) - 4))
        {
            response.success = false;
            std::stringstream ss;
            ss << "message size of "
               << request.data.size()
               << " bytes exceeds modulation type's limit of "
               << (modulation_max_bytes.at(request.modulation) - 4)
               << " bytes";
            response.message = ss.str();
            return true;
        }

        // Cannot downlink while a transmit is already in progress
        if (transmit_in_progress || lbl_ping_in_progress)
        {
            response.success = false;
            response.message = "transmit already in progress";
            return true;
        }

        // Load the transmit buffer
        tx_buffer = request.data;

        // Initiate a modem transmit cycle. The modem will then send a request
        // for binary data to transmit, which will be handled in the serial
        // read callback
        transmit_in_progress = true;
        write_command(WHOI_INITIATE_CYCLE(MODEM_ID, MODEM_ID,
            request.modulation));

        // Return a success response
        response.success = true;
        response.message = "success";
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        ping_transponders_srv_callback
    // Description: Service callback called when the ping transponders service
    //              is called.
    // Arguments:   - request: Request message received from the client.
    //              - response: Response message to be returned to the client.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool ping_transponders_srv_callback(Trigger::Request& request,
                                        Trigger::Response& response)
    {

        // Cannot ping transponders while a transmit is already in progress
        if (transmit_in_progress || lbl_ping_in_progress)
        {
            response.success = false;
            response.message = "transmit already in progress";
            return true;
        }

        // Initiate the transponder ping
        lbl_ping_in_progress = true;
        write_command(WHOI_PING_DIGITAL_TRANSPONDERS(lbl_channel, lbl_timeout));

        // Return a success response
        response.success = true;
        response.message = "success";
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        serial_read_callback
    // Description: Seral read callback called when a message is received from
    //              the micromodem.
    // Arguments:   - data: Data read from the serial port.
    //--------------------------------------------------------------------------
    void serial_read_callback(std::vector<uint8_t> data)
    {

        // Format the data as a string, and remove any \r\n characters
        std::string message = std::string(data.begin(), data.end());
        avl::strip(message, '\r');
        avl::strip(message, '\n');
        log_data("[serial] rx %s", message.c_str());

        // Split the message by * to remove the CRC, then by commas to get the
        // message components
        auto split_line = avl::split(message, "*");
        split_line = avl::split(split_line.at(0), ",");

        // The NMEA message type is the first entry and leads with a $
        std::string msg_type = split_line.at(0);

        // Decide what to do based on what the message type is. There are a few
        // message types that require us to do something in response, and the
        // others can be ignored

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Cycle init message. This is received any time an uplink or downlink
        // cycle is started by this modem or another modem. We must figure out
        // what kind of cycle (uplink, downlink, etc.) it is in order to handle
        // it properly
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(msg_type == "$CACYC")
        {
            log_debug("received cycle init (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Heartbeat message. This message is printed occasionally by the modem
        // and coprocessor. We don't have to do anything with it, but it's nice
        // to see that the modem is alive
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CAREV")
        {
            log_debug("received heartbeat (%s)",
                message.c_str());

            // Check if the heartbeat message indicates a reboot
            if (split_line.at(2) == "SLOT1" || split_line.at(2) == "SLOT2")
            {
                log_warning("micromodem reboot detected, reconfiguring");
                configure_micromodem();
            }

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Modem configuration message. This is printed when a configuration
        // parameter is queried to display the parameter's value
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CACFG")
        {
            log_debug("received configuration parameter message (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Time of arrival message. This is printed with the arrival of every
        // data packet, including faulty packets. Store it anyway, and we will
        // only publish it with valid messages
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CATOA")
        {
            log_debug("received time of arrival (%s)", message.c_str());
            time_of_arrival = avl::hms_to_epoch_time(split_line.at(1));
            log_debug("time_of_arrival = %.4f", time_of_arrival);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Detection of start of packet message. This is printed when the start
        // of an incoming message is detected
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CARXP")
        {
            log_debug("received detection of start of packet (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Confirmation of binary data transmit message. This is printed to
        // confirm that the modem received a binary data transmit message
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CATXD")
        {
            log_debug("received confirmation of binary data transmit (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Confirmation of ASCII data transmit message. This is printed to
        // confirm that the modem received an ASCII data transmit message
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CATXA")
        {
            log_debug("received confirmation of ASCII data transmit (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Start of packet transmission message. This is printed when the modem
        // starts transmitting a packet
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CATXP")
        {
            log_debug("received of start of packet transmission alert (%s)",
                message.c_str());
            transmit_in_progress = true;
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // End of packet transmission. This is printed when the modem
        // finishes transmitting a packet
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CATXF")
        {
            log_debug("received of end of packet transmission alert (%s)",
                message.c_str());
            transmit_in_progress = false;
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Timing source information packets. These are printed when the modem
        // timing source changes or when the timing is queried
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CATMG" || msg_type == "$CATMQ")
        {
            log_debug("received timing source information (%s)", message.c_str());

            std::string clock_source = split_line.at(2);
            std::string pps_source = split_line.at(3);
            if (clock_source == "GPS" &&
                (pps_source == "EXT" || pps_source == "EXT_SYNC") )
            {
                synced_to_gps = true;
                log_info("modem clock is synced to GPS");
            }
            else
            {
                synced_to_gps = false;
                log_warning("modem clock is not synced to GPS");
            }

            WhoiStatusMsg status_msg;
            status_msg.synced_to_gps = synced_to_gps;
            status_pub.publish(status_msg);

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Communication cycle transmit statistics message. This is printed
        // after an acoustic transmission detailing transmission statistics
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CAXST")
        {
            log_debug("received transmission statistics (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Communication cycle receive statistics message. This is printed
        // after an acoustic message is received detailing transmission
        // statistics
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CACST")
        {
            log_debug("received reception statistics (%s)",
                message.c_str());

            // Remove the message type string from the data
            split_line.erase(split_line.begin());

            // Loop through message components and turn them into a log string
            std::string data_string;
            for (auto elem : split_line)
                data_string += (elem + " ");

            log_data("[stats] %s", data_string.c_str());

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // SNR statistics on incoming PSK packet. This is printed
        // after an acoustic PSK message is received detailing SNR statistics
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CASNR")
        {
            log_debug("received SNR statistics (%s)",
                message.c_str());

            // Remove the message type string from the data
            split_line.erase(split_line.begin());

            // Loop through message components and turn them into a log string
            std::string data_string;
            for (auto elem : split_line)
                data_string += (elem + " ");

            log_data("[snr] %s", data_string.c_str());

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Data quality factor message. This is printed after an FSK encoded
        // acoustic message is received indicating the data quality from 0-255
        // Values above 200 are normally decodable, higher is better
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CADQF")
        {
            log_debug("received data quality message (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Data request message. This is printed when the modem is ready to
        // receive data from the computer, which will be immediately transmitted
        // when the modem receives the data. We must determine which data buffer
        // to send based on what type of cycle (uplink, downlink, etc.) it is
        // for.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CADRQ")
        {

            log_debug("received data request (%s)",
                message.c_str());

            // Get packet time of departure by using the time from the WHOI
            // modem data request message. Add one since the message will be
            // transmitted exactly on the next second transition
            uint32_t tod = hms_to_epoch_time(split_line.at(1)) + 1;

            // Append the time of departure to the front of the data to be
            // transmitted
            std::vector<uint8_t> bytes = avl::to_bytes<uint32_t>(tod);
            avl::append(bytes, tx_buffer);

            // Pass the data from the transmit buffer to the modem using the
            // transmit binary data command, and then remove it from the queue
            write_command(WHOI_TRANSMIT_BINARY_DATA(MODEM_ID, MODEM_ID,
                bytes));

            // Log the acoustic message sent
            log_info("time of departure: %d", tod);
            log_data("[comms] tx nan %s",
                avl::byte_to_hex(tx_buffer, false).c_str());

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Acoustic ASCII data received message. This message is printed when
        // the modem receives acoustic ASCII data from another modem. We must
        // determine which topic to publish the data to based on what type of
        // cycle (uplink, downlink, etc.) it is from.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CARXA")
        {
            log_debug("received acoustic ASCII data (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Acoustic binary data received message. This message is printed when
        // the modem receives acoustic binary data from another modem.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CARXD")
        {

            log_debug("received acoustic data (%s)", message.c_str());

            // Convert the hex-encoded ASCII data into a vector of bytes
            std::string ascii_data = split_line.at(5);
            std::vector<uint8_t> data;

            for (size_t i = 0; i < ascii_data.size(); i += 2)
            {
                std::string byte_string = ascii_data.substr(i, 2);
                uint8_t byte = strtol(byte_string.c_str(), NULL, 16);
                data.push_back(byte);
            }

            // Extract the packet time of departure, which is the first 4 bytes
            // representing epoch time in seconds. Erase them from the data
            uint32_t time_of_departure = avl::from_bytes<uint32_t>(
                avl::subvector(data, 0, 4));
            avl::remove(data, 0, 4);

            // Publish a WHOI downlinnk RX message with the received data and
            // time of arrival
            WhoiDataMsg downlink_rx_msg;
            downlink_rx_msg.time_of_departure = static_cast<double>(time_of_departure);
            downlink_rx_msg.time_of_arrival = time_of_arrival;
            downlink_rx_msg.data = data;
            downlink_rx_pub.publish(downlink_rx_msg);

            // Log the acoustic message received
            log_data("[comms] rx %.6f %s",
                time_of_arrival, ascii_data.c_str());

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Transponder travel times. This message is printed in response to a
        // Ping Digital Transponders (CCPDT) command after all transponders
        // have responded, or the timeout time is hit. The message contains
        // the one way travel time to each transponder in seconds. If the
        // transponder did not respond before the timeout, it is left blank.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$SNTTA")
        {
            log_debug("received transponder travel times message (%s)",
                message.c_str());

            std::string time_a = split_line.at(1);
            std::string time_b = split_line.at(2);
            std::string time_c = split_line.at(3);
            std::string time_d = split_line.at(4);

            WhoiTranspondersMsg msg;
            msg.travel_time_a = time_a.empty() ? NAN : std::stod(time_a);
            msg.travel_time_b = time_b.empty() ? NAN : std::stod(time_b);
            msg.travel_time_c = time_c.empty() ? NAN : std::stod(time_c);
            msg.travel_time_d = time_d.empty() ? NAN : std::stod(time_d);
            transponders_rx_pub.publish(msg);

            log_data("[lbl] %f %f %f %f",
                msg.travel_time_a,
                msg.travel_time_b,
                msg.travel_time_c,
                msg.travel_time_d);

            // Mark the transponder ping as finished
            lbl_ping_in_progress = false;

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Acoustic link error message. This error message is printed on data
        // request timeouts and bad CRC errors for incoming data.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CAMSG")
        {
            log_error("received acoustic link error message (%s)",
                message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Serial error message. This error message is printed when the serial
        // command sent to the modem from the computer is improperly formatted
        // or is somehow wrong
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$CAERR")
        {
            log_error("received serial error message (%s)",
                message.c_str());

            // If the error type is a data timeout, the modem did not receive
            // the data that it expected to transmit for a cycle
            if (avl::contains(message, "DATA_TIMEOUT"))
            {
                log_error("micromodem data request timed out");
                transmit_in_progress = false;
            }

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // GPS data mesage
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else if(msg_type == "$GPRMC")
        {
            log_data("[gps] %s", message.c_str());
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Everything else can be ignored for now until more features are
        // developed
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        else
        {
            log_debug("message ignored (%s)", message.c_str());
        }

    }

    //--------------------------------------------------------------------------
    // Name:        write_command
    // Description: Writes a serial command to the micromodem and logs it.
    // Arguments:   - command: Command string to write to the modem.
    //--------------------------------------------------------------------------
    void write_command(std::string command)
    {
        serial.write(command);
        avl::strip(command, '\r');
        avl::strip(command, '\n');
        log_data("[serial] tx %s", command.c_str());
    }

    //--------------------------------------------------------------------------
    // Name:        configure_micromodem
    // Description: Configures the micromodem settings from the config file.
    //--------------------------------------------------------------------------
    void configure_micromodem()
    {

        // Reset transmit flags
        transmit_in_progress = false;
        lbl_ping_in_progress = false;

        // Generic modem settings
        write_command(WHOI_SET_PARAMETER("SRC", MODEM_ID));
        write_command(WHOI_SET_PARAMETER("BND",
            avl::get_param<int>("~frequency_band")));
        write_command(WHOI_SET_PARAMETER("DTO",
            avl::get_param<int>("~data_request_timeout")));
        write_command(WHOI_SET_PARAMETER("SCG",
            (int)avl::get_param<bool>("~set_clock_from_gps")));
        write_command(WHOI_SET_PARAMETER("SGP",
            (int)avl::get_param<bool>("~show_gps_messages")));
        write_command(WHOI_SET_PARAMETER("SNV",
            (int)avl::get_param<bool>("~synchronous_transmission")));
        write_command(WHOI_SET_PARAMETER("TOA",
            (int)avl::get_param<bool>("~display_toa_messages")));
        write_command(WHOI_SET_PARAMETER("RXA", 0));
        write_command(WHOI_SET_PARAMETER("RXD", 1));
        write_command(WHOI_SET_PARAMETER("SNR", 1));

        // Transponder-related settings
        write_command(WHOI_SET_PARAMETER("AGN",
            avl::get_param<int>("~analog_gain")));
        write_command(WHOI_SET_PARAMETER("AGC", 1));
        write_command(WHOI_SET_PARAMETER("NDT",
            avl::get_param<int>("~detection_threshold")));
        write_command(WHOI_SET_PARAMETER("MFD", 1));
        write_command(WHOI_SET_PARAMETER("NRL", 0));
        write_command(WHOI_SET_PARAMETER("NRV", 0));

        // Configure the serial ports according to the config file
        write_command(WHOI_SET_PARAMETER("uart1.bitrate",
            avl::get_param<int>("~serial_port1/bitrate")));
        write_command(WHOI_SET_PARAMETER("uart1.task",
            avl::get_param<int>("~serial_port1/task")));
        write_command(WHOI_SET_PARAMETER("uart1.parse_gps",
            avl::get_param<int>("~serial_port1/parse_gps")));
        write_command(WHOI_SET_PARAMETER("uart1.show_gps",
            avl::get_param<int>("~serial_port1/show_gps")));
        write_command(WHOI_SET_PARAMETER("uart1.set_clk_GPS",
            avl::get_param<int>("~serial_port1/set_clk_GPS")));
        write_command(WHOI_SET_PARAMETER("uart1.crc32",
            avl::get_param<int>("~serial_port1/crc32")));

        write_command(WHOI_SET_PARAMETER("uart2.bitrate",
            avl::get_param<int>("~serial_port2/bitrate")));
        write_command(WHOI_SET_PARAMETER("uart2.task",
            avl::get_param<int>("~serial_port2/task")));
        write_command(WHOI_SET_PARAMETER("uart2.parse_gps",
            avl::get_param<int>("~serial_port2/parse_gps")));
        write_command(WHOI_SET_PARAMETER("uart2.show_gps",
            avl::get_param<int>("~serial_port2/show_gps")));
        write_command(WHOI_SET_PARAMETER("uart2.set_clk_GPS",
            avl::get_param<int>("~serial_port2/set_clk_GPS")));
        write_command(WHOI_SET_PARAMETER("uart2.crc32",
            avl::get_param<int>("~serial_port2/crc32")));

        // Request that the micromodem prints all of its configuration
        // parameters and device information
        write_command(WHOI_GET_ALL_PARAMETERS());
        write_command(WHOI_GET_DEVICE_INFO());
        write_command("$CCCFQ,uart1\r\n");
        write_command("$CCCFQ,uart2\r\n");
        write_command("$CCTMQ,0\r\n");

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[stats] version mode toa_time toa_mode mfd_peak mfd_power "
            "mfd_ratio spl shf_agn shf_ainpshift shf_ainshift shf_mfdshift "
            "shf_p2bshift rate src dest psk_err_code modulation_type nframes nbad "
            "snr_rss snr_in snr_out symbols_snr mse dqf dop stdev_noise "
            "carrier bandwidth");
        add_data_header("[stats] NA NA NA NA ? dB NA dB ? ? ? ? ? ? NA NA NA NA"
            " NA NA dB dB dB dB NA ? ? dB Hz Hz");

        add_data_header("[snr] rss snr_in snr_out symbol_snr noise_level");
        add_data_header("[snr] dB dB dB dB ?");

        add_data_header("[comms] dir toa data");
        add_data_header("[comms] rx/tx sec hex");

        // Get config file settings
        lbl_timeout = avl::get_param<int>("~lbl_timeout");
        lbl_channel = avl::get_param<int>("~lbl_channel");

        // Open the serial connection to the WHOI modem
        serial.open(avl::get_param<std::string>("~serial/port_name"),
            avl::get_param<int>("~serial/baudrate"));
        serial.set_read_timeout(avl::get_param<int>("~serial/read_timeout"));
        serial.set_match(Match("\n",
            &WhoiMicromodemNode::serial_read_callback, this));

        // Configure the modem
        configure_micromodem();

        // Set up the service servers and publishers
        downlink_tx_server = node_handle->advertiseService(
            "device/whoi/downlink_tx",
            &WhoiMicromodemNode::downlink_tx_srv_callback, this);
        ping_transponders_server = node_handle->advertiseService(
            "device/whoi/ping_transponders",
            &WhoiMicromodemNode::ping_transponders_srv_callback, this);

        status_pub = node_handle->advertise<WhoiStatusMsg>(
            "device/whoi/status", 1, true);
        downlink_rx_pub = node_handle->advertise<WhoiDataMsg>(
            "device/whoi/downlink_rx", 1);
        transponders_rx_pub = node_handle->advertise<WhoiTranspondersMsg>(
            "device/whoi/transponders_rx", 1);

        // Publish an initial status message for the status topic
        WhoiStatusMsg status_msg;
        status_msg.synced_to_gps = synced_to_gps;
        status_pub.publish(status_msg);

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
    WhoiMicromodemNode node(argc, argv);
    node.start();
    return 0;
}
