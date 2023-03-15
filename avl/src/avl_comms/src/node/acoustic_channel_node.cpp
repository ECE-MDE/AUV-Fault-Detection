//==============================================================================
// Autonomous Vehicle Library
//
// Description: A ROS node that interfaces with a WHOI micromodem in order to
//              provide an acoustic communication interface to the comms
//              manager node. The node implements a TDMA sequence where the
//              modem will broadcast message in its transmit queue when it
//              reaches the modem's turn in the cycle.
//
//              When the modem's TDMA window is reached, it follows the
//              following steps:
//
//                  1. Transmit an LBL ping if LBL pings are enabled.
//                  2. Transmit a heartbeat message if enabled.
//                  3. Transmit as many messages from the transmit queue as
//                     possible.
//
// Servers:     /comms/acoustic_tx (avl_msgs/DataTxSrv)
//              /comms/acoustic_tx_override (avl_msgs/DataTxSrv)
//
// Clients:     device/whoi/downlink_tx (avl_msgs/WhoiDownlinkSrv)
//              device/whoi/ping_transponders (std_srvs/Trigger)
//
// Publishers:  /comms/acoustic_rx (avl_msgs/DataRxMsg)
//
// Subscribers: device/whoi/downlink_rx (avl_msgs/WhoiDataMsg)
//              /comms/heartbeat (avl_msgs/HeartbeatMsg)
//==============================================================================

// Base node class
#include <avl_core/node.h>

// Utility functions
#include <avl_core/util/string.h>
#include <avl_core/util/math.h>

// ROS messages
#include <avl_msgs/DataTxSrv.h>
#include <avl_msgs/WhoiDownlinkSrv.h>
#include <std_srvs/Trigger.h>
#include <avl_msgs/DataRxMsg.h>
#include <avl_msgs/WhoiDataMsg.h>
#include <avl_msgs/HeartbeatMsg.h>
using namespace avl_msgs;
using namespace std_srvs;

// Transmit queue class
#include <avl_comms/transmit_queue.h>

// Comms architecture enums
#include <avl_comms/comms.h>

// Command handler class
#include <avl_comms/command_handler.h>
using namespace avl;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class AcousticInterfaceNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        AcousticInterfaceNode constructor
    //--------------------------------------------------------------------------
    AcousticInterfaceNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Service servers and clients, publishers, and subscribers
    ros::ServiceServer acoustic_tx_server;
    ros::ServiceServer acoustic_tx_override_server;
    ros::ServiceClient downlink_tx_client;
    ros::ServiceClient ping_transponders_client;
    ros::Publisher acoustic_rx_pub;
    ros::Subscriber downlink_rx_sub;
    ros::Subscriber heartbeat_sub;

    // TX queue to hold messages to be transmitted when the modem's TDMA
    // window begins
    TransmitQueue tx_queue;

    // Variables for TDMA broadcasting of data in the queue
    ros::Timer tdma_timer;
    double total_tdma_time;
    double tdma_offset;
    double tdma_duration;
    double max_tx_duration = 4.0;
    bool tdma_tx_active = false;
    std::vector<size_t> modulation_max_bytes = {32, 64, 64, 256, 256, 256, 32};

    // Config file setting for broadcasting
    bool broadcast_heartbeats;
    bool enable_lbl_pings;

    // Transmit modulation from config file. Can be changed by command handler
    uint8_t tx_modulation;

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

    // Ethernet channel TX client for sending status updates to the BSD
    ros::ServiceClient ethernet_tx_client;

private:

    //--------------------------------------------------------------------------
    // Name:        get_sec_to_next_window
    // Description: Gets the get_sec_to_next_window of seconds to the next
    //              broadcast window based on the current system time and the
    //              TDMA settings.
    // Returns:     Number of seconds to the next broadcast window.
    //--------------------------------------------------------------------------
    double get_sec_to_next_window()
    {

        double time_now = avl::get_epoch_time();
        double time_next_cycle = avl::round_up_to_multiple(time_now,
                                                           total_tdma_time);
        // double time_next_window = time_next_cycle + tdma_offset;
        double sec_till_next_cycle = time_next_cycle - time_now;
        double sec_till_next_window = sec_till_next_cycle + tdma_offset;

        // log_debug("=======================================================");
        // log_debug("time_now:             %f", time_now_sys);
        // log_debug("time_next_cycle:      %f", time_next_cycle);
        // log_debug("time_next_window:     %f", time_next_window);
        // log_debug("sec_till_next_cycle:  %f", sec_till_next_cycle);
        // log_debug("sec_till_next_window: %f", sec_till_next_window);
        // log_debug("=======================================================");

        return sec_till_next_window;

    }

    //--------------------------------------------------------------------------
    // Name:        transmit_data
    // Description: Transmits data using the device/whoi/downlink_tx service.
    // Returns:     Number of seconds to the next broadcast window.
    //--------------------------------------------------------------------------
    std::string transmit_data(std::vector<uint8_t> data)
    {

        // Load a downlink service request with the data
        WhoiDownlinkSrv downlink_tx_srv;
        downlink_tx_srv.request.modulation = tx_modulation;
        downlink_tx_srv.request.data = data;

        // Attempt to call the downlink service
        if (downlink_tx_client.call(downlink_tx_srv))
        {

            if (downlink_tx_srv.response.success)
            {
                log_data("[comms] tx %s",
                    avl::byte_to_hex(tx_queue.top(), false).c_str());
                return "success";
            }
            else
            {
                log_warning("downlink failed: %s",
                    downlink_tx_srv.response.message.c_str());
                return downlink_tx_srv.response.message;
            }

        }
        else
        {
            std::string msg = "downlink service call was not responded to, "
                "an acoustic modem node may not be running";
            log_warning(msg);
            return msg;
        }

    }

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when a COMMAND packet is received by the
    //              communication architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - command_name: Command name of the received command.
    //              - params: Command parameter list.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool command_callback(CommsChannel channel, CommsInterface interface,
        std::string command_name, ParameterList params,
        bool& result, std::vector<uint8_t>& data)
    {

        // Handle BROADCAST ABORT commands
        if (command_name == "BROADCAST ABORT")
        {

            log_debug("handling BROADCAST ABORT command");

            // Don't broadcast during TX queue transmissions
            if (tdma_tx_active)
            {
                std::string msg = "acoustic transmit already active";
                log_warning(msg);
                result = false;
                data = std::vector<uint8_t>(msg.begin(), msg.end());
                return true;
            }

            // Construct the packet header
            avl::PacketHeader header;
            header.timestamp = avl::get_epoch_time_nanoseconds();
            header.timeout = 0;
            header.source_id = avl::get_vehicle_id();
            header.destination_id = params.get("TARGET ID").to_int();

            // Construct the ABORT packet
            AbortPacket abort_packet(header);

            // Attempt to transmit the data
            std::string tx_result = transmit_data(abort_packet.get_bytes());
            result = tx_result == "success";
            data = std::vector<uint8_t>(tx_result.begin(), tx_result.end());

            log_debug("done handling BROADCAST ABORT command");
            return true;

        }

        // Handle RANGING commands
        if (command_name == "RANGING")
        {
            log_debug("handling RANGING command");
            enable_lbl_pings = params.get("ENABLE").to_bool();
            result = true;
            return true;
        }

        // Handle ACOMMS SET TDMA commands
        if (command_name == "ACOMMS SET TDMA")
        {
            log_debug("handling ACOMMS SET TDMA command");
            total_tdma_time = params.get("TOTAL TDMA TIME").to_double();
            tdma_offset = params.get("TDMA OFFSET").to_double();
            tdma_duration = params.get("TDMA DURATION").to_double();
            if (params.has("TX MODULATION"))
                tx_modulation = params.get("TX MODULATION").to_int();
            result = true;
            return true;
        }

        // Handle ACOMMS GET TDMA commands
        if (command_name == "ACOMMS GET TDMA")
        {
            log_debug("handling ACOMMS GET TDMA command");
            ParameterList params;
            params.add(Parameter("TOTAL TDMA TIME", total_tdma_time));
            params.add(Parameter("TDMA OFFSET", tdma_offset));
            params.add(Parameter("TDMA DURATION", tdma_duration));
            params.add(Parameter("TX MODULATION", tx_modulation));
            data = params.to_bytes();
            result = true;
            return true;
        }

        // Handle ACOMMS LBL commands
        if (command_name == "ACOMMS LBL")
        {
            log_debug("handling ACOMMS LBL command");
            enable_lbl_pings = params.get("ENABLE").to_bool();
            result = true;
            return true;
        }

        // Handle ACOMMS HEARTBEAT commands
        if (command_name == "ACOMMS HEARTBEAT")
        {
            log_debug("handling ACOMMS HEARTBEAT command");
            broadcast_heartbeats = params.get("ENABLE").to_bool();
            result = true;
            return true;
        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        acoustic_tx_srv_callback
    // Description: Service callback called when the acoustic interface transmit
    //              service is requested by a client.
    // Arguments:   - request: Request message received from the client.
    //              - response: Response message to be returned to the client.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool acoustic_tx_srv_callback(DataTxSrv::Request& request,
                                  DataTxSrv::Response& response)
    {
        Packet request_packet(request.data);
        if(request_packet.get_descriptor() == PASSTHROUGH_PACKET)
        {
            PassthroughPacket passthrough_packet(request_packet);
            std::vector<uint8_t> micro_packet = MicroPassthroughPacket::from_passthrough_packet(passthrough_packet);
            tx_queue.push(request.interface, micro_packet);
            log_info("added message to queue with priority %d: %s",
                request.interface, avl::byte_to_hex(micro_packet).c_str());
        }
        else
        {
            tx_queue.push(request.interface, request.data);
            log_info("added message to queue with priority %d: %s",
                request.interface, avl::byte_to_hex(request.data).c_str());
        }

        response.success = true;
        response.message = "success";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        acoustic_tx_override_srv_callback
    // Description: Service callback called when the acoustic interface transmit
    //              override service is requested by a client.
    // Arguments:   - request: Request message received from the client.
    //              - response: Response message to be returned to the client.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool acoustic_tx_override_srv_callback(DataTxSrv::Request& request,
                                           DataTxSrv::Response& response)
    {
        log_error("override not yet implemented");
        response.success = false;
        response.message = "override not yet implemented";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        downlink_rx_msg_callback
    // Description: Called when a micromodem data message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void downlink_rx_msg_callback(const WhoiDataMsg& message)
    {

        // Create and publish a data receive message
        std::vector<uint8_t> bytes = message.data;
        DataRxMsg acoustic_rx_msg;
        acoustic_rx_msg.interface = INTERFACE_FSD;
        acoustic_rx_msg.data = bytes;
        acoustic_rx_pub.publish(acoustic_rx_msg);
        log_data("[comms] rx %s", avl::byte_to_hex(bytes, false).c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        tdma_timer_callback
    // Description: Called when the TDMA timer triggers.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void tdma_timer_callback(const ros::TimerEvent& event)
    {

        log_info("TDMA window started, broadcasting messages...");
        tdma_tx_active = true;

        // Notify the BSD
        Event tdma_event;
        tdma_event.type = EVENT_TYPE_ACOMMS_BROADCAST_WINDOW_OPEN;
        ParameterList tdma_window_open_params;
        tdma_window_open_params.add(Parameter("MAX BYTES", static_cast<int>(modulation_max_bytes.at(tx_modulation))));
        tdma_window_open_params.add(Parameter("QUEUE SIZE", static_cast<int>(tx_queue.size())));
        tdma_window_open_params.add(Parameter("TDMA DURATION", tdma_duration));
        tdma_window_open_params.add(Parameter("MAX TX DURATION", max_tx_duration));
        tdma_event.data = tdma_window_open_params.to_bytes();
        event_handler(tdma_event, INTERFACE_BSD);

        // If LBL pinging is enabled, execute an LBL ping
        if (enable_lbl_pings)
        {
            log_info("pinging LBL transponders...");
            Trigger ping_transponders_srv;
            ping_transponders_client.call(ping_transponders_srv);
            if (ping_transponders_srv.response.success)
                log_debug("ping transponders succeeded");
            else
                log_warning("ping transponders failed: %s",
                    ping_transponders_srv.response.message.c_str());
        }

        // If the transmit queue has heartbeat data but we don't want to
        // broadcast heartbeats, remove the heartbeat data from the queue
        if (tx_queue.has_heartbeat() && !broadcast_heartbeats)
            tx_queue.pop();

        // Transmit as many acoustic messages as possible from the transmit
        // queue. A message should only be transmitted if there is enough time
        // for message transmission left in the TDMA window. This also
        // includes heartbeat messages since they are returned by the transmit
        // queue's top function before any other data

        double time_now = avl::get_epoch_time();
        double time_window_end = time_now + tdma_duration;
        double remaining_duration = time_window_end - time_now;
        bool sufficient_time = true;//remaining_duration > max_tx_duration;

        ros::Rate spin_rate(1000);
        while (ros::ok() && !tx_queue.empty() && sufficient_time)
        {

            // Check that the downlink service is running
            if (!downlink_tx_client.exists())
            {
                log_error("acoustic downlink service not found");
                break;
            }

            log_info("=======================================================");
            log_info("Starting TDMA broadcast...");
            log_info("Current time:                %f", time_now);
            log_info("Time of window end:          %f", time_window_end);
            log_info("Remaining window duration:   %f", remaining_duration);
            log_info("Sufficient time for TX:      %d", sufficient_time);
            log_info("TX queue size:               %d", tx_queue.size());
            log_info("Initiating acoustic transmit cycle...");
            log_info("=======================================================");

            // Attempt to transmit the data from the top of the queue
            std::string tx_result = transmit_data(tx_queue.top());
            log_info("transmit result: %s", tx_result.c_str());

            // If the transmit was a success, remove the transmitted data from
            // the queue. Otherwise, wait a short time to retry transmitting
            if (tx_result == "success")
            {
                tx_queue.pop();
            }
            else
            {
                double wait_duration = 1.0;
                log_info("waiting %.1f seconds to retry transmit...",
                    wait_duration);
                ros::Duration(wait_duration).sleep();
            }

            // Check if there is enough time in the TDMA window for another
            // transmission
            time_now = avl::get_epoch_time();
            remaining_duration = time_window_end - time_now;
            //sufficient_time = remaining_duration > max_tx_duration;

            ros::spinOnce();
            spin_rate.sleep();

        }

        // Set the timer to trigger at the next window
        tdma_timer.setPeriod(ros::Duration(get_sec_to_next_window()));
        tdma_timer.start();

        log_info("TDMA message transmissions finished");
        tdma_tx_active = false;

        //Notify the BSD
        tdma_event.type = EVENT_TYPE_ACOMMS_BROADCAST_WINDOW_CLOSED;
        ParameterList tdma_window_closed_params;
        tdma_window_closed_params.add(Parameter("MAX BYTES", static_cast<int>(modulation_max_bytes.at(tx_modulation))));
        tdma_window_closed_params.add(Parameter("QUEUE SIZE", static_cast<int>(tx_queue.size())));
        tdma_window_closed_params.add(Parameter("TDMA DURATION", tdma_duration));
        tdma_window_closed_params.add(Parameter("MAX TX DURATION", max_tx_duration));
        tdma_event.data = tdma_window_closed_params.to_bytes();
        event_handler(tdma_event, INTERFACE_BSD);

    }

    //--------------------------------------------------------------------------
    // Name:        heartbeat_msg_callback
    // Description: Called when a heartbeat message is received.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void heartbeat_msg_callback(const HeartbeatMsg& message)
    {

        // Don't update the heartbeat while the TDMA window is active,
        // otherwise we will just be broadcasting heartbeat messages nonstop.
        // We only want to transmit one heartbeat per window
        if (!tdma_tx_active)
            tx_queue.set_heartbeat(message.micro_heartbeat_packet);

    }

    //--------------------------------------------------------------------------
    // Name:        event_handler
    // Description: Updates the BSD with events from the mission manager
    // Arguments:   - event: Event to send
    //--------------------------------------------------------------------------
    void event_handler(Event event, CommsInterface interface)
    {

        // Construct the packet header
        avl::PacketHeader header;
        header.timestamp = avl::get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        // Construct the notification packet
        EventPacket event_packet(header, event);

        // Create the DataTx service message
        DataTxSrv srv;
        srv.request.interface = interface;
        srv.request.data = event_packet.get_bytes();

        // Call the data transmit service
        if (ethernet_tx_client.exists())
            ethernet_tx_client.call(srv);
        else
            log_error("ethernet tx service not found");

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[stats] version mode toa_time toa_mode mfd_peak "
            "mfd_power mfd_ratio spl shf_agn shf_ainpshift shf_ainshift "
            "shf_mfdshift shf_p2bshift rate src dest psk_err_code packet_type "
            "nframes nbad snr_rss snr_in snr_out symbols_snr mse dqf dop "
            "stdev_noise carrier bandwidth");
        add_data_header("[stats] NA NA NA NA ? dB NA dB ? ? ? ? ? ? NA NA NA NA"
            " NA NA dB dB dB dB NA ? ? dB Hz Hz");

        add_data_header("[snr] rss snr_in snr_out symbol_snr noise_level");
        add_data_header("[snr] dB dB dB dB ?");

        add_data_header("[comms] dir data");
        add_data_header("[comms] rx/tx hex");

        // Get config file settings
        broadcast_heartbeats = get_param<bool>("~broadcast_heartbeats");
        enable_lbl_pings =     get_param<bool>("~enable_lbl_pings");
        total_tdma_time =      get_param<double>("~total_tdma_time");
        tdma_offset =          get_param<double>("~tdma_offset");
        tdma_duration =        get_param<double>("~tdma_duration");
        tx_modulation =        get_param<int>("~default_tx_modulation");

        // Set up the Service servers, clients, publishers, and subscribers
        acoustic_tx_server = node_handle->advertiseService(
            "comms/acoustic_tx",
            &AcousticInterfaceNode::acoustic_tx_srv_callback, this);
        acoustic_tx_override_server = node_handle->advertiseService(
            "comms/acoustic_tx_override",
            &AcousticInterfaceNode::acoustic_tx_override_srv_callback, this);

        downlink_tx_client = node_handle->serviceClient<WhoiDownlinkSrv>(
            "device/whoi/downlink_tx");
        ping_transponders_client = node_handle->serviceClient<Trigger>(
            "device/whoi/ping_transponders");

        acoustic_rx_pub = node_handle->advertise<DataRxMsg>(
            "comms/acoustic_rx", 1);

        // Set up the ethernet channel TX client and RX subscriber
        ethernet_tx_client =
            node_handle->serviceClient<DataTxSrv>("comms/ethernet_tx");

        downlink_rx_sub = node_handle->subscribe("device/whoi/downlink_rx", 1,
            &AcousticInterfaceNode::downlink_rx_msg_callback, this);
        heartbeat_sub = node_handle->subscribe("comms/fsd_heartbeat", 1,
            &AcousticInterfaceNode::heartbeat_msg_callback, this);

        // Configure the timer to to trigger at the next TDMA window
        tdma_timer = node_handle->createTimer(
            ros::Duration(get_sec_to_next_window()),
            &AcousticInterfaceNode::tdma_timer_callback, this);

        // Set the command handler's callback
        command_handler.set_callback(&AcousticInterfaceNode::command_callback,
            this);

        // Wait for the WHOI micromodem node to start up
        downlink_tx_client.waitForExistence();
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
    AcousticInterfaceNode node(argc, argv);
    node.start();
    return 0;
}
