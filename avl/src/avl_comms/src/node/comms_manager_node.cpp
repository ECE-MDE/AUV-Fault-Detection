//==============================================================================
// Autonomous Vehicle Library
//
// Description:
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/misc.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

// Comms architecture enums
#include <avl_comms/comms.h>

// Packet handler class
#include <avl_comms/packet_handler.h>

// ROS message includes
#include <avl_msgs/CommsTxSrv.h>
#include <avl_msgs/CommsRxMsg.h>
#include <avl_msgs/DataTxSrv.h>
#include <avl_msgs/DataRxMsg.h>
using namespace avl_msgs;

// Packet validator class
#include <avl_core/protocol/packet_validator.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class CommsManagerNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        CommsManagerNode constructor
    //--------------------------------------------------------------------------
    CommsManagerNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Ethernet channel TX client and RX subscriber
    ros::ServiceClient ethernet_tx_client;
    ros::Subscriber ethernet_rx_sub;

    // Acoustic channel TX client and RX subscriber
    ros::ServiceClient acoustic_tx_client;
    ros::Subscriber acoustic_rx_sub;

    // Iridium channel TX client and RX subscriber
    ros::ServiceClient iridium_tx_client;
    ros::Subscriber iridium_rx_sub;

    // Mission channel TX client and RX subscriber
    ros::ServiceClient mission_tx_client;
    ros::Subscriber mission_rx_sub;

    // Comms manager TX server and RX publisher
    ros::ServiceServer comms_tx_server;
    ros::Publisher comms_rx_pub;

    // Vehicle ID number of this vehicle from global config variable
    uint8_t vehicle_id;

    // Packet validator for rejecting packets with invalid paramters
    PacketValidator validator;

    // Broadcast receive settings from config file
    bool rx_all_broadcasts;
    bool rx_own_broadcasts;

private:

    //--------------------------------------------------------------------------
    // Name:        comms_tx_srv_callback
    // Description: Called when the comms transmit service is called.
    // Arguments:   - request: request received on the service
    //              - response: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool comms_tx_srv_callback(CommsTxSrv::Request& request,
                               CommsTxSrv::Response& response)
    {

        try
        {
            Packet packet(request.data);
            transmit_packet(packet,
                static_cast<CommsInterface>(request.interface),
                static_cast<CommsChannel>(request.channel));
        }
        catch (const std::exception& ex)
        {
            response.success = false;
            response.message = std::string("unable to transmit packet (") +
                ex.what() + ")";
            log_warning("unable to transmit packet (%s)",
                response.message.c_str());
        }

        response.success = true;
        response.message = "success";
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        rx_msg_callback
    // Description: Called when a message is received.
    // Arguments:   - message: Message bytes received.
    //--------------------------------------------------------------------------
    void rx_msg_callback(const DataRxMsg& message,
        CommsChannel channel)
    {

        // Try intercepting micro packets
        try
        {

            // Intercept micro heartbeat packets and publish them as full
            // heartbeat packets
            if (message.data.at(0) == MICRO_HEARTBEAT_PACKET ||
                message.data.size() == 32)
            {
                log_debug("intercepting micro heartbeat packet");
                HeartbeatPacket packet =
                    MicroHeartbeatPacket::to_heartbeat_packet(message.data);
                CommsRxMsg msg;
                msg.channel = channel;
                msg.interface = message.interface;
                msg.data = packet.get_bytes();
                comms_rx_pub.publish(msg);
                return;
            }

            // Intercept micro passthrough packets
            if (message.data.at(0) == MICRO_PASSTHROUGH_PACKET)
            {
                log_debug("intercepting micro passthrough packet");
                PassthroughPacket passthrough_packet =
                    MicroPassthroughPacket::to_passthrough_packet(message.data);
                handle_passthrough_packet(passthrough_packet,
                        static_cast<CommsInterface>(message.interface),
                        channel);
                return;
            }

        }
        catch(const std::exception& ex)
        {
            log_warning("failed to intercept micro packet (%s) (%s), ingnoring",
                ex.what(), avl::byte_to_hex(message.data).c_str());
        }

        // If the bytes do not form a valid packet, they will be ignored
        try
        {

            // Create a packet from the message data
            Packet packet(message.data);
            PacketHeader header = packet.get_header();

            // If the packet is from the BSD interface, the timeout, source ID,
            // and destination ID bytes are marked as reserved. We will set the
            // source and destination to this vehicle's ID so they are
            // properly handled
            bool packet_from_bsd = message.interface == INTERFACE_BSD;
            if (packet_from_bsd)
            {
                header.timeout = 0;
                header.source_id = vehicle_id;
                header.destination_id = vehicle_id;
                packet.set_header(header);
            }

            // We want to handle any packet with a destination ID that matches
            // our vehicle ID. Additionally, A packet with a destination ID of
            // 0 is meant for all vehicles, so those should also be handled, but
            // only if the rx_all_broadcasts and rx_own_broadcasts settings
            // allow it
            bool src_is_self = header.source_id == vehicle_id;
            bool dest_is_self = header.destination_id == vehicle_id;
            bool dest_is_all = header.destination_id == 0;

            if (dest_is_self || (dest_is_all && rx_all_broadcasts &&
                (!src_is_self || rx_own_broadcasts)))
            {

                try
                {

                    // Validate the packet with the packet definitions file.
                    // If it fails, an exception will be thrown
                    validator.validate(packet, packet_from_bsd);

                    // If the packet is a passthrough packet, we will intercept
                    // it and handling the forwarding of the packet. This will
                    // allow us to skip the overhead of passing it to the
                    // comms_rx channel and then picking it up again with a
                    // packet handler on this node
                    if (packet.get_descriptor() == PASSTHROUGH_PACKET)
                    {
                        log_debug("intercepting PASSTHROUGH packet");
                        PassthroughPacket passthrough_packet(packet.get_bytes());
                        handle_passthrough_packet(passthrough_packet,
                            static_cast<CommsInterface>(message.interface),
                            channel);
                    }

                    // All other packet types are simply passed to the comms_rx
                    // topic for handling by other nodes
                    else
                    {
                        CommsRxMsg msg;
                        msg.channel = channel;
                        msg.interface = message.interface;
                        msg.data = message.data;
                        comms_rx_pub.publish(msg);
                    }

                }
                catch (const std::exception& ex)
                {

                    log_warning("packet validation failed (%s)", ex.what());

                    // Format a response packet to be transmitted indicating
                    // that the validation failed
                    ResponsePacket response_packet =
                        ResponsePacket::create_from(packet,
                            avl::get_vehicle_id(), false, ex.what());

                    // Transmit the response packet
                    CommsInterface interface = static_cast<CommsInterface>(
                        message.interface);
                    transmit_packet(response_packet, interface,
                        CHANNEL_ETHERNET);

                }

            }

        }
        catch(const std::exception& ex)
        {
            log_warning("rx_msg_callback: ingnoring invalid packet bytes "
                "(%s) (%s)", ex.what(), avl::byte_to_hex(message.data).c_str());
        }

    }

    //--------------------------------------------------------------------------
    // Name:        handle_contained_message
    // Description: Handles the payload of a passthrough packet based on whether
    //              it an FSD or BSD passthrough.
    // Arguments:   - packet: Passthrough packet to be handled.
    //--------------------------------------------------------------------------
    void handle_contained_message(PassthroughPacket packet)
    {

        PassthroughMessage message = packet.get_passthrough_message();

        // If the passthrough message is destined for this vehicle's FSD
        // interface, pass the passthrough data to the FSD comms_rx topic
        if (message.interface == INTERFACE_FSD)
        {

            log_debug("passthrough packet interface is FSD, extracting "
                "contained packet and publishing to comms_rx");

            // Ensure the data forms a packet
            Packet contained_packet(message.data);

            // Publish the packet to comms_rx
            CommsRxMsg msg;
            msg.channel = message.channel;
            msg.interface = message.interface;
            msg.data = message.data;
            comms_rx_pub.publish(msg);

        }

        // If the passthrough message is destined for this vehicle's BSD
        // interface, transmit the passthrough packet to the BSD interface.
        else
        {
            log_debug("passthrough packet interface is BSD, transmitting "
                "whole packet to BSD interface");
            transmit_packet(packet, INTERFACE_BSD, CHANNEL_ETHERNET);
        }

    }

    //--------------------------------------------------------------------------
    // Name:        handle_passthrough_packet
    // Description: Handles the arrival of a passthrough packet by either
    //              extracting the data it contains if its destination is this
    //              vehicle or re-transmitting it if its destination is another
    //              vehicle.
    // Arguments:   - packet: Passthrough packet to be handled.
    //--------------------------------------------------------------------------
    void handle_passthrough_packet(PassthroughPacket packet,
        CommsInterface received_interface, CommsChannel received_channel)
    {

        PacketHeader header = packet.get_header();
        PassthroughMessage message = packet.get_passthrough_message();

        bool target_is_self = message.target_id == vehicle_id;
        bool target_is_all = message.target_id == 0;
        bool has_origin = packet.has_origin_id();

        log_info("================================================================================");
        log_info("handling PASSTHROUGH packet");
        log_info("timestamp:      %ju", header.timestamp);
        log_info("timeout:        %u",  header.timeout);
        log_info("source_id:      %u",  header.source_id);
        log_info("destination_id: %u",  header.destination_id);
        log_info("--------------------------------------------------------------------------------");
        if (packet.has_origin_id())
            log_info("ORIGIN_ID: %u",  packet.get_origin_id());
        log_info("TARGET_ID: %u",  message.target_id);
        log_info("CHANNEL:   %s",  channel_to_string(static_cast<CommsChannel>(message.channel)).c_str());
        log_info("INTERFACE: %s",  interface_to_string(static_cast<CommsInterface>(message.interface)).c_str());
        log_info("DATA:      %s",  avl::byte_to_hex(message.data).c_str());
        log_info("================================================================================");

        // If the passthrough message origin is specified, then the passthrough
        // packet has already been passed by another vehicle to here
        if (has_origin)
        {

            uint8_t origin_id = packet.get_origin_id();
            bool origin_is_self = origin_id == vehicle_id;

            // If the passthrough target is this vehicle or all vehicles,
            // the passthrough packet should be passed to this vehicle's
            // comms_rx topic, but only if the origin is not itself. We don't
            // want any loops somehow occuring and don't want to handle
            // passthrough packets that this vehicle rebroadcasted
            if ((target_is_self || target_is_all) && !origin_is_self)
            {

                log_info("executing passthrough packet from ID %d passed "
                    "through ID %d", origin_id, header.source_id);

                handle_contained_message(packet);

            }

            // If the passthrough origin is this vehicle or somehow this vehicle
            // received a passthrough packet that was already rebroadcast but
            // is targeted for another vehicle, ignore the packet
            else
            {
                log_warning("ignoring passthrough packet from ID %d passed "
                    "through ID %d targeted for ID %d",
                    origin_id, header.source_id, message.target_id);
            }

        }

        // If the passthrough message origin is not specified, then the
        // passthrough packet should be passed on to the message target
        else
        {

            // If the passthrough target is only this vehicle, that means
            // someone sent this vehicle a passthrough packet targeted for
            // itself, which is silly but fine
            if (target_is_self)
            {
                log_info("executing passthrough packet from ID %d targeted "
                    "for this vehicle, no passthrough required",
                    header.source_id);

                handle_contained_message(packet);

            }

            // If the passthrough target is another vehicle or all vehicles,
            // the passthrough packet should be rebroadcast
            else
            {
                log_info("received passthrough packet from ID %d to be "
                    "passed through to target ID %d, rebroadcasting...",
                    header.source_id, message.target_id);

                // Format a RESPONSE packet
                PacketHeader response_header;
                response_header.timestamp = avl::get_epoch_time_nanoseconds();
                response_header.timeout = 0;
                response_header.source_id = avl::get_vehicle_id();
                response_header.destination_id = header.source_id;
                std::vector<uint8_t> response_data;
                bool response_result;

                // Form the passthrough packet to be re-transmitted
                PacketHeader passthrough_header;
                passthrough_header.timestamp = avl::get_epoch_time_nanoseconds();
                passthrough_header.timeout = 0;
                passthrough_header.source_id = avl::get_vehicle_id();
                passthrough_header.destination_id = message.target_id;
                PassthroughPacket passthrough_packet(passthrough_header, message);
                passthrough_packet.add_field(PASSTHROUGH_ORIGIN_ID, {header.source_id});

                // Re-transmit the entire passthrough packet through the channel
                // that it indicates in order to get it to the target
                try
                {
                    transmit_packet(passthrough_packet,
                        INTERFACE_FSD,
                        static_cast<CommsChannel>(message.channel));
                    response_result = true;
                }
                catch (const std::exception& ex)
                {
                    std::string message(ex.what());
                    response_data = std::vector<uint8_t>(message.begin(),
                                                         message.end());
                    response_result = false;
                }

                avl::Response response;
                response.source = header.timestamp;
                response.result = response_result;
                response.data = response_data;
                ResponsePacket response_packet(response_header, response);

                // Transmit the response packet to the vehicle that sent it
                transmit_packet(response_packet.get_bytes(), received_interface,
                    received_channel);

            }

        }

    }

    //--------------------------------------------------------------------------
    // Name:        transmit_packet
    // Description: Transmits a packet to the specified interface through the
    //              specified channel.
    // Arguments:   - packet: Packet to be transmitted.
    //              - interface: Interface to transmit to.
    //              - channel: Channel to transmit through.
    //--------------------------------------------------------------------------
    void transmit_packet(Packet packet, CommsInterface interface,
        CommsChannel channel)
    {

        // // If the packet is to the BSD interface, the timeout, source ID,
        // // and destination ID bytes are marked as reserved and should be
        // // zero. We will set them to zero here just to make sure.
        // if (interface == INTERFACE_BSD)
        // {
        //     PacketHeader header = packet.get_header();
        //     header.timeout = 0;
        //     header.source_id = 0;
        //     header.destination_id = 0;
        //     packet.set_header(header);
        // }

        // Create the DataTx service message
        DataTxSrv srv;
        srv.request.interface = interface;
        srv.request.data = packet.get_bytes();

        // Call the data transmit service depending on channel
        if (channel == CHANNEL_ETHERNET)
            ethernet_tx_client.call(srv);
        else if (channel == CHANNEL_ACOUSTIC)
            acoustic_tx_client.call(srv);
        else if (channel == CHANNEL_IRIDIUM)
            iridium_tx_client.call(srv);
        else if (channel == CHANNEL_MISSION)
            mission_tx_client.call(srv);
        else
            throw std::runtime_error("transmit_packet: invalid channel for "
                "transmission");

        // Check whether the service call was successful
        if (!srv.response.success)
            throw std::runtime_error("transmit_packet: transmission failed (" +
                srv.response.message + ")");

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get this vehicle's ID
        vehicle_id = avl::get_vehicle_id();
        log_info("vehicle ID: 0x%X", vehicle_id);

        // Get config file settings
        rx_all_broadcasts = get_param<bool>("~rx_all_broadcasts");
        rx_own_broadcasts = get_param<bool>("~rx_own_broadcasts");

        // Configure the RX subscriber callbacks
        typedef boost::function<void(const DataRxMsg&)> callback;

        callback ethernet_rx_callback = boost::bind(
            &CommsManagerNode::rx_msg_callback, this, boost::placeholders::_1,
            CHANNEL_ETHERNET);

        callback acoustic_rx_callback = boost::bind(
            &CommsManagerNode::rx_msg_callback, this, boost::placeholders::_1,
            CHANNEL_ACOUSTIC);

        callback iridium_rx_callback = boost::bind(
            &CommsManagerNode::rx_msg_callback, this, boost::placeholders::_1,
            CHANNEL_IRIDIUM);

        callback mission_rx_callback = boost::bind(
            &CommsManagerNode::rx_msg_callback, this, boost::placeholders::_1,
            CHANNEL_MISSION);

        // Set up the ethernet channel TX client and RX subscriber
        ethernet_tx_client =
            node_handle->serviceClient<DataTxSrv>("comms/ethernet_tx");
        ethernet_rx_sub = node_handle->subscribe<DataRxMsg>(
            "comms/ethernet_rx", 32, ethernet_rx_callback);

        acoustic_tx_client =
            node_handle->serviceClient<DataTxSrv>("comms/acoustic_tx");
        acoustic_rx_sub = node_handle->subscribe<DataRxMsg>(
            "comms/acoustic_rx", 32, acoustic_rx_callback);

        iridium_tx_client =
            node_handle->serviceClient<DataTxSrv>("comms/iridium_tx");
        iridium_rx_sub = node_handle->subscribe<DataRxMsg>(
            "comms/iridium_rx", 32, iridium_rx_callback);

        mission_tx_client =
            node_handle->serviceClient<DataTxSrv>("comms/mission_tx");
        mission_rx_sub = node_handle->subscribe<DataRxMsg>(
            "comms/mission_rx", 32, mission_rx_callback);

        // Set up the comms interface TX server and RX publisher
        comms_tx_server = node_handle->advertiseService("comms/comms_tx",
            &CommsManagerNode::comms_tx_srv_callback, this);
        comms_rx_pub =
            node_handle->advertise<CommsRxMsg>("comms/comms_rx", 32);

        // Load the packet validation files
        validator.load(get_param<std::string>("~commands_path"),
                       get_param<std::string>("~actions_path"));

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    CommsManagerNode node(argc, argv);
    node.start();
    return 0;
}
