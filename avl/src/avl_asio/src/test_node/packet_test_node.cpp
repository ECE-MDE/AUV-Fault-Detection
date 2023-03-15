//==============================================================================
// Autonomous Vehicle Library
//
// Description: A simple test node AVL binary command packets.
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
#include <avl_core/util/byte.h>
#include <avl_core/util/misc.h>

// Command packets
#include <avl_core/protocol/avl.h>

#include <avl_core/protocol/action_packet.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class PacketTestNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        PacketTestNode constructor
    //--------------------------------------------------------------------------
    PacketTestNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {
        print_packet_tests();
        print_example_packets();
    }

    //--------------------------------------------------------------------------
    // Name:        print_packet_tests
    // Description: Prints packet test cases.
    //--------------------------------------------------------------------------
    void print_packet_tests()
    {

        log_warning("Field Tests ====================================================================");

        // Construct an empty field --------------------------------------------
        avl::Field field;

        log_info("Get field bytes:       %s", avl::byte_to_hex(field.get_bytes()).c_str());
        log_info("Get field descriptor:  %s", avl::byte_to_hex(field.get_descriptor()).c_str());
        log_info("Get field data length: %s", avl::byte_to_hex(field.get_data_length()).c_str());
        log_info("Get field data:        %s", avl::byte_to_hex(field.get_data()).c_str());
        log_info("Get field string:      %s", field.get_string().c_str());

        // Construct a field from scratch --------------------------------------
        log_info("--------------------------------------------------------------------------------");

        avl::Field field0;

        field0.set_descriptor(0xAA);
        log_info("Set field descriptor: 0xAA");
        log_info("Get field descriptor: %s", avl::byte_to_hex(field0.get_descriptor()).c_str());

        field0.set_data({0x01, 0x02, 0x03});
        log_info("Set field data:        0x01 0x02 0x03");
        log_info("Get field data:        %s", avl::byte_to_hex(field0.get_data()).c_str());

        log_info("Field length:          0x03");
        log_info("Get field data length: %s", avl::byte_to_hex(field0.get_data_length()).c_str());

        log_info("Get field bytes:       %s", avl::byte_to_hex(field0.get_bytes()).c_str());
        log_info("Get field string:      %s", field0.get_string().c_str());

        // Construct a field from values ---------------------------------------
        log_info("--------------------------------------------------------------------------------");

        avl::Field field2(0xBB, {0x04, 0x05, 0x06, 0x07, 0x08});

        log_info("Field descriptor:     0xBB");
        log_info("Get field descriptor: %s", avl::byte_to_hex(field2.get_descriptor()).c_str());

        log_info("Field data:           0x04 0x05 0x06 0x07 0x08");
        log_info("Get field data:       %s", avl::byte_to_hex(field2.get_data()).c_str());

        log_info("Field length:         0x05");
        log_info("Get field length:     %s", avl::byte_to_hex(field2.get_data_length()).c_str());

        log_info("Get field bytes:      %s", avl::byte_to_hex(field2.get_bytes()).c_str());
        log_info("Get field string:     %s", field2.get_string().c_str());

        // Construct a field from bytes ----------------------------------------
        log_info("--------------------------------------------------------------------------------");

        avl::Field field3({0xCC, 0x00, 0x02, 0xAB, 0xCD});

        log_info("Field descriptor:     0xCC");
        log_info("Get field descriptor: %s", avl::byte_to_hex(field3.get_descriptor()).c_str());

        log_info("Field data:           0xAB 0xCD");
        log_info("Get field data:       %s", avl::byte_to_hex(field3.get_data()).c_str());

        log_info("Field length:         0x02");
        log_info("Get field length:     %s", avl::byte_to_hex(field3.get_data_length()).c_str());

        log_info("Get field bytes:      %s", avl::byte_to_hex(field3.get_bytes()).c_str());
        log_info("Get field string:     %s", field3.get_string().c_str());

        // Construct a field from bytes incorrectly ----------------------------
        log_info("--------------------------------------------------------------------------------");

        try
        {
            avl::Field field4({0x04, 0x00, 0x01, 0x02, 0x03});
        }
        catch (const std::exception& ex)
        {
            log_info("Field construction failed: %s", ex.what());
        }

        // Construct a field with no data incorrectly --------------------------
        log_info("--------------------------------------------------------------------------------");

        try
        {
            avl::Field field5({0x04, 0x00});
        }
        catch (const std::exception& ex)
        {
            log_info("Field construction failed: %s", ex.what());
        }

        log_warning("Packet Tests ===============================================================");

        // Construct an empty packet -------------------------------------------
        log_info("--------------------------------------------------------------------------------");

        avl::PacketHeader in_header;
        in_header.timestamp = avl::get_epoch_time_nanoseconds();
        in_header.timeout = 10;
        in_header.source_id = 1;
        in_header.destination_id = 2;
        avl::PacketDescriptor descriptor = avl::PASSTHROUGH_PACKET;

        avl::Packet packet(in_header, descriptor);

        avl::PacketHeader out_header = packet.get_header();
        log_info("set timestamp:      %ju", in_header.timestamp);
        log_info("get timestamp:      %ju", out_header.timestamp);
        log_info("set timeout:        %ju", in_header.timeout);
        log_info("get timeout:        %ju", out_header.timeout);
        log_info("set source_id:      %ju", in_header.source_id);
        log_info("get source_id:      %ju", out_header.source_id);
        log_info("set destination_id: %ju", in_header.destination_id);
        log_info("get destination_id: %ju", out_header.destination_id);
        log_info("set descriptor:     %ju", descriptor);
        log_info("get descriptor:     %ju", packet.get_descriptor());
        log_info("get bytes:          %s", avl::byte_to_hex(packet.get_bytes()).c_str());

        // Add a field ---------------------------------------------------------
        log_info("--------------------------------------------------------------------------------");

        packet.add_field(avl::Field(0x0A, {0x0B, 0x0C, 0x0D}));
        log_info("get_num_fields:  %d", packet.get_num_fields());
        log_info("has_field(0xAA): %d", packet.has_field(0xAA));

        try
        {
            packet.get_field(0xAA);
        }
        catch (const std::exception& ex)
        {
            log_info("get_field(0xAA): %s", ex.what());
        }

        log_info("has_field(0x0A): %d", packet.has_field(0x0A));
        log_info("get_field(0x0A): %s", packet.get_field(0x0A).get_string().c_str());
        log_info("get_bytes:       %s", avl::byte_to_hex(packet.get_bytes()).c_str());
        log_info("get_string:      %s", packet.get_string().c_str());

        // Construct a packet from bytes ---------------------------------------
        log_info("--------------------------------------------------------------------------------");

        try
        {
            avl::Packet packet2(packet.get_bytes());
            avl::PacketHeader out_header2 = packet2.get_header();
            log_info("set timestamp:      %ju", in_header.timestamp);
            log_info("get timestamp:      %ju", out_header2.timestamp);
            log_info("set timeout:        %ju", in_header.timeout);
            log_info("get timeout:        %ju", out_header2.timeout);
            log_info("set source_id:      %ju", in_header.source_id);
            log_info("get source_id:      %ju", out_header2.source_id);
            log_info("set destination_id: %ju", in_header.destination_id);
            log_info("get destination_id: %ju", out_header2.destination_id);
            log_info("set descriptor:     %ju", descriptor);
            log_info("get descriptor:     %ju", packet2.get_descriptor());
            log_info("get bytes:          %s", avl::byte_to_hex(packet2.get_bytes()).c_str());
        }
        catch (const std::exception& ex)
        {
            log_error("Packet construction failed: %s", ex.what());
        }

        // Construct a packet from bytes incorrectly ------------------------
        log_info("--------------------------------------------------------------------------------");

        try
        {
            avl::Packet packet3({
                0x75, 0x65, // Start-of-frame
                0x15, 0xF9, 0x7E, 0xD2, 0x98, 0x0C, 0x0A, 0x94, // Timestamp
                0x00, 0x00, 0x00, 0x0A, // Timeout
                0x01, // Source ID
                0x02, // Destination ID
                0xBC, // Descriptor
                0x00, 0x05, // Payload length (1 lower than it should be)
                0x0A, 0x00, 0x03, 0x0B, 0x0C, 0x0D, // Payload
                0xF3, 0x7A // Checksum
            });
        }
        catch (const std::exception& ex)
        {
            log_info("Packet construction failed: %s", ex.what());
        }

    }

    //--------------------------------------------------------------------------
    // Name:        print_example_packets
    // Description: Prints example packets.
    //--------------------------------------------------------------------------
    void print_example_packets()
    {

        log_warning("Example Packets =================================================================");

        avl::PacketHeader header;

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // COMMAND Packet 1 (BSD Computer → VEHICLE Computer)

        // Header
        header.timestamp = 1587565261115159459;
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        // Command
        avl::Command command1;
        command1.name = "HEARTBEAT";
        command1.parameters.add(Parameter("RATE", (int)4));

        // Packet
        avl::CommandPacket command_packet1(header, command1);
        log_info("COMMAND Packet 1 (BSD Computer → VEHICLE Computer): %s",
            command_packet1.get_string().c_str());

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // RESPONSE Packet 1 (Vehicle Computer → BSD Computer)

        // Header
        header.timestamp = 1587567154269395900;
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        // Response
        avl::Response response1;
        response1.source = 1587565261115159459;
        response1.result = true;

        // Packet
        avl::ResponsePacket response_packet1(header, response1);
        log_info("RESPONSE Packet 1 (Vehicle Computer → BSD Computer): %s",
            response_packet1.get_string().c_str());

        log_info("--------------------------------------------------------------------------------");

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // PRIMITIVE_ACTION Packet (BSD Computer → VEHICLE Computer)

        // Header
        header.timestamp = 1587650043824064956;
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        // Action
        avl::Action primitive_action;
        primitive_action.name = "PRIMITIVE";
        primitive_action.mode = ACTION_MODE_SET;
        primitive_action.parameters.add(Parameter("DURATION", 100.0));
        primitive_action.parameters.add(Parameter("YAW", 23.0));
        primitive_action.parameters.add(Parameter("WATER SPEED", 2.0));
        primitive_action.parameters.add(Parameter("DEPTH", 5.0));

        // Packet
        avl::ActionPacket action_packet(header, primitive_action);
        log_info("PRIMITIVE_ACTION Packet (BSD Computer → VEHICLE Computer): %s",
            action_packet.get_string().c_str());

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // RESPONSE Packet 2 (Vehicle Computer → BSD Computer)

        // Header
        header.timestamp = 1587567154269395900;
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        // Response
        avl::Response response2;
        response2.source = 1587650043824064956;
        response2.result = true;

        // Packet
        avl::ResponsePacket response_packet2(header, response2);
        log_info("RESPONSE Packet 2 (Vehicle Computer → BSD Computer): %s",
            response_packet2.get_string().c_str());

        log_info("--------------------------------------------------------------------------------");

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // COMMAND Packet 2 (BSD Computer → VEHICLE Computer)

        // Header
        header.timestamp = 1587565261115159459;
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        // Command
        avl::Command command2;
        command2.name = "MISSION";
        command2.parameters.add(Parameter("COMMAND", 0));

        // Packet
        avl::CommandPacket command_packet2(header, command2);
        log_info("COMMAND Packet 2 (BSD Computer → VEHICLE Computer): %s",
            command_packet2.get_string().c_str());

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // RESPONSE Packet 3 (Vehicle Computer → BSD Computer)

        // Header
        header.timestamp = 1587567154269395900;
        header.timeout = 0;
        header.source_id = 0;
        header.destination_id = 0;

        // Response
        avl::Response response3;
        response3.source = 1587565261115159459;
        response3.result = true;

        // Packet
        avl::ResponsePacket response_packet3(header, response3);
        log_info("RESPONSE Packet 3 (Vehicle Computer → BSD Computer): %s",
            response_packet3.get_string().c_str());

        log_warning("================================================================================");

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    PacketTestNode node(argc, argv);
    node.start();
    return 0;
}
