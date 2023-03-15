//==============================================================================
// Autonomous Vehicle Library
//
// Description: A test node for testing the packet handler.
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

// Packet handler class
#include <avl_comms/packet_handler.h>
using namespace avl;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class PacketHandlerTestNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        PacketHandlerTestNode constructor
    //--------------------------------------------------------------------------
    PacketHandlerTestNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Packet handler to handle incoming packets
    PacketHandler packet_handler;

private:

    //--------------------------------------------------------------------------
    // Name:        packet_callback
    // Description: Called when an AVL packet is received by the communication
    //              architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - packet: Received packet.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool packet_callback(CommsChannel, CommsInterface, Packet, bool&,
        std::vector<uint8_t>&)
    {
        log_info("received packet");
        return false;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Set the packet handler's packet callback
        packet_handler.set_callback(
            &PacketHandlerTestNode::packet_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::Rate spin_rate(1000);
        while(ros::ok())
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
    PacketHandlerTestNode node(argc, argv);
    node.start();
    return 0;
}
