//==============================================================================
// Autonomous Vehicle Library
//
// Description: Guidance node for handling command actions. These actions
//              publish a packet to the communication architecture when
//              executed. This allows for missions to execute commands at a
//              specific time in a missions.
//
// Servers:     /comms/mission_tx (avl_msgs/DataTxSrv)
//
// Clients:     None
//
// Publishers:  comms/mission_rx (avl_msgs/DataRxMsg)
//
// Subscribers: None
//==============================================================================

// Guidance node base class
#include <avl_guidance/guidance_node.h>

// ROS message includes
#include <avl_msgs/DataTxSrv.h>
#include <avl_msgs/DataRxMsg.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

// Comms architecture enums
#include <avl_comms/comms.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class CommandGuidanceNode : public GuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    CommandGuidanceNode(int argc, char **argv) :
        GuidanceNode("guidance/COMMAND", argc, argv)
    {

    }

private:

    // Mission interface TX server and RX publisher
    ros::ServiceServer mission_tx_server;
    ros::Publisher mission_rx_pub;

private:

    //--------------------------------------------------------------------------
    // Name:        mission_tx_srv_callback
    // Description: Called when the mission channel transmit service is called.
    // Arguments:   - request: Request received on the service.
    //              - response: Response to the service request.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool mission_tx_srv_callback(DataTxSrv::Request& request,
                                 DataTxSrv::Response& response)
    {
        log_debug("received transmit request on mission channel");
        response.success = true;
        response.message = "success";
        return true;
    }

    //--------------------------------------------------------------------------
    // Name:        start_new_action
    // Description: Called when a new action is received.
    //--------------------------------------------------------------------------
    bool start_new_action(Action action)
    {

        // Create a command from the action parameters. All action parameters
        // other than the NAME parameter are the command's parameters
        Command command;
        command.name = action.parameters.get("COMMAND NAME").to_string();
        for (const auto& param : action.parameters)
            if (param.name != "COMMAND NAME")
                command.parameters.add(param);

        PacketHeader header;
        header.timestamp = avl::get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = avl::get_vehicle_id();
        header.destination_id = avl::get_vehicle_id();

        CommandPacket packet(header, command);

        // Publish a data RX message to the comms manager to distribute the
        // command packet to the comms architecture
        DataRxMsg msg;
        msg.interface = INTERFACE_FSD;
        msg.data = packet.get_bytes();
        mission_rx_pub.publish(msg);

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_action
    // Description: Called at the iteration interval while a action is executing.
    //--------------------------------------------------------------------------
    Feedback iterate_action()
    {
        Result result;
        result.success = true;
        // finish_action(true, result);
        Feedback feedback;
        feedback.percent = 100.0;
        return feedback;
    }

    //--------------------------------------------------------------------------
    // Name:        stop_action
    // Description: Called when a action has been finished or has been canceled
    //              by the client.
    //--------------------------------------------------------------------------
    void stop_action()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {

        // Set up the mission channel TX server and RX publisher.
        mission_tx_server = node_handle->advertiseService("comms/mission_tx",
            &CommandGuidanceNode::mission_tx_srv_callback, this);
        mission_rx_pub = node_handle->advertise<DataRxMsg>
            ("comms/mission_rx", 32);

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    CommandGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
