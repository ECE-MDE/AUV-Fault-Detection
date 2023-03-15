//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node that alters ros parameters
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>

// Packet handler class
#include <avl_comms/packet_handler.h>

// Util functions
#include <avl_core/util/byte.h>
#include <avl_core/util/math.h>
#include <avl_core/util/logic.h>

// ROS tx service client
#include <avl_msgs/CommsTxSrv.h>
using namespace avl_msgs;

// std functions
#include <algorithm> //std::find

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ParameterNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ParameterNode constructor
    //--------------------------------------------------------------------------
    ParameterNode(int argc, char **argv) : Node(argc, argv) { }

private:

    // TX service client for sending parameter information
    ros::ServiceClient comms_tx_client;

    // Packet handler for handling incoming packets
    PacketHandler packet_handler;

    // Vector to hold the names of all rosparams
    std::vector<std::string> rosparam_names;

    // BSD parameter white list
    std::vector<std::string> bsd_white_list;

    // FSD parameter white list
    std::vector<std::string> fsd_white_list;


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
    bool packet_callback(CommsChannel channel, CommsInterface interface,
        Packet packet, bool& result, std::vector<uint8_t>& data)
    {

        PacketDescriptor type = packet.get_descriptor();

        try
        {

            // Handle COMMAND packets
            if (type == COMMAND_PACKET)
            {

                CommandPacket command_packet(packet);
                Command command = command_packet.get_command();

                std::vector<std::string> white_list;
                if(interface == INTERFACE_BSD)
                    white_list = bsd_white_list;

                if(interface == INTERFACE_FSD)
                    white_list = fsd_white_list;

                // Handle setting parameters
                if(command.name == "SET PARAM")
                {
                    Parameter param = command.parameters.get(0);
                    result = set_parameter(param, white_list);
                }

                // Handle Listing parameters
                if(command.name == "LIST PARAMS")
                {

                    log_debug("handling LIST PARAMS command");

                    // Make sure you're not dealing with an empty list
                    if(white_list.empty())
                    {
                        result = true;
                        return true;
                    }

                    // List of parameters to return
                    ParameterList results;

                    // Iterate through the parameters in the list
                    for(size_t i = 0; i < white_list.size(); i++)
                    {

                        // Make the temporary parameter object and attempt
                        // to set the values
                        Parameter param;
                        param.name = white_list.at(i);

                        // Only report the values that are also on the white list
                        if(get_parameter(param))
                            results.add(param);

                    } // end for

                    data = results.to_bytes();
                    result = true;
                    return true;
                }

            }

        }
        catch (const std::exception& ex)
        {
            std::string message(ex.what());
            data = std::vector<uint8_t>(message.begin(), message.end());
            result = false;
            return true;
        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        set_parameter
    // Description: Attempts to set the parameter of interest to the input value
    // Arguments:   - param: Parameter struct containing parameter info
    //              - white_list: List of parameters allowed to be set
    // Returns:     True if the parameter was correctly set. False otherwise.
    //--------------------------------------------------------------------------
    bool set_parameter(Parameter param, std::vector<std::string> white_list)
    {

        // Only worry about parameters in the white list
        if(std::find(white_list.begin(), white_list.end(),
            param.name) != bsd_white_list.end())
        {

            // Set the parameter according to type
            switch (param.type)
            {
                case TYPE_BOOL:
                    return set_param(param.name, param.to_bool());
                case TYPE_INT:
                    return set_param(param.name, param.to_int());
                case TYPE_DOUBLE:
                    return set_param(param.name, param.to_double());
                case TYPE_STRING:
                    return set_param(param.name, param.to_string());
                default: return false;
            }

        }

        return false;
    }

    //--------------------------------------------------------------------------
    // Name:        get_parameter
    // Description: Attempts to construct a Parameter struct
    // Arguments:   - param_name: name of the parameter to be fetched
    // Returns:     true if set correctly false otherwise
    //--------------------------------------------------------------------------
    bool get_parameter(Parameter &param)
    {

        // Get the parameter from the rosparam server
        XmlRpc::XmlRpcValue ros_param;
        if(node_handle->getParam(param.name, ros_param))
        {
            switch (ros_param.getType()) {
                case XmlRpc::XmlRpcValue::TypeBoolean:
                    {
                        param.type = TYPE_BOOL;
                        param.value = {static_cast<bool>(ros_param)};
                        return true;
                    }
                case XmlRpc::XmlRpcValue::TypeInt:
                    {
                        param.type = TYPE_INT;
                        int temp = static_cast<int>(ros_param);
                        param.value = avl::to_bytes(temp);
                        return true;
                    }
                case XmlRpc::XmlRpcValue::TypeDouble:{
                        param.type = TYPE_DOUBLE;
                        double temp = static_cast<double>(ros_param);
                        param.value = avl::to_bytes(temp);
                        return true;}
                case XmlRpc::XmlRpcValue::TypeString:
                    {
                        param.type = TYPE_STRING;
                        std::string temp = static_cast<std::string>(ros_param);
                        param.value = std::vector<uint8_t>(temp.begin(), temp.end());
                        return true;
                    }
                default: return false;
            }
        }

        // If the name isn't recognized by the rosparam server return false
        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        get_param_names
    // Description: Recursive function used to retreive all rosparameter names
    // Arguments:   - prev_string: String preceding poriton of rosparam name
    //              - var: value contained in rosparam
    // Returns:     Recursively fills the vector rosparam_names
    //--------------------------------------------------------------------------
    void get_rosparam_names(std::string prev_string, XmlRpc::XmlRpcValue var)
    {

        // Check for map structures (this implies a namespace)
        if(var.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {

            // Iterate through the map
            std::map<std::string, XmlRpc::XmlRpcValue>::iterator it;
            for ( it = var.begin(); it != var.end(); it++ )
            {

                // Create the preceding string
                std::string temp = prev_string;
                temp.append("/");
                temp.append(it->first);

                // Recurse to go through the levels of namespaces
                get_rosparam_names(temp, it->second);

            }

        }
        else
        {
            // Push the resulting string to the vector
            rosparam_names.push_back(prev_string);
        }

        return;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Set the packet handler's callback to handle incoming packets
        packet_handler.set_callback(&ParameterNode::packet_callback, this);

        // Set up the service clients
        comms_tx_client = node_handle->serviceClient<CommsTxSrv>("comms/comms_tx");

        // Initialize the white lists
        bsd_white_list = get_param<std::vector<std::string>>("~bsd_white_list");
        fsd_white_list = get_param<std::vector<std::string>>("~fsd_white_list");

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function or when an exception is thrown
    //              in the init or run functions.
    //--------------------------------------------------------------------------
    void shutdown()
    {

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    ParameterNode node(argc, argv);
    node.start();
    return 0;
}
