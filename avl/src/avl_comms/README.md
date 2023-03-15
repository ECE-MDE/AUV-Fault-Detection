<div align="center">

# AVL Comms Package
</div>

The `avl_comms` package contains nodes that implement a communications architecture for AVL. The goal of the architecture is to facilitate communications between vehicles, computers running user interfaces, and third party computers onboard vehicles over ethernet, acoustics, and Iridium. The following diagram gives an overview of the communication architecture.

<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_comms/-/raw/master/images/comms_architecture.png" alt="Comms architecture" width="80%" height="80%">
</div>

The diagram shows communication between two vehicles and computers running user interface software over Ethernet, acoustics, and Iridium. Also pictured in red are third party computers which can communicate with and control vehicles through a backseat driver interface. The components of this diagram will be explained in more detail in the following subsections. Because the communication architecture uses the AVL binary communication protocol for passing message information and contents, it is assumed that the reader has an understanding of the AVL binary communication protocol.


## Channels

Communications channels are the methods through which messages can be transmitted or received in the communications architecture. The following table lists the communication channels implemented in the `avl_comms` package.

| Channel    | Description |
|:-:         |:-           |
| `ETHERNET` | Messages passed over Ethernet via TCP or UDP sockets. |
| `ACOUSTIC` | Messages passed through the sound via an acoustic modem. |
| `IRIDIUM`  | Messages passed through the Iridium satellite network via an Iridium modem. |
| `MISSION`  | Messages received from mission execution, such as a `COMMAND` packet from a command action. |

Messages can be passed between vehicles through any of these channels, with the exception of the `MISSION` channel, which is for internal messages from mission actions. Each channel has different behaviors for how and when messages are transmitted and received.

## Interfaces

The communications architecture is designed with the following two methods of vehicle command and control in mind:

1. Control of the vehicle via the AVL Mission Control user interface.
2. Control of the vehicle by a third party computer via an API.

These two methods are implemented through two interfaces to the communication architecture: The frontseat driver (FSD) and backseat driver (BSD) interfaces. The FSD interface is used for communication between vehicles and computers running the AVL Mission Control user interface. The BSD interface is used for communication between third party systems and main vehicle computers.


### Ethernet

Communication over the Ethernet interface is implemented in the `ethernet_channel_node`. All communication over Ethernet is done through UDP multicast, where the FSD and BSD interface each have their own UDP multicast address and port. All packets transmitted through the Ethernet interface are broadcast to a multicast address/port based on the interface. Every vehicle joins both multicast groups and received every message transmitted to them. The communication architecture then filters out packets that do not have a destination ID matching the vehicle's ID.

### Acoustic

Communication over the acoustic interface is implemented in the `acoustic_channel_node`. Because multiple acoustic modems cannot communicate at the same time without interference, communication of packets over the acoustic channel is done with a time-division multiple access (TDMA) scheme. In the TDMA scheme, each vehicle takes a turn transmitting one at a time, and the cycle repeats indefinitely. Consider example shown in the diagram:
<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_comms/-/raw/master/images/tdma_diagram.png" alt="TDMA" width="100%" height="100%">
</div>
Each TDMA cycle consists of 5 seconds of no transmission followed by three vehicles, each with a different duration for their turns in the cycle. Each vehicle's turn in the TDNA cycle is paramaterized by its offset from the start of the cycle, its duration, and the total cycle time. When AVL is started, it is assumed that TDMA cycles started at epoch time 0 (midnight 1/1/1970) and extrapolated to the current time. Naturally, all vehicle clocks must be synchronized for this scheme to work.

Packets transmitted through the acoustic channel will be added to a queue to be transmitted when it becomes the vehicle's turn to transmit. Because each vehicle's turn has a limited duration, it is possible that not all messages will be transmitted in a single turn.

### Iridium

Communication over the Iridium interface is implemented in the `iridium_channel_node`. Iridium modems require a service plan to operate, and modems must be configured to specify which modems will receive the data that they transmit via their IMEI numbers. A connection to the Iridium network is required to transmit and receive data to/from the network. When data is transmitted from a modem to the Iridium network, it is added to a mailbox for each receiving modem. Data will be queued in the mailbox until the receiving modem obtains an Iridium network connection and downloads the data. Upon acquiring an Iridium network connection, AVL will read all data from the mailbox. Packets transmitted through the Iridium channel will be added to a queue to be transmitted as soon as an Iridium connection is acquired.

### Mission


<br/><br/><br/>
<div align="center">

## Implementation
</div>

The communications architecture is implemented through 4 nodes contained in the `avl_comms` package: the ethernet, acoustic, and iridium channel nodes and the comms manager node. The specifics of transmitting and receiving bytes over each of the channels is handled by its respective channel node, and an easy to use interface to the communication architecture is implemented in the comms manager node. The following diagram illustrates the relation ship between the channel nodes, the comms manager node, and other nodes in AVL.

<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_comms/-/raw/master/images/comms_manager_node.png" alt="Comms manager node" width="50%" height="50%">
</div>

Each channel node provides a ROS service that can be called to transmit packets through the channel. These services use the service definition `avl_comms/DataTxSrv`, where the interface to transmit to and the packet to be transmitted are specified in the service request. Each channel node also publishes any packets received through the channel with the message definition `avl_comms/DataRxMessage`, which specifies the interface that the packet was received through and packet that was recevied. These services and topics are used by or subscribed to by the comms manager node.

### Comms Manager Node

The comms manager node uses the channel nodes to provide a unified means for AVL nodes to transmit and receive packets. Any packet received from a channel node is received by the comms manager node and checked for its destination ID. If the destination ID of the packet matches the vehicle ID or the destination ID is 0 (destination all vehicles) then the packet is published to the `/comms/comms_rx` topic.  In other words, all packets desined for the vehicle will be published to the topic. Similarly, the comms manager node provides a `/comms/comms_tx` ROS service that can be called to transmit a packet over a speficied channel and interface. Packets to be transmitted that are received on this service are then passed to the indicated channel node.

### PASSTHROUGH Packets

The comms manager node also handles `PASSTHROUGH` packets. Passthrough packets are packets that request that a vehicle transmit a specified packet to another vehicle through one of the communications channels. These are useful in the case of acoustic or Iridium communication where a laptop may not be conencted to the modems required to use these channel and instead sends a `PASSTHROUGH` packet to a deckbox or another vehicle to forward the packet. For example, consider the following diagram.

<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_comms/-/raw/master/images/passthrough_packet.png" alt="Passthrough packet" width="80%" height="80%">
</div>

A Laptop running AVL Mission Control (ID 1) on a boat sends a `PASSTHROUGH` packet to an AUV on the surface (ID 4) indicating that the contained `COMMAND` packet through the acoustic channel to the AUV below the surface (ID 2). The surfaced AUV responds with a RESPONSE packet and then transmits the passthrough packet to the AUV below the water. Note that `DESTINATION_ID` appears twice in a `PASSTHROUGH` packet. One instance is in the packet header, which indicates the vehicle that the packet is being sent to, and the second instance sppears as the first field of the packet, which indicates the final destination of the `PASSTHROUGH` packet.

### Packet and Command Handler Classes

The `avl_comms` package provides the `PacketHandler` and `CommandHandler` classes to simplify the use of the communication architecture for nodes. An instance of the `PacketHandler` class can be created in a node and a packet callback with signature
```c++
bool packet_callback(avl::CommsChannel channel, avl::CommsInterface interface, avl::Packet packet , bool& success, bytes_t& data)
```
can be set by calling
```c++
packet_handler.set_callback(&NodeName::packet_callback, this);
```
on the `PacketHandler` instance, assuming the node class is named `NodeName` and the `PacketHandler` instance is named `packet_handler`. Whenever the communication architecture receives a packet with the correct destination ID, the packet callback function will be called with `channel` containing the channel that the packet was received through, `interface` containing the interface that the packet was received from, and `packet` containing the packet that was received. The callback function should contain any logic for handling the received packet. The arguments `success` and `data` should be populated to construct a `RESPONSE` packet in response to the received packet. Finally, the function should return `true` if the packet was handled and the response should be sent or `false` if the packet was not handled and no response should be sent.

The `CommandHandler` is a subclass of the `PacketHandler` class that will only call the packet callback if the received packet is a `COMMAND` packet. The callback has the signature
```c++
bool command_callback(CommsChannel channel, CommsInterface interface, std::string command_name, ParameterList params, bool& result, std::vector<uint8_t>& data)
```
where `command_name` contains the `COMMAND` packet `NAME` field and `params` contains a list of dynamic parameters in the command. The remaining arguments are the same as in the `PacketHandler`. This class makes it easy to implement handling of incoming `COMMAND` packets.



