<div align="center">

# AVL System Package
</div>

The `avl_system` package is a collection of ROS nodes that control and coordinate the overall operation of the vehicle including safety monitoring, heartbeat broadcasting, and mission execution management.

<br/><br/><br/>
<div align="center">

## Mission Manager
</div>

The `avl_system` package contains the `mission_manager_node` that facilitates all mission execution on the vehicle through ROS action clients. The mission manager node contains two mission queues: one for the frontseat driver and one for the backseat driver. Actions are added to these queues and, when a mission is executed, the action is sent to the corresponding guidance node. An illustration of the relationship between the mission queues, guidance nodes, and control nodes is shown in th following diagram.

<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_system/-/raw/master/images/mission_manager.png" alt="Mission manager" width="70%" height="70%">
</div>

When a mission is started, the mission manager takes the first action from either the FSD mission queue or the BSD mission queue, depending on the mission mode, and uses an action client corresponding to the action type to send it to the guidance node that handles execution of the action type. The guidance node then iterates and sends setpoints to control nodes that are required to execute the action. When the guidance node reports that the action is finished, the mission manager sends the next action in the queue to the corresponding guidance node and the process repeats until there are no actions left in the queue. The above diagram illustrates an FSD mission being executed where actions are coming from the FSD mission queue and are sent to the corresponding guidance node and then from the guidance node to the control nodes, marked by red lines. Other actions may be passed to other guidance nodes and other control nodes.

### Mission Queue

A mission queue is an ordered list of actions to be executed by the vehicle. The mission manager node contains two mission queues: one for the frontseat driver and one for the backseat driver. Actions are executed from a mission queue in the order that they are added to the queue, and can be added to the FSD or BSD mission queue by sending an `ACTION` packet to the FSD or BSD interface.

Each mission queue in the mission manager can be in one of the following states: `ACTIVE`, `PAUSED`, or `INACTIVE`. A mission queue is `ACTIVE` when an action from the queue is being executed by a ROS action server in a guidance node.  A `PAUSED` and an `INACTIVE` queue are both queues that are not executing an action, but behave differently under conditions that will be described later.

Whan an `ACTION` packet is received from the FSD or BSD interface, it is added to the corresponding mission queue depending on the `MODE` parameter in the action. If the action mode is `SET`, the mission queue is first cleared and the action is then added to the queue. If an action is in the `ACTIVE` when a new action is received in `SET` mode, the previous action will be stopped and the received action will be immediately executed. If the packet mode is `APPEND`, the action will be added to the end of the queue.

### Mission Mode

The mission manager may be in one of the following modes.

|Mode        | Description |
|:-          |:-           |
| `MANUAL`   | Vehicle operation is controlled by manual control inputs from the frontseat driver interface. Intended for joystick control of a vehicle |
| `FSD`      | Vehicle operation is controlled by the frontseat driver mission queue. |
| `BSD`      | Vehicle operation is controlled by the backseat driver mission queue. |
| `DISABLED` | Vehicle operation is disabled. |

The mission manager mode can be changed either though a `COMMAND` packet with the `FSD_MODE`, `BSD_MODE`, and `MANUAL_MODE` commands or through the `system/mission_mode` ROS service if another node must control the mission mode. When changing mission modes, a duration can be optionally specified. If specified, the mission mode will be changed to the new mode for the speficied duration, after which the mode will be changed back to the mode that the mission manager was in before the switch. For example, if the mission mode is `FSD` and a command to change to `BSD` mode for 50 seconds is received, the mission manager will switch into `BSD` mode for 50 seconds, and then switch back to `FSD` mode.

### Mission Control

A `COMMAND` packet with a `MISSION` command can be used to control the execution of the mission queues. These commands will only be accepted from the FSD or BSD when in the corresponding mode. For example, the BSD interface cannot control its mission queue while the mission manager node is in FSD mode. THe following table lists the commands that can be used to control mission execution.

|Command | Description |
|:-|:-|
| `START`   | Starts execution of the vehicle’s BSD mission queue. |
| `PAUSE`   | Stops mission execution but does not move to the next action. Upon starting mission execution again, the action that was active when the mission was paused will be repeated. |
| `STOP`    | Stops mission execution and clears the vehicle’s BSD mission queue. |
| `ADVANCE` | Moves to the next action in the vehicle’s BSD mission queue. If the mission is executing when this command is issued, the next action will be executed. |
| `CLEAR`   | Removes all actions from the vehicle's BSD mission queue. |
| `READ`    | Requests the contents of the vehicle's BSD mission queue. The DATA field of the response to this command will contain bytes representing the ACTION packets of the actions contained in the mission queue. The bytes are arranged as <action packet 1 bytes><action packet 2 bytes>...<action packet N bytes>. |

<br/><br/><br/>
<div align="center">

## Safety Monitoring
</div>

The safety node monitors a collection of topics and verifies that the monitored topics stay within user defined bounds. Rather than embedding the safety logic into the various monitoring and publishing sensor nodes, we'll instead let them all simply publish, but monitor the values with this safety node. That way we have a centralized place to detect and respond to safety limit violations. This also allows safety logic to be independent of any new sensor nodes that may be developed in the future, assuming they publish to the correct topics.

When a safety parameter's limits are exceeded, the safety node takes the following actions:
1. Logs an error message describing the parameter whose limits were exceeded
2. Disables all vehicle actuator control using the `actuator_node`'s `device/disable_actuators` service
3. Stops any running missions with the `mission_node`'s `/mission_action` service with the STOP argument

The safety node can be reset after a violation is detected by calling its `/reset_safety_node` service. This service will re-enable actuator control and resume checking for safety parameter violations.

The safety node's safety checks can be enabled/disabled and modified in the safety node config file.

<br/><br/><br/>
<div align="center">

## Heartbeat Broadcasting
</div>

The `avl_system` package contains the `heartbeat_node` that creates `HEARTBEAT` packets from various ROS topics and periodically broadcasts them using the AVL comms architecture. The node also handles any `HEARTBEAT` commands from a `COMMAND` packet received by the vehicle to configure the broadcast rate. The rate can be configured separately for the frontseat and backseat driver interfaces.