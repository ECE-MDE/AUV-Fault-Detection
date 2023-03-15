<div align="center">

# AVL Control Package
</div>

The `avl_control` package provides a classes and nodes that can be used to implement control algorithms as ROS nodes consisting of inputs, setpoints, and outputs. In general, a control node attempts to control a variable of interest by comparing a setpoint to an input and producing a control output. For example, an RPM controller is given an RPM setpoint, compares it to the measured RPM, and adjusts the output motor speed percentage until they match. The following figure represents a generic control node with an input, a setpoint, and an output.

<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_control/-/raw/08a9f5fa9d01ca28ec1c033bc7214e656c3fa759/images/control_node.png" alt="Control node" width="40%" height="40%">
</div>


Additionally, control nodes can be cascaded. This means that the output of one control node is the setpoint to another as illustrated in the following figure.

<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_control/-/raw/08a9f5fa9d01ca28ec1c033bc7214e656c3fa759/images/cascaded_control_node.png" alt="Cascaded control nodes" width="65%" height="65%">
</div>

For example, in an autonomous underwater vehicle, depth can be controlled using a cascaded control architecture illustrated in the following figure.

<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_control/-/raw/08a9f5fa9d01ca28ec1c033bc7214e656c3fa759/images/depth_control_example.png" alt="Cascaded control example" width="65%" height="65%">
</div>


A depth setpoint is published to the `setpoint/depth` topic, which is received by the `depth_control_node`. The node also subscribes to the `device/depth` for measurements of depth as the control node input. The `depth_control_node` publishes a pitch setpoint which is the setpoint of the `attitude_control_node`. The `attitude_control_node` then publishes its output to the `device/fins` topic in order to control the vehicle attitude.

The `avl_control` package provides a `ControlNode` base class that automates setpoint, input, iteration, and fault detection logic and can be inherited to define new control nodes. The package also contains a collection of nodes inheriting the control node base class that can be used to control vehicle trajectory. Finally, the package also contains the `Pid` and `StateSpaceModel` classes to facilitate the creation of control nodes.

<br/><br/><br/>
<div align="center">

## Control Node Base Class
</div>
The `ControlNode` base class implements logic for setpoint amd input subscription, algorithm iteration, and fault detection in order to ensure the proper operation of all AVL controllers and abstract away the details for developers. The abstraction allows for easy implementation of new control nodes. The following flowchart illustrates the logical flow of the control node base class.

<div align="center">


<img src="https://ascl3.ece.vt.edu/avl/ros-packages/avl_control/-/raw/master/images/control_node_flowchart.png" alt="Control node flowchart" width="30%" height="30%">
</div>


### Initialization
The `ControlNode` base class provides three functions for initializing the node. To configure the iteration rate of the node, the function
```c++
void set_iteration_rate(double rate)
```
should be called where `rate` is the iteration rate in Hz. To configure inputs, call the function
```c++
template <typename T>
void add_input(std::string topic_name, double min_rate)
```
where `topic_name` is the name of the input topic to subscribe to and `min_rate` is the minimum message rate in Hz. If input messages are not received faster than this rate and the node has enabled setpoints, the node will detect a fault and shut down. Any ROS message type may be used as an input and any number of inputs may be added.

Setpoint topics can be added by calling
```c++
template <typename T>
void add_setpoint(std::string topic_name, double min_rate)
```
where `topic_name` is the name of the setpoint topic to subscribe to and `min_rate` is the minimum message rate in Hz. If a setpoint message is not received faster than this rate, the setpoint will be disabled. Any message type can be used as a setpoint, **provided that the message has a `bool enable` parameter in its definition**. The `avl_control` package provides some setpoint message definitions for common setpoint types. A setpoint will be enabled if a setpoint message with `enable` set to true is received, and will be disabled if a setpoint message with `enable` set to false is received or if the setpoint is not received at its minimum rate. Any number of setpoints may be added. The following code is an example of control node initialization for the depth control node in the cascaded control diagram above:
```c++
set_iteration_rate(10.0);
add_input<std_msgs/Float64>("device/depth", 1.0);
add_setpoint<avl_control/Float64SetpointMsg>("setpoint/depth", 1.0);
```

### Iteration
If any of the setpoints configured with the `add_setpoint` function are enabled, the control node will call a user-defined `void iterate()` function at the configured iteration rate. This function should be implemented in the node inheriting the control node and should contain the main control algorithm calculations by taking the inputs and setpoints and publishing an output. The most recently received input and setpoint messages can be accessed through the following funcitons:

```c++
template <typename T>
T get_input(std::string topic_name)
    
template <typename T>
T get_setpoint(std::string topic_name)
```
Additionally, the convenience function
```c++
double get_float64_setpoint(std::string topic_name, double disabled_value=NAN)
```
can also be used if the setpoint is a `avl_control/Float64Setpoint` message. The `disabled_value` argument specifies the value that will be returned by the function if the setpoint is disabled.

After getting the most recent input and setpoint messages, the user-defined `void iterate()` function should calculate and publish one or more output messages.

If all added setpoints become disabled, the user-defined `void disable()` function will be called once. This function should be implemented in the node inheriting the control node and should contain any logic to reset the control algorithm or any logic that is run when the controller is disabled, such as publishing a disable setpoint to any cascaded control nodes.
