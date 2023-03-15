<div align="center">

# AVL Guidance Package
</div>

The `avl_guidance` package contains nodes implementing ROS action servers that a vehicle can use to execute actions such as line following, loitering, diving, etc. In general, a guidance node subscribes to vehicle navigation topics and receives a guidance action to be executed, and publishes controller setpoints to be handled by a control node. The `avl_guidance` package provides a `GuidanceNode` base class to simplify the implementation of guidance node logic.

<br/><br/><br/>
<div align="center">

## Guidance Node Base Class
</div>

The `GuidanceNode` base class provided by the `avl_guidance` package simplifies the implementation of a guidance node by handling the setup of a ROS action server, and providing functions that must be overridden for action start, iteration, and action stop. The guidance node base class operates with the `Mission` action definition where the result is a boolean indicating if the action completed successfully, and the feedback is a action completion percent. 

To implement a guidance node using the `GuidanceNode` base class, first inherit the `GuidanceNode` class and, in the child class's constructor, call the `GuidanceNode` base class constructor of the form
```c++
GuidanceNode(std::string server_name, int argc, char **argv)
```
where `server_name` is the name of the action server that will be used to receive ROS actions. For example, the dive guidance node constructor is declared as follows:
```c++
DiveGuidanceNode(int argc, char **argv) : GuidanceNode("guidance/DIVE", argc, argv)
```
where the action server name is set to `/guidance/DIVE`. The following table lists the functions that must be implemented by the class inheriting the `GuidanceNode` base class.


| Function&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | Description |
|:-|:-|
| `bool start_new_action(Action action)` | This function is called when a new action is received, and should implement all logic required to set up the execution of an action. It should return true if the action is valid and should be executed, or false if it is invalid and should not be executed. |
| `Feedback iterate_action()`            | If an action is being executed, this function will be called at the configured iteration rate. This function should contain the main guidance logic and should publish setpoints to the control nodes. If an action is finished, the `finish_action(bool success, Result result)` function should be called indicating if the action finished successfully or unsuccessfully. The function must return `Feedback` consisting of an action completion percentage. |
| `void stop_action()`                   | This function is called when action is canceled, times out, or ends because the `finish_action` function was called during iteration. It should contain any logic for ending an action such as sending setpoint disable messages to controllers. |


