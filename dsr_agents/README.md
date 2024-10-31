# dsr_agents

## Overview

This package provides ROS 2 agents designed to facilitate connectivity with the CORTEX architecture through the DSR graph, enabling seamless integration between CORTEX-based systems and the ROS 2 middleware. This functionality simplifies the user's ability to interact with CORTEX-based systems and supports sensor data integration and robot control.

A core feature of the package is a template for creating agents that publish data to the DSR:
* **`Topic agent:`** this agent enables the user to subscribe to a generic topic and publish its data in the DSR, supporting the integration of various sensor types into a CORTEX system.

In addition to the generic template, depending on the specific task for which they are implemented, there are specialized agents for data publications in the DSR:
* **`Docking agent:`** initiate docking actions directly within the DSR.
* **`Navigation agent:`**  facilitates the execution of navigation commands from the DSR.
* **`TF agent:`** publishes the transformation tree as nodes in the DSR, providing a visual representation of transformations across different reference frames in the CORTEX environment.

**Keywords:** ROS2, cortex, dsr, deep space representation

**Author: Alberto Tudela, Oscar Pons Fernández, José Galeas Merchan<br />**

The dsr_agents package has been tested under [ROS2] Humble on [Ubuntu] 22.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Nodes

### topic_agent

Agent that subscribe to a generic topic in ROS 2 and publishes it as a node in the DSR.

#### Subscribed Topics

* **`ros_topic`**

	The name of the topic that the node should subscribe to.

#### Parameters

* **`dsr_node_name`** (string, default: "")

	The name of the node in the DSR where the sensor data should be published.

* **`dsr_parent_node_name`** (string, default: "") (Optional)

	String that specifies the name of the parent node in the DSR where the `dsr_node_name` should be attached to. If left empty, the node will be attached to frame_id from the ROS header message.

### docking_agent

Agent that wait until a `dock` node is created in the DSR and then start the docking process. The `dock` node must be created with the attribute `dock_id` that contains the id of the docking station where the robot should dock.

#### Actions

* **`dock_robot`**  ([opennav_docking_msgs/action/DockRobot])

	Action to send the robot to the docking station.

#### Parameters

* **`dsr_action_name`** (string, default: "")

	The name of the node in the DSR where the action should be listened.

### nav_agent

Agent that wait until a `move` node is created in the DSR and then start the navigation. The `move` node must be created with the attributes `goal_x`, `goal_y` and `goal_angle` that contains the goal pose for the robot.

#### Actions

* **`navigate_to_pose`**  ([nav2_msgs/action/NavigateToPose])

	Action to send a navigation goal from the DSR.

#### Parameters

* **`dsr_action_name`** (string, default: "")

	The name of the node in the DSR where the action should be listened.

### tf_agent

Agent that publishes the transformations tree as RT nodes in the DSR. This allows the user to have a visual representation of the transformations between different reference frames in their CORTEX system. The frame `base_link` is replaced by the `robot` frame in the DSR and the frame `map` is replaced by the `world` frame in the DSR.

#### Subscribed Topics

* **`tf`**  ([tf2_msgs/TFMessage])

	The dynamic transformation tree.

* **`tf_static`**  ([tf2_msgs/TFMessage])

	The static transformation tree.


[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/humble/
[tf2_msgs/TFMessage]: http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html
[nav2_msgs/action/NavigateToPose]: hhttps://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
[opennav_docking_msgs/action/DockRobot]: https://github.com/open-navigation/opennav_docking/blob/main/opennav_docking_msgs/action/DockRobot.action