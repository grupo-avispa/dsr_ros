# dsr_agents

## Overview

This package provides ROS 2 agents for connecting to CORTEX architecture using a Deep State Representation (DSR) graph. The package enables the user to easily interface with CORTEX-based systems through a ROS 2 middleware.

This package feature a template for creating agents that can be used to publish data in the DSR:
* **`Topic agent:`** which allows the user to subscribe to a generic topic and publish it in the DSR. This node is useful for users who need to integrate different types of sensors into their CORTEX system.

By default all nodes created with this templates will have the priority 0. This means that the nodes will only publish if there are no other nodes with the same name and priority 1 or higher. The user can also specify the priority of the nodes by update the `priority` attribute to the message.

Along with the template mentioned above, this package also provides the next agents that can be used to publish data in the DSR:
* **`Docking agent:`** which allows the user to call the docking action from the DSR.
* **`Navigation agent:`**  which allows the user to call the navigation action from the DSR.
* **`TF agent:`** which publishes the transformation tree as nodes in the DSR. This allows the user to have a visual representation of the transformations between different reference frames in their CORTEX system.

**Keywords:** ROS2, cortex, dsr, deep space representation

**Author: Alberto Tudela, Oscar Pons Fernández, José Galeas Merchan<br />**

The dsr_agent package has been tested under [ROS2] Humble on [Ubuntu] 22.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Usage

To use this package, first modify the `default_params.yaml` file located in the `params` directory to set the configuration options for the generic_agent node.

Next, launch the package using the provided launch file `default.launch.py`. This file can be found in the launch directory of the package. To launch the package with default settings, use the following command:

	ros2 launch dsr_agent default.launch.py

The package comes with an example configuration file called `robot_params.yaml` located in the `params` directory and an example launch file called `robot.launch.py` located in the launch directory. These files can be used as a starting point for creating custom configurations and launch files.

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

Agent that wait until a `dock` node is created in the DSR and then start the docking process.

#### Actions

* **`dock_robot`**  ([opennav_docking_msgs/action/DockRobot])

	Action to send the robot to the docking station.

### nav_agent

Agent that wait until a `move` node is created in the DSR and then start the navigation. The `move` node must be created with the attributes `goal_x`, `goal_y` and `goal_angle` that contains the goal pose for the robot.

#### Actions

* **`navigate_to_pose`**  ([nav2_msgs/action/NavigateToPose])

	Action to send a navigation goal from the DSR.
### tf_agent

Agent that publishes the transformations tree as RT nodes in the DSR. This allows the user to have a visual representation of the transformations between different reference frames in their CORTEX system. The frame `base_link` is replaced by the `robot` frame in the DSR and the frame `map` is replaced by the `world` frame in the DSR.

#### Subscribed Topics

* **`tf`**  ([tf2_msgs/TFMessage])

	The dynamic transformation tree.

* **`tf_static`**  ([tf2_msgs/TFMessage])

	The static transformation tree.

## Future work
- [ ] Convert nodes to LifeCycleNodes.
- [x] Inherit from a generic agent class.

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/humble/
[tf2_msgs/TFMessage]: http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html
[nav2_msgs/action/NavigateToPose]: hhttps://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
[opennav_docking_msgs/action/DockRobot]: https://github.com/open-navigation/opennav_docking/blob/main/opennav_docking_msgs/action/DockRobot.action