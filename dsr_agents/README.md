# dsr_agents
![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)

## Overview

This package provides ROS 2 agents for connecting to CORTEX architecture using a Deep State Representation (DSR) graph. The package enables the user to easily interface with CORTEX-based systems through a ROS 2 middleware.

This package feature two templates for creating agents that can be used to publish data in the DSR:
* **`Topic agent:`** which allows the user to subscribe to a generic topic and publish it in the DSR. This node is useful for users who need to integrate different types of sensors into their CORTEX system.
* **`Action agent:`** which allows the user to communicate with the DSR using ROS 2 actions.

By default all nodes created with this templates will have the priority 0. This means that the nodes will only publish if there are no other nodes with the same name and priority 1 or higher. The user can also specify the priority of the nodes by update the `priority` attribute to the message.

Along with the templates mentioned above, this package also provides three agents that can be used to publish data in the DSR:
* **`Navigation agent:`**  which allows the user to publish the navigation data in the DSR.
* **`TF agent:`** which publishes the transformation tree as nodes in the DSR. This allows the user to have a visual representation of the transformations between different reference frames in their CORTEX system.
* **`Person agent:`** which publishes the detected people in the DSR.

**Keywords:** ROS2, cortex, dsr, deep space representation

**Author: Alberto Tudela<br />**

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

### nav_agent

Agent that wait until a `move` node is created in the DSR and then start the navigation. The `move` node must be created with an attribute called `zone` that contains the name of the zone where the robot should navigate or with the attributes `goal_x`, `goal_y` and `goal_angle` that contains the goal pose for the robot.

#### Actions

* **`navigate_to_pose`**  ([nav2_msgs/action/NavigateToPose])

	Action to send a navigation goal from the DSR.

* **`dock`**  ([auto_docking_interfaces/action/Dock])

	Action to send a docking goal from the DSR.

* **`semantic_goals`**  ([semantic_navigation_msgs/action/SemanticGoals])

	Action to generate a random pose from a region of interest.

### tf_agent

Agent that publishes the transformations tree as RT nodes in the DSR. This allows the user to have a visual representation of the transformations between different reference frames in their CORTEX system. The frame `base_link` is replaced by the `robot` frame in the DSR and the frame `map` is replaced by the `world` frame in the DSR.

#### Subscribed Topics

* **`tf`**  ([tf2_msgs/TFMessage])

	The dynamic transformation tree.

* **`tf_static`**  ([tf2_msgs/TFMessage])

	The static transformation tree.

### person_agent

Agent that publishes the detected people in the DSR.

#### Subscribed Topics

* **`ros_topic`**  ([vision_msgs/Detection3DArray])

	The detected people in 3D format.

#### Parameters

* **`timeout`** (int default: 30)

	The time in seconds that the agent will wait for a new message before it removes the node from the DSR.

### Common parameters for all nodes

* **`agent_id`** (int, default: 0)

	A unique identifier for the agent.

* **`dsr_input_file`** (string, default: "") (Optional)

	The path to the DSR file that will be loaded.

## Future work
- [ ] Convert nodes to LifeCycleNodes.
- [x] Inherit from a generic agent class.
- [ ] Finish the action agent.
- [ ] Replace the Detection3DArray message with a custom people message.

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/humble/
[tf2_msgs/TFMessage]: http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html
[nav2_msgs/action/NavigateToPose]: hhttps://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
[auto_docking_interfaces/action/Dock]: https://gitlab.com/grupo-avispa/ros/docking/-/blob/dev/auto_docking_interfaces/action/Dock.action
[semantic_navigation_msgs/action/SemanticGoals]: https://gitlab.com/grupo-avispa/ros/semantic_navigation/-/blob/dev/semantic_navigation_msgs/srv/SemanticGoals.srv
[vision_msgs/Detection3DArray]: http://docs.ros.org/api/vision_msgs/html/msg/Detection3DArray.html
