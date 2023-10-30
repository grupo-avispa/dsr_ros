# dsr_agents
![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)

## Overview

This package provides ROS2 agents for connecting to CORTEX architecture using a Deep State Representation (DSR) graph. The package enables the user to easily interface with CORTEX-based systems through a ROS2 middleware.

This package features two types of nodes:
* **`Generic agent:`** which allows the user to subscribe to a sensor topic of any kind and publish it in the DSR. This node is useful for users who need to integrate different types of sensors into their CORTEX system.
* **`TF agent:`** which publishes the transformation tree as nodes in the DSR. This allows the user to have a visual representation of the transformations between different reference frames in their CORTEX system.

By default all nodes are created with low priority (0).

A DSR viewer is also included in this package. This viewer allows the user to visualize the DSR graph in a graphical interface. 

**Keywords:** ROS2, cortex, dsr, deep space representation

**Author: Alberto Tudela<br />**

The dsr_agent package has been tested under [ROS2] Humble on [Ubuntu] 22.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/humble/) (middleware for robotics)
- [Cortex](hhttps://github.com/robocomp/cortex) (DSR library)
- [FAST-DDS](https://github.com/eProsima/Fast-DDS) (eprosima Fast DDS)

#### Building

To build from source, clone the latest version from the main repository into your colcon workspace and compile the package using

	cd colcon_workspace/src
	git clone https://gitlab.com/grupo-avispa/ros/dsr_ros.git -b humble
	cd ../
	rosdep install -i --from-path src --rosdistro humble -y
	colcon build --symlink-install

## Usage

To use this package, first modify the `default_params.yaml` file located in the `params` directory to set the configuration options for the generic_agent node.

Next, launch the package using the provided launch file `default.launch.py`. This file can be found in the launch directory of the package. To launch the package with default settings, use the following command:

	ros2 launch dsr_agent default.launch.py

The package comes with an example configuration file called `robot_params.yaml` located in the `params` directory and an example launch file called `robot.launch.py` located in the launch directory. These files can be used as a starting point for creating custom configurations and launch files.

The viewer can be launched using the following command:

	ros2 launch dsr_agent dsr_viewer.launch.py

## Nodes

### generic_agent

Agent that subscribe to a generic topic and publishes it in the DSR.

#### Subscribed Topics

* **`ros_topic`**

	The name of the topic that the node should subscribe to.

#### Parameters

* **`agent_id`** (int, default: 0)

	A unique identifier for the agent.

* **`dsr_input_file`** (string, default: "") (Optional)

	The path to the DSR file that will be loaded.

* **`dsr_node_name`** (string, default: "")

	The name of the node in the DSR where the sensor data should be published.

* **`dsr_parent_node_name`** (string, default: "") (Optional)

	String that specifies the name of the parent node in the DSR where the `dsr_node_name` should be attached to. If left empty, the node will be attached to frame_id from the header message.

### tf_agent

Agent that publishes the transformation tree as nodes in the DSR.

#### Subscribed Topics

* **`tf`**  ([tf2_msgs/TFMessage])

	The dynamic transformation tree.

* **`tf_static`**  ([tf2_msgs/TFMessage])

	The static transformation tree.

#### Parameters

* **`agent_id`** (int, default: 0)

	A unique identifier for the agent.

* **`dsr_input_file`** (string, default: "") (Optional)

	The path to the DSR file that will be loaded.


## Future work
- [ ] Convert nodes to LifeCycleNodes.
- [x] Inherit from a generic agent class.

[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/humble/
[tf2_msgs/TFMessage]: https://docs.ros2.org/humble/api/tf2_msgs/msg/TFMessage.html