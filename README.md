# dsr_ros

![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)

## Overview

A ROS 2 stack that contains agents, interfaces and RQT plugins for connecting to CORTEX architecture using a Deep State Representation (DSR) graph.

 * [dsr_agents]: agents to connect to CORTEX architecture.
 * [dsr_interfaces]: messages and services to interact with the DSR agents and ROS.
 * [dsr_rqt_plugin]: RQT plugin to visualize a Deep State Representation (DSR) graph.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/humble/) (middleware for robotics)
- [Cortex](hhttps://github.com/robocomp/cortex) (DSR library)
- [FAST-DDS](https://github.com/eProsima/Fast-DDS) (eprosima Fast DDS)

#### Building

To build from source, clone the latest version of the repository into your colcon workspace and compile the package using the following commands:

	cd colcon_workspace/src
	git clone https://gitlab.com/grupo-avispa/ros/dsr_ros.git -b humble
	cd ../
	rosdep install -i --from-path src --rosdistro humble -y
	colcon build --symlink-install


[dsr_agents]: /dsr_agents
[dsr_interfaces]: /dsr_interfaces
[dsr_rqt_plugin]: /dsr_rqt_plugin