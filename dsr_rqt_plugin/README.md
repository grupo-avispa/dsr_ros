# dsr_rqt_plugin

![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)

## Overview

[RQT] plugin to visualize a Deep State Representation (DSR) graph. 

![DSR View](doc/dsr_rqt.png)
*View of the DSR with a world*

**Keywords:** ROS2, DSR, RQT

### License

**Author: Alberto Tudela<br />**

The dsr_rqt_plugin package has been tested under [ROS2] Humble on [Ubuntu] 22.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

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

Launch your DSR nodes as usual and then open RQT. You will see a new plugin called "DSR View".


[Ubuntu]: https://ubuntu.com/
[ROS2]: https://docs.ros.org/en/humble/
[RQT]: https://github.com/ros-visualization/rqt
