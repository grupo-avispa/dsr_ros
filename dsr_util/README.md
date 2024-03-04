# dsr_util

![ROS2](https://img.shields.io/badge/ros2-humble-blue?logo=ros&logoColor=white)

## Overview

This package contains utility functions and classes for the DSR ROS stack.

* **`AgentNode:`** This class inherits from `rclcpp::Node` and is used to create a ROS 2 node that can communicate with the DSR. It contains a pointer to the `DSRGraph` object that is used to communicate with the DSR. It also contains a `Service` to save the graph to a file.

* **`QT Executor:`** This class is a custom executor that allows the user to run a QT application and a ROS 2 node in the same thread. This is useful for creating graphical interfaces that need to communicate with the ROS 2 middleware.