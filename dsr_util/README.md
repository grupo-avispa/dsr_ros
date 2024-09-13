# dsr_util

## Overview

This package contains utility functions and classes for the DSR ROS stack:

* **`AgentNode:`** This class inherits from `rclcpp::Node` and is used to create a ROS 2 node that can communicate with the DSR. It contains a pointer to the `DSRGraph` object that is used to communicate with the DSR. It also contains a `Service` to save the graph to a file.

* **`ActionAgent:`** This class is a wrapper around the `rclcpp_action` object that provides a more user-friendly interface to call ROS 2 actions from the DSR graph. This class has callbacks that listen a 'wants_to' edge between the robot and the action node. When the robot wants to start the action, the agent calls the ROS 2 action server and waits for the result. The agent also listens to 'abort' and 'cancel' edges to cancel the action.

* **`DSRGraphExt:`** This class is a wrapper around the `DSRGraph` object that provides a more user-friendly interface to the DSR. It contains functions to add nodes, edges, and properties to the graph.

* **`QT Executor:`** This class is a custom executor that allows the user to run a QT application and a ROS 2 node in the same thread. This is useful for creating graphical interfaces that need to communicate with the ROS 2 middleware.

* **`ros_to_dsr_types.hpp:`** This file contains the conversion functions from ROS 2 messages to DSR types. The user must update this file to add new DSR types and add the conversion from ROS to DSR. You must update this file to add the new DSR types and add the conversion from ROS to DSR in the corresponding agent file.

There is also included a `worlds` folder that contains some worlds files used in the DSR examples.