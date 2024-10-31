# dsr_util

## Overview

This package contains utility functions and classes for the DSR ROS stack:

* **`ActionAgent:`** This class is a wrapper around the `rclcpp_action` object that provides a more user-friendly interface to call ROS 2 actions from the DSR graph. This class has callbacks that listen a 'wants_to' edge between the robot and the action node. When the robot wants to start the action, the agent calls the ROS 2 action server and waits for the result. The agent also listens to 'abort' and 'cancel' edges to cancel the action.

* **`AgentNode:`** This class inherits from `rclcpp_lifecycle::LifecycleNode` and is used to create a lifecycle ROS 2 node that can communicate with the DSR. It contains a pointer to the `DSRGraph` object that is used to communicate with the DSR. It also contains a `Service` to save the graph to a file. By default all nodes created with this templates will have the priority 0. This means that the nodes will only publish if there are no other nodes with the same name and priority 1 or higher. The user can also specify the priority of the nodes by update the `priority` attribute to the message.

* **`DSRGraphExt:`** This class is a wrapper around the `DSRGraph` object that provides a more user-friendly interface to the DSR. It contains functions to add nodes, edges, and properties to the graph.

* **`QT Executor:`** This class is a custom executor that allows the user to run a QT application and a ROS 2 node in the same thread. This is useful for creating graphical interfaces that need to communicate with the ROS 2 middleware.

* **`ServiceAgent:`** This class is a wrapper around the `rclcpp::Service` object that provides a more user-friendly interface to call ROS 2 services from the DSR graph. This class has callbacks that listen a 'wants_to' edge between the robot and the service node. When the robot wants to call the service, the agent calls the ROS 2 service and waits for the result.

* **`ros_to_dsr_types.hpp:`** This file contains the conversion functions from ROS 2 messages to DSR types. The user must update this file to add new DSR types and add the conversion from ROS to DSR. You must update this file to add the new DSR types and add the conversion from ROS to DSR in the corresponding agent file.

There is also included a `worlds` folder that contains some worlds files used in the DSR examples.