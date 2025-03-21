# dsr_python

## Overview

This package contains utility functions and classes for the DSR ROS stack using the Python API, similar to the `dsr_util` package.

* **`NodeAgent:`** This class inherits from `rclcpp_lifecycle::LifecycleNode` and is used to create a lifecycle ROS 2 node that can communicate with the DSR. It contains a pointer to the `DSRGraph` object that is used to communicate with the DSR. It also contains a `Service` to save the graph to a file. By default all nodes created with this templates will have the priority 0. This means that the nodes will only publish if there are no other nodes with the same name and priority 1 or higher. The user can also specify the priority of the nodes by update the `priority` attribute to the message.
