#! /usr/bin/env python3
# Copyright 2025 Alberto J. Tudela Roldán
# Copyright 2025 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

from dsr_python.node_agent import NodeAgent
from PySide6 import QtCore

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String


class DSRListener(NodeAgent):
    """
    This class is a ROS 2 listener node that subscribes to a topic and prints the messages.

    It is a subclass of the NodeAgent class, which provides the basic functionality for
    communicating with the DSR graph.
    """

    def __init__(self):
        """Initialize the DSRListener class."""
        super().__init__('dsr_listener')
        self.get_logger().info('DSRListener node started')

    def on_activate(self, state):
        """Activate the node."""
        self.get_logger().info('Activating DSRListener node')
        self.msg_sub = self.create_subscription(String, 'chatter', self.chatter_callback, 10)
        super().on_activate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info('Deactivating DSRListener node')
        super().on_deactivate(state)
        self.destroy_subscription(self.msg_sub)
        return TransitionCallbackReturn.SUCCESS

    def chatter_callback(self, msg):
        """Handle incoming messages from the chatter topic."""
        self.get_logger().info(f'Received message: {msg.data}')

    def node_deleted(self, node_id: int):
        """Handle node deletion."""
        self.get_logger().info(f'Deleting node {node_id}')


def main(args=None):
    app = QtCore.QCoreApplication(sys.argv)
    rclpy.init(args=args)
    node = DSRListener()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok():
        try:
            executor.wait_for_ready_callbacks(0)
            executor.spin_once()
        except Exception:
            pass
        app.processEvents()
    app.quit()
    executor.remove_node(node)


if __name__ == '__main__':
    main()
