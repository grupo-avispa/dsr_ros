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

from dsr_msgs.srv import SaveDSR
from pydsr import DSRGraph, Edge as DSREdge, Node as DSRNode, rt_api, signals
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


class NodeAgent(Node):
    """
    Base class to connect the DSR graph with ROS 2.

    It contains common methods and attributes
    to send data from ROS 2 to the DSR graph and vice versa.
    All agents must inherit from this class.
    """

    def __init__(self, node_name='node_agent', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node.

        :param state: State of the node.
        :return: TransitionCallbackReturn.SUCCESS if successful.
        """
        # Get ROS parameters
        self.__get_common_params()

        # Create graph
        try:
            self.g = DSRGraph(0, self.agent_name, self.agent_id)
        except RuntimeError as e:
            self.get_logger().info(f'Could not create graph: {e}')
            self.on_cleanup(state)
            return TransitionCallbackReturn.FAILURE

        # Get RT API
        self.rt = rt_api(self.g)

        # Create service to save the DSR graph
        self.save_dsr_service = self.create_service(SaveDSR, 'save_dsr', self.__save_dsr)

        # Add connection signals
        try:
            signals.connect(self.g, signals.UPDATE_NODE, self.node_updated)
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.node_attr_updated)
            signals.connect(self.g, signals.UPDATE_EDGE, self.edge_updated)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.edge_attr_updated)
            signals.connect(self.g, signals.DELETE_NODE, self.node_deleted)
            signals.connect(self.g, signals.DELETE_EDGE, self.edge_deleted)
        except RuntimeError as e:
            self.get_logger().info(f'Could not create signals: {e}')

        self.get_logger().info('Configured agent node')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Activate the node.

        :param state: State of the node.
        :return: TransitionCallbackReturn.SUCCESS if successful.
        """
        super().on_activate(state)
        self.get_logger().info('Activating the node...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Deactivate the node.

        :param state: State of the node.
        :return: TransitionCallbackReturn.SUCCESS if successful.
        """
        super().on_deactivate(state)
        self.get_logger().info('Deactivating the node...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node.

        :param state: State of the node.
        :return: TransitionCallbackReturn.SUCCESS if successful.
        """
        self.get_logger().info('Cleaning the node...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node.

        :param state: State of the node.
        :return: TransitionCallbackReturn.SUCCESS if successful.
        """
        self.get_logger().info(f'Shutting the node from state {state.label}.')
        return TransitionCallbackReturn.SUCCESS

    def __declare_parameter_if_not_declared(self, param_name: str, default_value,
                                            descriptor: ParameterDescriptor = None):
        """
        Declare a static ROS 2 parameter and sets it to a given value \
        if it was not already declared.

        :param param_name: The name of the parameter.
        :param default_value: Parameter value to initialize with.
        :param descriptor: Parameter descriptor (optional).
        """
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, default_value, descriptor)

    def __get_common_params(self):
        """Initialize ROS parameters."""
        # Agent parameters
        self.__declare_parameter_if_not_declared(
            'agent_name', 'agent', ParameterDescriptor(description='The name of the agent'))
        self.agent_name = self.get_parameter('agent_name').get_parameter_value().string_value
        self.get_logger().info(f'The parameter agent_name is set to: [{self.agent_name}]')

        self.__declare_parameter_if_not_declared(
            'agent_id', 0, ParameterDescriptor(description='The id of the agent'))
        self.agent_id = self.get_parameter('agent_id').get_parameter_value().integer_value
        self.get_logger().info(f'The parameter agent_id is set to: [{self.agent_id}]')

        # Other parameters
        self.__declare_parameter_if_not_declared(
            'source', 'robot', ParameterDescriptor(description='Physical source of the agent'))
        self.source = self.get_parameter('source').get_parameter_value().string_value
        self.get_logger().info(f'The parameter source_ is set to: [{self.source}]')

    def __update_node_with_source(self, node: DSRNode) -> bool:
        """
        Update the attributes of the given edge in the DSR graph \
        and change the source attribute to the name of the physical machine (set by source).

        :param
        node: The DSR node.
        :return: True if the node was updated successfully, False otherwise.
        """
        # TODO(ajtudela): Implement add_or_modify_attri_local
        return self.g.update_node(node)

    def __update_edge_with_source(self, edge: DSREdge) -> bool:
        """
        Update the attributes of the given edge in the DSR graph \
        and change the source attribute to the name of the physical machine (set by source).

        :param edge: The DSR edge.
        """
        # TODO(ajtudela): Implement add_or_modify_attri_local
        return self.g.insert_or_assign_edge(edge)

    def __save_dsr(self, request, response):
        """
        Save the DSR graph into a file.

        :param request: URL of the file.
        :param response: True if the DSR graph was saved successfully, false otherwise.
        """
        self.g.write_to_json_file(request.url)
        response.result = True

    def node_updated(self, node_id: int, node_type: str):
        """
        Handle the event when a node is updated in the DSR graph.

        :param node_id: The id of the node.
        :param node_type: The type of the node.
        """
        pass

    def node_attr_updated(self, node_id: int, att_names: [str]):
        """
        Handle the event when a node attribute is updated in the DSR graph.

        :param node_id: The id of the node.
        :param att_names: The names of the attributes updated.
        """
        pass

    def edge_updated(self, fr: int, to: int, edge_type: str):
        """
        Handle the event when an edge is updated in the DSR graph.

        :param fr: The id of the parent node.
        :param to: The id of the child node.
        :param edge_type: The type of the edge.
        """
        pass

    def edge_attr_updated(self, fr: int, to: int, edge_type: str, att_names: [str]):
        """
        Handle the event when an edge attribute is updated in the DSR graph.

        :param fr: The id of the parent node.
        :param to: The id of the child node.
        :param edge_type: The type of the edge.
        :param att_names: The names of the attributes updated.
        """
        pass

    def node_deleted(self, node_id: int):
        """
        Handle the event when a node is deleted in the DSR graph.

        :param node_id: The id of the node.
        """
        pass

    def edge_deleted(self, fr: int, to: int, edge_type: str):
        """
        Handle the event when an edge is deleted in the DSR graph.

        :param fr: The id of the parent node.
        :param to: The id of the child node.
        :param edge_type: The type of the edge.
        """
        pass

    def add_node(self, node_name: str, node_type: str) -> DSRNode:
        """
        Add a node into the DSR graph with the given name and type.

        By default, all nodes have a low priority (0) and the source attribute is set
        to the name of the physical machine.

        :param node_name: Name of the DSR node.
        :param node_type: The type of the DSR node.
        :return: The DSR node if it was added successfully.
        """
        # Create the node
        new_node = DSRNode(self.agent_id, node_name, node_type)
        # TODO(ajtudela): Update the timestamp
        # Insert the node into the DSR graph
        if self.g.insert_node(new_node):
            self.get_logger().info(
                f'Inserted node [{node_name}] successfully of type [{node_type}]')
        else:
            self.get_logger().error(f'The node [{node_name}] could not be inserted')
        return new_node

    def add_edge(self, from_node: int, to_node: int, edge_type: str) -> DSREdge:
        """
        Add an edge into the DSR graph with the given parent and child nodes id.

        By default, all edges have the source attribute set to the name of the physical machine.

        :param from_node: Id of the parent DSR node.
        :param to_node: Id of the child DSR node.
        :param edge_type: The type of the DSR edge.
        :return: The DSR edge if it was added successfully.
        """
        parent_node = self.g.get_node(from_node)
        child_node = self.g.get_node(to_node)
        if parent_node is not None and child_node is not None:
            # Create the edge
            new_edge = DSREdge(to_node, from_node, edge_type, self.agent_id)
            # TODO(ajtudela): Update the timestamp
            # Insert the edge into the DSR graph
            if self.g.insert_or_assign_edge(new_edge):
                self.get_logger().info(
                    f'Inserted edge [{from_node} -> {to_node}] successfully of type [{edge_type}]')
            else:
                self.get_logger().error(
                    f'The edge [{from_node} -> {to_node}] could not be inserted')
