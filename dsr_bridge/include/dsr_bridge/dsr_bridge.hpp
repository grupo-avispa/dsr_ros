// Copyright (c) 2024 Óscar Pons Fernández
// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Jose Miguel Galeas Merchan
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DSR_BRIDGE__DSR_BRIDGE_HPP_
#define DSR_BRIDGE__DSR_BRIDGE_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "dsr_msgs/msg/edge.hpp"
#include "dsr_msgs/msg/node.hpp"
#include "dsr_msgs/srv/get_graph.hpp"

// DSR
#include "dsr_util/node_agent.hpp"

namespace dsr_bridge
{

/**
 * @class dsr_bridge::DSRBridge
 * @brief Bridge to connect the DSR graphs between machines throughROS 2 topics.
 */
class DSRBridge : public dsr_util::NodeAgent
{
public:
  /**
   * @brief Construct a new DSRBridge object.
   *
   * @param options Node options
   */
  explicit DSRBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the DSRBridge object.
   */
  ~DSRBridge();

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  using GetGraph = dsr_msgs::srv::GetGraph;

  /**
   * @brief Callback executed when a node is received from a ROS 2 topic.
   *
   * @param msg The node received.
   */
  void node_from_ros_callback(const dsr_msgs::msg::Node::SharedPtr msg);

  /**
   * @brief Callback executed when an edge is received from a ROS 2 topic.
   *
   * @param msg The edge received.
   */
  void edge_from_ros_callback(const dsr_msgs::msg::Edge::SharedPtr msg);

  /**
   * @brief Callback executed when a node is created in the DSR graph.
   *
   * @param id The id of the node.
   * @param type The type of the node.
   */
  void node_created(std::uint64_t id, const std::string & type) override;

  /**
   * @brief Callback executed when a node attribute is updated in the DSR graph.
   *
   * @param id The id of the node.
   * @param att_names The names of the attributes updated.
   */
  void node_attr_updated(uint64_t id, const std::vector<std::string> & att_names) override;

  /**
   * @brief Callback executed when an edge is updated in the DSR graph.
   *
   * @param from The id of the parent node.
   * @param to The id of the child node.
   * @param type The type of the edge.
   */
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override;

  /**
   * @brief Callback executed when an edge attribute is updated in the DSR graph.
   *
   * @param from The id of the parent node.
   * @param to The id of the child node.
   * @param type The type of the edge.
   * @param att_names The names of the attributes updated.
   */
  void edge_attr_updated(
    std::uint64_t from, std::uint64_t to,
    const std::string & type, const std::vector<std::string> & att_names) override;

  /**
   * @brief Callback executed when a node is deleted in the DSR graph.
   *
   * @param node The node.
   */
  void node_deleted_by_node(const DSR::Node & node) override;

  /**
   * @brief Callback executed when an edge is deleted in the DSR graph.
   *
   * @param from The id of the parent node.
   * @param to The id of the child node.
   * @param edge_tag The type of the edge.
   */
  void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string & edge_tag);

  /**
   * @brief Callback executed when the service to get the graph is called.
   *
   * @param request Empty request.
   * @param response The graph in ROS 2 message format.
   * @return true If the graph was successfully sent.
   */
  bool get_graph_service(
    const std::shared_ptr<GetGraph::Request> request, std::shared_ptr<GetGraph::Response> response);

  /**
   * @brief Create a DSR::Node from a ROS 2 message.
   *
   * @param msg The ROS 2 message.
   * @return DSR::Node The DSR node created.
   */
  DSR::Node from_msg(const dsr_msgs::msg::Node & msg);

  /**
   * @brief Create a dsr_msgs::msg::Node from a DSR::Node.
   *
   * @param node The DSR node.
   * @param deleted If the node has to be marked as deleted.
   * @return dsr_msgs::msg::Node The ROS 2 message.
   */
  dsr_msgs::msg::Node to_msg(const DSR::Node & node, bool deleted = false);

  /**
   * @brief Create a DSR::Edge from a ROS 2 message.
   *
   * @param msg The ROS 2 message.
   * @return DSR::Edge The DSR Edge created.
   */
  DSR::Edge from_msg(const dsr_msgs::msg::Edge & msg);

  /**
   * @brief Create a dsr_msgs::msg::Edge from a DSR::Edge.
   *
   * @param edge The DSR edge.
   * @param deleted If the edge has to be marked as deleted.
   * @return dsr_msgs::msg::Edge The ROS 2 message.
   */
  dsr_msgs::msg::Edge to_msg(const DSR::Edge & edge, bool deleted = false);

  /**
   * @brief Insert the lost edges in the DSR graph.
   */
  void insert_lost_edges();

  /**
   * @brief Synchronize the DSR graph using the service to get the graph.
   */
  void sync_graph();

  /**
   * @brief Get the nodes and edges from the DSR graph.
   *
   * @param nodes_msg The nodes in the DSR graph in ROS 2 message format.
   * @param edges_msg The edges in the DSR graph in ROS 2 message format.
   */
  void get_graph_from_dsr(
    std::vector<dsr_msgs::msg::Node> & nodes_msg, std::vector<dsr_msgs::msg::Edge> & edges_msg);

  // Subscribers and publishers for the ROS 2 topics (nodes and edges)
  rclcpp::Subscription<dsr_msgs::msg::Node>::SharedPtr node_from_ros_sub_;
  rclcpp::Subscription<dsr_msgs::msg::Edge>::SharedPtr edge_from_ros_sub_;
  rclcpp_lifecycle::LifecyclePublisher<dsr_msgs::msg::Node>::SharedPtr node_to_ros_pub_;
  rclcpp_lifecycle::LifecyclePublisher<dsr_msgs::msg::Edge>::SharedPtr edge_to_ros_pub_;

  // Service to get the graph
  rclcpp::Service<GetGraph>::SharedPtr get_graph_service_;
  // Timer to synchronize the graph the first time
  rclcpp::TimerBase::SharedPtr one_off_sync_timer_;
  // ROS 2 topics names
  std::string node_topic_, edge_topic_;
  // Vector of lost edges
  std::vector<dsr_msgs::msg::Edge> lost_edges_;
  // List of node types to include or exclude
  std::vector<std::string> include_nodes_, exclude_nodes_;
};

}  // namespace dsr_bridge

#endif  // DSR_BRIDGE__DSR_BRIDGE_HPP_
