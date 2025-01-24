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

// C++
#include <iterator>
#include <string>

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_util/helpers.hpp"
#include "dsr_bridge/dsr_bridge.hpp"
#include "rclcpp/qos.hpp"

namespace dsr_bridge
{

DSRBridge::DSRBridge(const rclcpp::NodeOptions & options)
: dsr_util::NodeAgent("dsr_bridge", options)
{
}

DSRBridge::~DSRBridge()
{
  lost_edges_.clear();
}

dsr_util::CallbackReturn DSRBridge::on_configure(const rclcpp_lifecycle::State & state)
{
  // ROS parameters
  declare_parameter_if_not_declared(
    this, "node_topic",
    rclcpp::ParameterValue("nodes"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The ROS topic to publish / subscribe the nodes to the DSR graph"));
  this->get_parameter("node_topic", node_topic_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter node_topic is set to: [%s]", node_topic_.c_str());

  declare_parameter_if_not_declared(
    this, "edge_topic",
    rclcpp::ParameterValue("edges"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The ROS topic to publish / subscribe the edges to the DSR graph"));
  this->get_parameter("edge_topic", edge_topic_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter edge_topic is set to: [%s]", edge_topic_.c_str());

  declare_parameter_if_not_declared(
    this, "include_nodes",
    rclcpp::ParameterValue(std::vector<std::string>{}),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("List of node types to be sent to the other DSR bridge"));
  this->get_parameter("include_nodes", include_nodes_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter include_nodes is set to: [%s]",
    std::accumulate(
      include_nodes_.begin(), include_nodes_.end(), std::string(),
      [](const std::string & a, const std::string & b) -> std::string {
        return a + (a.length() > 0 ? ", " : "") + b;
      }).c_str());

  std::vector<std::string> exclude;
  declare_parameter_if_not_declared(
    this, "exclude_nodes",
    rclcpp::ParameterValue(std::vector<std::string>{}),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("List of node types to be excluded from sending to the other DSR bridge"));
  this->get_parameter("exclude_nodes", exclude);

  if (!include_nodes_.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "The parameter include_nodes is not empty. It has priority over exclude.");
  } else {
    exclude_nodes_ = exclude;
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter exclude_nodes is set to: [%s]",
      std::accumulate(
        exclude_nodes_.begin(), exclude_nodes_.end(), std::string(),
        [](const std::string & a, const std::string & b) -> std::string {
          return a + (a.length() > 0 ? ", " : "") + b;
        }).c_str());
  }

  // Publisher to the other DSR bridge
  node_to_ros_pub_ = this->create_publisher<dsr_msgs::msg::Node>(node_topic_, 100);
  edge_to_ros_pub_ = this->create_publisher<dsr_msgs::msg::Edge>(edge_topic_, 100);

  // Subscriber to the external DSR graph
  node_from_ros_sub_ = this->create_subscription<dsr_msgs::msg::Node>(
    node_topic_, rclcpp::QoS(100).reliable(),
    std::bind(&DSRBridge::node_from_ros_callback, this, std::placeholders::_1));
  edge_from_ros_sub_ = this->create_subscription<dsr_msgs::msg::Edge>(
    edge_topic_, rclcpp::QoS(100).reliable(),
    std::bind(&DSRBridge::edge_from_ros_callback, this, std::placeholders::_1));

  // Service
  get_graph_service_ = this->create_service<GetGraph>(
    "~/get_graph", std::bind(
      &DSRBridge::get_graph_service, this, std::placeholders::_1, std::placeholders::_2));

  return NodeAgent::on_configure(state);
}

dsr_util::CallbackReturn DSRBridge::on_activate(const rclcpp_lifecycle::State & state)
{
  // Timer to synchronize the graph the first time
  one_off_sync_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&DSRBridge::sync_graph, this));

  return NodeAgent::on_activate(state);
}

dsr_util::CallbackReturn DSRBridge::on_deactivate(const rclcpp_lifecycle::State & state)
{
  one_off_sync_timer_.reset();
  return NodeAgent::on_deactivate(state);
}

dsr_util::CallbackReturn DSRBridge::on_cleanup(const rclcpp_lifecycle::State & state)
{
  node_to_ros_pub_.reset();
  edge_to_ros_pub_.reset();

  return NodeAgent::on_cleanup(state);
}

dsr_util::CallbackReturn DSRBridge::on_shutdown(const rclcpp_lifecycle::State & state)
{
  node_from_ros_sub_.reset();
  edge_from_ros_sub_.reset();

  return NodeAgent::on_shutdown(state);
}

void DSRBridge::node_from_ros_callback(const dsr_msgs::msg::Node::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(
    this->get_logger(),
    "Subscribed to nodes topic from [%s]", msg->header.frame_id.c_str());

  // The message comes from the same name, ignore it
  if (msg->header.frame_id == source_) {
    return;
  }

  // Create update or delete the current node
  if (!msg->deleted) {
    // Update the node
    if (auto node = G_->get_node(msg->name); node.has_value()) {
      dsr_util::helpers::modify_attributes_from_string(node.value(), msg->attributes);
      if (G_->update_node(node.value())) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Updated [%s] node successfully of type [%s] in the DSR",
          msg->name.c_str(), msg->type.c_str());
      }
      // Create the node
    } else {
      auto new_node = from_msg(*msg);
      G_->add_or_modify_attrib_local<timestamp_alivetime_att>(new_node, static_cast<uint64_t>(std::time(nullptr)));
      if (auto id = G_->insert_node(new_node); id.has_value()) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Inserted [%s] node successfully of type [%s] in the DSR",
          msg->name.c_str(), msg->type.c_str());
      }
    }
    // Delete the node
  } else {
    delete_node(msg->name);
  }

  insert_lost_edges();
}

void DSRBridge::edge_from_ros_callback(const dsr_msgs::msg::Edge::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(
    this->get_logger(), "Subscribed to edges topic from [%s]", msg->header.frame_id.c_str());

  // The message comes from the same name, ignore it
  if (msg->header.frame_id == source_) {
    return;
  }

  // Create or update the current edge
  if (!msg->deleted) {
    // Store the edge until the nodes are created
    auto parent_node = G_->get_node(msg->parent);
    auto child_node = G_->get_node(msg->child);
    if (!parent_node.has_value() || !child_node.has_value()) {
      lost_edges_.push_back(*msg);
      RCLCPP_WARN(
        this->get_logger(),
        "The edge [%s->%s] of type [%s] has been stored until the nodes are created",
        msg->parent.c_str(), msg->child.c_str(), msg->type.c_str());
    } else {
      // Update the edge attributes or create it
      DSR::Edge new_edge;
      if (auto edge = G_->get_edge(msg->parent, msg->child, msg->type); edge.has_value()) {
        dsr_util::helpers::modify_attributes_from_string(edge.value(), msg->attributes);
        new_edge = edge.value();
      } else {
        new_edge = from_msg(*msg);
      }
      // Insert the edge in the DSR graph
      if (G_->insert_or_assign_edge(new_edge)) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Inserted or updated edge [%s->%s] of type [%s] in the DSR",
          msg->parent.c_str(), msg->child.c_str(), msg->type.c_str());
      }
    }
    // Delete the edge
  } else {
    delete_edge(msg->parent, msg->child, msg->type);
  }
}

bool DSRBridge::get_graph_service(
  const std::shared_ptr<GetGraph::Request>/*request*/, std::shared_ptr<GetGraph::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Incoming service request to get the graph");

  std::vector<dsr_msgs::msg::Node> nodes_msg;
  std::vector<dsr_msgs::msg::Edge> edges_msg;
  get_graph_from_dsr(nodes_msg, edges_msg);

  response->nodes = nodes_msg;
  response->edges = edges_msg;
  return true;
}

void DSRBridge::node_created(std::uint64_t id, const std::string & /*type*/)
{
  // Filter the nodes that comes from the same source
  if (auto dsr_node = G_->get_node(id); dsr_node.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value());
      (source.has_value() && source.value() == source_))
    {
      // Check if the node type is in the include or exclude list, or if both are empty
      if ((include_nodes_.empty() && exclude_nodes_.empty()) ||
        (!include_nodes_.empty() &&
        std::find(
          include_nodes_.begin(), include_nodes_.end(),
          dsr_node.value().type()) != include_nodes_.end()) ||
        (!exclude_nodes_.empty() &&
        std::find(
          exclude_nodes_.begin(), exclude_nodes_.end(),
          dsr_node.value().type()) == exclude_nodes_.end()))
      {
        node_to_ros_pub_->publish(to_msg(dsr_node.value()));
      }
    }
  }
}

void DSRBridge::node_attr_updated(uint64_t id, const std::vector<std::string> & att_names)
{
  // Filter the nodes that comes from the same source
  if (auto dsr_node = G_->get_node(id); dsr_node.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value());
      (source.has_value() && source.value() == source_))
    {
      // Get all the updated attributes
      auto node_msg = to_msg(dsr_node.value());
      node_msg.attributes =
        dsr_util::helpers::attributes_to_string_by_names(dsr_node.value(), att_names);
      node_to_ros_pub_->publish(node_msg);
    }
  }
}

void DSRBridge::edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type)
{
  // Filter the edges that comes from the same source
  if (auto dsr_edge = G_->get_edge(from, to, type); dsr_edge.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_edge.value());
      (source.has_value() && source.value() == source_))
    {
      edge_to_ros_pub_->publish(to_msg(dsr_edge.value()));
    }
  }
}

void DSRBridge::edge_attr_updated(
  std::uint64_t /*from*/, std::uint64_t /*to*/,
  const std::string & /*type*/, const std::vector<std::string> & /*att_names*/)
{
  // TODO(ajtudela): Implement this method
  /*
  // Filter the edges that comes from the same source
  if (auto dsr_edge = G_->get_edge(from, to, type); dsr_edge.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_edge.value());
      (source.has_value() && source.value() == source_))
    {
      // Get all the updated attributes
      auto edge_msg = to_msg(dsr_edge.value());
      edge_msg.attributes =
        dsr_util::helpers::attributes_to_string_by_names(dsr_edge.value(), att_names);
      edge_to_ros_pub_->publish(edge_msg);
    }
  }
  */
}

void DSRBridge::node_deleted_by_node(const DSR::Node & node)
{
  node_to_ros_pub_->publish(to_msg(node, true));
}

void DSRBridge::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string & edge_tag)
{
  dsr_msgs::msg::Edge edge_msg;
  edge_msg.header.stamp = this->now();
  edge_msg.header.frame_id = source_;
  edge_msg.parent = G_->get_node(from).has_value() ? G_->get_node(from).value().name() : "";
  edge_msg.child = G_->get_node(to).has_value() ? G_->get_node(to).value().name() : "";
  edge_msg.type = edge_tag;
  edge_msg.deleted = true;
  edge_to_ros_pub_->publish(edge_msg);
}

DSR::Node DSRBridge::from_msg(const dsr_msgs::msg::Node & msg)
{
  if (!node_types::check_type(msg.type)) {
    throw std::runtime_error("Error, [" + msg.type + "] is not a valid node type");
  }

  DSR::Node new_node;
  new_node.name(msg.name);
  new_node.type(msg.type);
  dsr_util::helpers::modify_attributes_from_string(new_node, msg.attributes);
  return new_node;
}

dsr_msgs::msg::Node DSRBridge::to_msg(const DSR::Node & node, bool deleted)
{
  dsr_msgs::msg::Node node_msg;
  node_msg.header.stamp = this->now();
  node_msg.header.frame_id = source_;
  node_msg.name = node.name();
  node_msg.type = node.type();
  node_msg.attributes = dsr_util::helpers::attributes_to_string(node.attrs());
  node_msg.deleted = deleted;
  return node_msg;
}

DSR::Edge DSRBridge::from_msg(const dsr_msgs::msg::Edge & msg)
{
  if (!edge_types::check_type(msg.type)) {
    throw std::runtime_error("Error, [" + msg.type + "] is not a valid edge type");
  }

  DSR::Edge new_edge;
  auto parent_node = G_->get_node(msg.parent);
  auto child_node = G_->get_node(msg.child);
  if (parent_node.has_value() && child_node.has_value()) {
    new_edge.from(parent_node.value().id());
    new_edge.to(child_node.value().id());
    new_edge.type(msg.type);
    dsr_util::helpers::modify_attributes_from_string(new_edge, msg.attributes);
  }
  return new_edge;
}

dsr_msgs::msg::Edge DSRBridge::to_msg(const DSR::Edge & edge, bool deleted)
{
  dsr_msgs::msg::Edge edge_msg;
  auto parent_node = G_->get_node(edge.from());
  auto child_node = G_->get_node(edge.to());
  if (parent_node.has_value() && child_node.has_value()) {
    edge_msg.header.stamp = this->now();
    edge_msg.header.frame_id = source_;
    edge_msg.parent = parent_node.value().name();
    edge_msg.child = child_node.value().name();
    edge_msg.type = edge.type();
    edge_msg.attributes = dsr_util::helpers::attributes_to_string(edge.attrs());
    edge_msg.deleted = deleted;
  }
  return edge_msg;
}

void DSRBridge::insert_lost_edges()
{
  std::vector<dsr_msgs::msg::Edge> to_delete;
  for (auto & msg : lost_edges_) {
    auto parent_node = G_->get_node(msg.parent);
    auto child_node = G_->get_node(msg.child);
    if (parent_node.has_value() && child_node.has_value()) {
      auto new_edge = from_msg(msg);
      if (G_->insert_or_assign_edge(new_edge)) {
        to_delete.push_back(msg);
        RCLCPP_INFO(
          this->get_logger(),
          "Inserted lost edge [%s->%s] of type [%s] in the DSR",
          msg.parent.c_str(), msg.child.c_str(), msg.type.c_str());
      }
    }
  }

  for (auto & msg : to_delete) {
    lost_edges_.erase(
      std::remove_if(
        lost_edges_.begin(), lost_edges_.end(), [&msg](const dsr_msgs::msg::Edge & edge) {
          return edge == msg;
        }),
      lost_edges_.end());
  }
}

void DSRBridge::get_graph_from_dsr(
  std::vector<dsr_msgs::msg::Node> & nodes_msg, std::vector<dsr_msgs::msg::Edge> & edges_msg)
{
  // Get the nodes and edges from the DSR graph
  for (auto & node : G_->get_nodes()) {
    nodes_msg.push_back(to_msg(node));
    if (auto edges = G_->get_edges(node.id()); edges.has_value()) {
      for (const auto & edge_pair : edges.value()) {
        auto edge_msg = to_msg(edge_pair.second);
        edges_msg.push_back(edge_msg);
      }
    }
  }

  // Reverse the nodes
  std::reverse(nodes_msg.begin(), nodes_msg.end());
}

void DSRBridge::sync_graph()
{
  RCLCPP_INFO(this->get_logger(), "Synchronizing the graph ...");

  for (const auto & service : this->get_service_names_and_types() ) {
    // Check if the service name ends with "/get_graph" and is not the same as the current one
    const auto & service_name = service.first;
    auto current_name = "/" + std::string(this->get_name()) + "/get_graph";
    if (service_name.ends_with("/get_graph") && service_name != current_name) {
      RCLCPP_INFO(this->get_logger(), "Found get_graph service: %s", service_name.c_str());
      // Use this service name to create the client
      get_graph_client_ = this->create_client<dsr_msgs::srv::GetGraph>(service_name);
      if (!get_graph_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "get_graph service not found");
        return;
      }
      // Send the request
      auto req = std::make_shared<dsr_msgs::srv::GetGraph::Request>();
      auto result = get_graph_client_->async_send_request(
        req,
        [this](rclcpp::Client<dsr_msgs::srv::GetGraph>::SharedFuture response) {
          // Insert the nodes and edges in the DSR graph
          for (auto & node_msg : response.get()->nodes) {
            node_from_ros_callback(std::make_shared<dsr_msgs::msg::Node>(node_msg));
          }
          for (auto & edge_msg : response.get()->edges) {
            edge_from_ros_callback(std::make_shared<dsr_msgs::msg::Edge>(edge_msg));
          }
          RCLCPP_INFO(this->get_logger(), "Graph synchronized");
        });
      // Cancel the timer
      this->one_off_sync_timer_->cancel();
      return;
    }
  }

  // Cancel the timer if the service is not found
  RCLCPP_WARN(this->get_logger(), "get_graph service not found");
  this->one_off_sync_timer_->cancel();
}

}  // namespace dsr_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_bridge::DSRBridge)
