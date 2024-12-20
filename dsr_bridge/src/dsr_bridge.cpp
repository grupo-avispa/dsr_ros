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

namespace dsr_bridge
{

DSRBridge::DSRBridge(const rclcpp::NodeOptions & options)
: dsr_util::AgentNode("dsr_bridge", options)
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

  // Publisher to the other DSR bridge
  node_to_ros_pub_ = this->create_publisher<dsr_msgs::msg::Node>(node_topic_, 10);
  edge_to_ros_pub_ = this->create_publisher<dsr_msgs::msg::Edge>(edge_topic_, 10);

  // Subscriber to the external DSR graph
  node_from_ros_sub_ = this->create_subscription<dsr_msgs::msg::Node>(
    node_topic_, 10, std::bind(&DSRBridge::node_from_ros_callback, this, std::placeholders::_1));
  edge_from_ros_sub_ = this->create_subscription<dsr_msgs::msg::Edge>(
    edge_topic_, 10, std::bind(&DSRBridge::edge_from_ros_callback, this, std::placeholders::_1));

  return AgentNode::on_configure(state);
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
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "The node [%s] couldn't be updated in the DSR", msg->name.c_str());
      }
      // Create the node
    } else {
      auto new_node = from_msg(*msg);
      if (auto id = G_->insert_node(new_node); id.has_value()) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Inserted [%s] node successfully of type [%s] in the DSR",
          msg->name.c_str(), msg->type.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "The node [%s] couldn't be inserted in the DSR", msg->name.c_str());
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
        "Th edge [%s->%s] of type [%s] has been stored until the nodes are created",
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

void DSRBridge::node_created(std::uint64_t id, const std::string & /*type*/)
{
  // Filter the nodes that comes from the same source
  if (auto dsr_node = G_->get_node(id); dsr_node.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value());
      (source.has_value() && source.value() == source_))
    {
      node_to_ros_pub_->publish(to_msg(dsr_node.value()));
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
  // Filter the nodes that comes from the same source
  if (auto source = G_->get_attrib_by_name<source_att>(node);
    (source.has_value() && source.value() == source_))
  {
    node_to_ros_pub_->publish(to_msg(node, true));
  }
}

void DSRBridge::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string & edge_tag)
{
  // Filter the edges that comes from the same source
  if (auto dsr_edge = G_->get_edge(from, to, edge_tag); dsr_edge.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_edge.value());
      (source.has_value() && source.value() == source_))
    {
      edge_to_ros_pub_->publish(to_msg(dsr_edge.value(), true));
    }
  }
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
  node_msg.updated = false;
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
    edge_msg.updated = false;
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

}  // namespace dsr_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_bridge::DSRBridge)
