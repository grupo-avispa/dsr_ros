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

// ROS
#include "nav2_util/string_utils.hpp"

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_bridge/dsr_bridge.hpp"

namespace dsr_bridge
{

DSRBridge::DSRBridge(const rclcpp::NodeOptions & options)
: dsr_util::AgentNode("dsr_bridge", options)
{
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
  edge_to_ros_pub_ = this->create_publisher<dsr_msgs::msg::Edge>(edge_topic_, 10);
  node_to_ros_pub_ = this->create_publisher<dsr_msgs::msg::Node>(node_topic_, 10);

  // Subscriber to the external DSR graph
  edge_from_ros_sub_ = this->create_subscription<dsr_msgs::msg::Edge>(
    edge_topic_, 10, std::bind(&DSRBridge::edge_from_ros_callback, this, std::placeholders::_1));
  node_from_ros_sub_ = this->create_subscription<dsr_msgs::msg::Node>(
    node_topic_, 10, std::bind(&DSRBridge::node_from_ros_callback, this, std::placeholders::_1));

  return AgentNode::on_configure(state);
}

// ROS callbacks
void DSRBridge::edge_from_ros_callback(const dsr_msgs::msg::Edge::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(
    this->get_logger(), "Subscribed to edges topic from [%s]", msg->header.frame_id.c_str());
  // The message comes from the same name, ignore it
  if (msg->header.frame_id == source_) {
    return;
  }
  // Create the current edge
  if (!msg->deleted && !msg->updated) {
    auto new_edge = create_dsr_edge(msg->parent, msg->child, msg->type, msg->attributes);
    if (new_edge.has_value()) {
      modify_attributes(new_edge.value(), msg->attributes);
      if (G_->insert_or_assign_edge(new_edge.value())) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Inserted edge [%s->%s] of type [%s] in the DSR",
          msg->parent.c_str(), msg->child.c_str(), msg->type.c_str());
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "The edge [%s->%s] of type [%s] could not be inserted in the DSR",
        msg->parent.c_str(), msg->child.c_str(), msg->type.c_str());
    }
  } else if (!msg->deleted && msg->updated) {
    // Update the current edge
    if (auto edge = G_->get_edge(msg->parent, msg->child, msg->type); edge.has_value()) {
      modify_attributes(edge.value(), msg->attributes);
      if (G_->insert_or_assign_edge(edge.value())) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Updated edge [%s->%s] of type [%s] in the DSR",
          msg->parent.c_str(), msg->child.c_str(), msg->type.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "The edge [%s->%s] of type [%s] could not be updated in the DSR",
          msg->parent.c_str(), msg->child.c_str(), msg->type.c_str());
      }
    }
  } else if (msg->deleted) {
    // Delete the current edge
    if (auto edge = G_->get_edge(msg->parent, msg->child, msg->type); edge.has_value()) {
      if (G_->delete_edge(msg->parent, msg->child, msg->type)) {
        RCLCPP_INFO(
          this->get_logger(),
          "Deleted edge [%s->%s] successfully of type [%s]",
          msg->parent.c_str(), msg->child.c_str(), msg->type.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "The edge [%s->%s] of type [%s] couldn't be deleted",
          msg->parent.c_str(), msg->child.c_str(), msg->type.c_str());
      }
    } else {
      LostEdge lost(msg->parent, msg->child, msg->type, msg->attributes);
      auto it = std::find(lost_edges_.begin(), lost_edges_.end(), lost);
      if (it != lost_edges_.end()) {
        lost_edges_.erase(it);
        RCLCPP_INFO(this->get_logger(), "Deleted edge from lost_edges vector");
      }
    }
  } else {
    // Error
    RCLCPP_ERROR(this->get_logger(), "The edge message is not well defined");
  }
}

void DSRBridge::node_from_ros_callback(const dsr_msgs::msg::Node::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(
    this->get_logger(),
    "Subscribed to nodes topic from [%s]", msg->header.frame_id.c_str());
  RCLCPP_DEBUG(
    this->get_logger(),
    "Node name [%s] and Frame ID [%s]", msg->name.c_str(), msg->header.frame_id.c_str());
  // The message comes from the same name, ignore it
  if (msg->header.frame_id == source_) {
    return;
  }

  // Create or update the current node
  if (!msg->deleted) {
    if (auto node = G_->get_node(msg->name); node.has_value()) {
      modify_attributes(node.value(), msg->attributes);
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
    } else {
      auto new_node = create_dsr_node(msg->name, msg->type);
      modify_attributes(new_node.value(), msg->attributes);
      if (auto id = G_->insert_node(new_node.value()); id.has_value()) {
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
  } else if (msg->deleted) {
    // Delete the node
    if (auto node = G_->get_node(msg->name); node.has_value()) {
      if (G_->delete_node(node.value().name())) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Deleted [%s] node successfully of type [%s] in the DSR",
          msg->name.c_str(), msg->type.c_str());
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "The node [%s] couldn't be deleted in the DSR", msg->name.c_str());
      }

    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "The node [%s] could not be deleted in the DSR", msg->name.c_str());
    }
  }
}

// DSR callbacks
void DSRBridge::node_created(std::uint64_t id, const std::string & /*type*/)
{
  // Filter the edges that comes from the same source
  if (auto dsr_node = G_->get_node(id); dsr_node.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value());
      (source.has_value() && source == source_))
    {
      // Create the message
      auto node_msg = create_msg_node(dsr_node.value().name(), dsr_node.value().type());
      // Get all the attributes
      node_msg.attributes = attributes_to_string(dsr_node.value().attrs());
      // Publish the message
      node_to_ros_pub_->publish(node_msg);
      RCLCPP_DEBUG(
        this->get_logger(),
        "The new node [%s] of type [%s] has been published through ROS",
        node_msg.name.c_str(), node_msg.type.c_str());
    }
  }
  // Retry to insert lost edges
  std::vector<std::vector<LostEdge>::iterator> delete_lost_edges;
  unsigned int i = 0;
  for (auto & edge : lost_edges_) {
    auto parent_node = G_->get_node(edge.from);
    auto child_node = G_->get_node(edge.to);
    if (parent_node.has_value() && child_node.has_value()) {
      DSR::Edge new_edge;
      new_edge.from(parent_node.value().id());
      new_edge.to(child_node.value().id());
      new_edge.type(edge.type);
      modify_attributes(new_edge, edge.attrs);
      if (G_->insert_or_assign_edge(new_edge)) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Inserted edge losted [%s->%s] of type [%s] in the DSR",
          edge.from.c_str(), edge.to.c_str(), edge.type.c_str());
        auto it = (lost_edges_.begin() + i);
        delete_lost_edges.push_back(it);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "The edge losted [%s->%s] of type [%s] could not be inserted in the DSR",
          edge.from.c_str(), edge.to.c_str(), edge.type.c_str());
      }
    }
    i++;
  }
  for (auto it : delete_lost_edges) {
    lost_edges_.erase(it);
  }
}

void DSRBridge::node_attr_updated(uint64_t id, const std::vector<std::string> & att_names)
{
  // Filter the edges that comes from the same source
  if (auto dsr_node = G_->get_node(id); dsr_node.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value());
      (source.has_value() && source.value() == source_))
    {
      // Create the message
      auto node_msg = create_msg_node(dsr_node.value().name(), dsr_node.value().type());
      // Mark the node as updated
      node_msg.updated = true;
      // Get all the updated attributes
      node_msg.attributes = attributes_updated_to_string(dsr_node.value(), att_names);
      // Publish the message
      node_to_ros_pub_->publish(node_msg);
      RCLCPP_DEBUG(
        this->get_logger(),
        "The updated node [%s] of type [%s] has been published through ROS",
        node_msg.name.c_str(), node_msg.type.c_str());
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
      // Create the message
      auto edge_msg = create_msg_edge(from, to, type);
      // Get all the attributes
      edge_msg.attributes = attributes_to_string(dsr_edge.value().attrs());
      // Publish the message
      edge_to_ros_pub_->publish(edge_msg);
      RCLCPP_DEBUG(
        this->get_logger(),
        "The new edge [%s->%s] of type [%s] has been published through ROS",
        edge_msg.parent.c_str(), edge_msg.child.c_str(), edge_msg.type.c_str());
    }
  }
}

void DSRBridge::edge_attr_updated(
  std::uint64_t from, std::uint64_t to,
  const std::string & type, const std::vector<std::string> & att_names)
{
  // Filter the edges that comes from the same source
  if (auto dsr_edge = G_->get_edge(from, to, type); dsr_edge.has_value()) {
    if (auto source = G_->get_attrib_by_name<source_att>(dsr_edge.value());
      (source.has_value() && source.value() == source_))
    {
      // Create the message
      auto edge_msg = create_msg_edge(from, to, type);
      // Mark the node as updated
      edge_msg.updated = true;
      // Get all the updated attributes
      edge_msg.attributes = attributes_updated_to_string(dsr_edge.value(), att_names);
      // Publish the message
      edge_to_ros_pub_->publish(edge_msg);
      RCLCPP_DEBUG(
        this->get_logger(),
        "The updated edge [%s->%s] of type [%s] has been published through ROS",
        edge_msg.parent.c_str(), edge_msg.child.c_str(), edge_msg.type.c_str());
    }
  }
}

void DSRBridge::node_deleted_by_node(const DSR::Node & node)
{
  // Create the message
  auto node_msg = create_msg_node(node.name(), node.type());
  // Mark the node as deleted
  node_msg.deleted = true;
  // Publish the message
  node_to_ros_pub_->publish(node_msg);
  RCLCPP_DEBUG(
    this->get_logger(),
    "The deleted node [%s] of type [%s] has been published through ROS",
    node_msg.name.c_str(), node_msg.type.c_str());
}

void DSRBridge::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string & edge_tag)
{
  // Create the message
  auto edge_msg = create_msg_edge(from, to, edge_tag);
  // Mark the edge as deleted
  edge_msg.deleted = true;
  // Publish the message
  edge_to_ros_pub_->publish(edge_msg);
  RCLCPP_DEBUG(
    this->get_logger(),
    "The deleted edge [%s->%s] of type [%s] has been published through ROS",
    edge_msg.parent.c_str(), edge_msg.child.c_str(), edge_msg.type.c_str());
}

// Converter functions
std::optional<DSR::Node> DSRBridge::create_dsr_node(std::string name, std::string type)
{
  if (!node_types::check_type(type)) {
    throw std::runtime_error("Error, [" + type + "] is not a valid node type");
  }
  DSR::Node new_node;
  new_node.name(name);
  new_node.type(type);
  return new_node;
}

std::optional<DSR::Edge> DSRBridge::create_dsr_edge(
  std::string from, std::string to, const std::string & type, std::vector<std::string> & atts)
{
  if (!edge_types::check_type(type)) {
    throw std::runtime_error("Error, [" + type + "] is not a valid edge type");
  }
  DSR::Edge new_edge;
  auto parent_node = G_->get_node(from);
  auto child_node = G_->get_node(to);
  if (parent_node.has_value() && child_node.has_value()) {
    new_edge.from(parent_node.value().id());
    new_edge.to(child_node.value().id());
    new_edge.type(type);
    return new_edge;
  }

  lost_edges_.push_back(LostEdge(from, to, type, atts));
  return {};
}

dsr_msgs::msg::Node DSRBridge::create_msg_node(std::string name, std::string type)
{
  dsr_msgs::msg::Node node_msg;
  node_msg.header.stamp = this->now();
  node_msg.header.frame_id = source_;
  node_msg.name = name;
  node_msg.type = type;
  node_msg.updated = false;
  node_msg.deleted = false;
  return node_msg;
}

dsr_msgs::msg::Edge DSRBridge::create_msg_edge(
  std::uint64_t from, std::uint64_t to, const std::string & type)
{
  dsr_msgs::msg::Edge edge_msg;
  auto parent_node = G_->get_node(from);
  auto child_node = G_->get_node(to);
  if (parent_node.has_value() && child_node.has_value()) {
    edge_msg.header.stamp = this->now();
    edge_msg.header.frame_id = source_;
    edge_msg.parent = parent_node.value().name();
    edge_msg.child = child_node.value().name();
    edge_msg.type = type;
    edge_msg.updated = false;
    edge_msg.deleted = false;
  }
  return edge_msg;
}

// Helper functions
template<typename TYPE>
void DSRBridge::modify_attributes(TYPE & elem, std::vector<std::string> & att_str)
{
  for (unsigned int i = 0; i < att_str.size(); i += 3) {
    std::string att_name = att_str[i];
    std::string att_value = att_str[i + 1];
    std::string att_type = att_str[i + 2];

    // Add the attribute to the element
    DSR::Attribute new_att = string_to_attribute(att_value, std::stoi(att_type));
    elem.attrs().insert_or_assign(att_name, new_att);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Updating attribute [%s] = [%s] with type [%ld]",
      att_name.c_str(), att_value.c_str(), new_att.value().index());
  }
}

std::string DSRBridge::attribute_to_string(const DSR::Attribute & att)
{
  std::locale::global(std::locale("C"));
  switch (att.value().index()) {
    case 0: {
        return std::get<std::string>(att.value());
      }
    case 1: {
        return std::to_string(std::get<int32_t>(att.value()));
      }
    case 2: {
        return std::to_string(std::get<float>(att.value()));
      }
    case 3: {
        std::string att_str;
        for (const auto & value : std::get<std::vector<float>>(att.value())) {
          att_str += std::to_string(value) + std::string(",");
        }
        att_str.pop_back();
        return att_str;
      }
    case 4: {
        return std::get<bool>(att.value()) ? "true" : "false";
      }
    case 5: {
        std::string att_str;
        for (const auto & value : std::get<std::vector<uint8_t>>(att.value())) {
          att_str += std::to_string(value) + std::string(",");
        }
        att_str.pop_back();
        return att_str;
      }
    case 6: {
        return std::to_string(std::get<uint32_t>(att.value()));
      }
    case 7: {
        return std::to_string(std::get<uint64_t>(att.value()));
      }
    case 8: {
        return std::to_string(std::get<double>(att.value()));
      }
    default:
      return "";
  }
}

std::vector<std::string> DSRBridge::attributes_to_string(
  const std::map<std::string, DSR::Attribute> & atts)
{
  std::vector<std::string> att_vector_str;
  for (const auto & [att_name, att_value] : atts) {
    std::string att_str = attribute_to_string(att_value);
    att_vector_str.push_back(att_name);
    att_vector_str.push_back(att_str);
    att_vector_str.push_back(get_type_from_attribute(att_value));
    RCLCPP_DEBUG(
      this->get_logger(),
      "Attribute [%s] = [%s]", att_name.c_str(), att_str.c_str());
  }
  return att_vector_str;
}

template<typename TYPE>
std::vector<std::string> DSRBridge::attributes_updated_to_string(
  TYPE & elem,
  const std::vector<std::string> & atts)
{
  std::vector<std::string> att_vector_str;
  for (const auto & att_name : atts) {
    auto search = elem.attrs().find(att_name);
    if (search != elem.attrs().end()) {
      std::string att_value = attribute_to_string(search->second);
      att_vector_str.push_back(att_name);
      att_vector_str.push_back(att_value);
      att_vector_str.push_back(get_type_from_attribute(search->second));
      RCLCPP_DEBUG(
        this->get_logger(),
        "Attribute updated [%s] = [%s]", att_name.c_str(), att_value.c_str());
    }
  }
  return att_vector_str;
}

DSR::Attribute DSRBridge::string_to_attribute(const std::string & att_value, int att_type)
{
  DSR::Attribute new_att;
  switch (att_type) {
    case 0: {
        new_att.value(std::string(att_value));
        break;
      }
    case 1: {
        new_att.value(std::stoi(att_value));
        break;
      }
    case 2: {
        if (att_value == "nan") {
          new_att.value(std::numeric_limits<float>::quiet_NaN());
        } else {
          new_att.value(std::stof(att_value));
        }
        break;
      }
    case 3: {
        std::vector<std::string> values = nav2_util::split(att_value, ',');
        std::vector<float> float_values;
        for (const auto & value : values) {
          if (value == "nan") {
            float_values.push_back(std::numeric_limits<float>::quiet_NaN());
          } else {
            float_values.push_back(std::stof(value));
          }
        }
        new_att.value(float_values);
        break;
      }
    case 4: {
        new_att.value(att_value == "true");
        break;
      }
    case 6: {
        new_att.value(std::stoull(att_value));
        break;
      }
    case 7: {
        new_att.value(std::stoull(att_value));
        break;
      }
    default:
      break;
  }
  return new_att;
}

std::string DSRBridge::get_type_from_attribute(const DSR::Attribute & att)
{
  return std::to_string(att.value().index());
}

}  // namespace dsr_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_bridge::DSRBridge)
