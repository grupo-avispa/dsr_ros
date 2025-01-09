// Copyright (c) 2023 Alberto J. Tudela Roldán
// Copyright (c) 2023 Grupo Avispa, DTE, Universidad de Málaga
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
#include <chrono>
#include <thread>

// ROS
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/topic_agent.hpp"

namespace dsr_agents
{

TopicAgent::TopicAgent(const rclcpp::NodeOptions & options)
: dsr_util::NodeAgent("generic_agent", options)
{
}

dsr_util::CallbackReturn TopicAgent::on_configure(const rclcpp_lifecycle::State & state)
{
  // ROS parameters
  declare_parameter_if_not_declared(
    this, "ros_topic", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The ROS topic to subscribe to"));
  this->get_parameter("ros_topic", ros_topic_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter ros_topic is set to: [%s]", ros_topic_.c_str());

  // DSR parameters
  declare_parameter_if_not_declared(
    this, "dsr_node_name", rclcpp::ParameterValue(ros_topic_),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the node in the DSR graph"));
  this->get_parameter("dsr_node_name", dsr_node_name_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter dsr_node_name is set to: [%s]", dsr_node_name_.c_str());

  declare_parameter_if_not_declared(
    this, "dsr_parent_node_name", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the parent node in the DSR graph"));
  this->get_parameter("dsr_parent_node_name", dsr_parent_node_name_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter dsr_parent_node_name is set to: [%s]", dsr_parent_node_name_.c_str());

  if (ros_topic_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "The parameter 'ros_topic' is empty, cannot configure");
    return dsr_util::CallbackReturn::FAILURE;
  }

  // Subscriber to the topic with a generic subscription
  auto data = rclcpp_lifecycle::LifecycleNode::get_topic_names_and_types();
  for (auto type : data[ros_topic_]) {
    generic_sub_ = create_generic_subscription(
      ros_topic_, type, rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
      std::bind(&TopicAgent::serial_callback, this, std::placeholders::_1));
  }

  return NodeAgent::on_configure(state);
}

dsr_util::CallbackReturn TopicAgent::on_cleanup(const rclcpp_lifecycle::State & state)
{
  // Cleaning the generic subscription
  generic_sub_.reset();

  // Delete the node from the DSR graph
  if (auto dsr_node = G_->get_node(dsr_node_name_); dsr_node.has_value()) {
    delete_node(dsr_node_name_);
  }

  return NodeAgent::on_cleanup(state);
}

void TopicAgent::serial_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  // In order to deserialize the message we have to manually create a ROS2
  // message in which we want to convert the serialized data.
  auto data = rclcpp_lifecycle::LifecycleNode::get_topic_names_and_types();
  const std::string topic_type = data[ros_topic_][0];
  RCLCPP_INFO_ONCE(
    this->get_logger(), "Subscribed to topic [%s] of type [%s]",
    ros_topic_.c_str(), topic_type.c_str());

  handle_topic_type(msg, topic_type);
}

void TopicAgent::handle_topic_type(
  const std::shared_ptr<rclcpp::SerializedMessage> & msg, const std::string & topic_type)
{
  // TODO(ajtudela): Replace 'has_edge_type' to a type according to (sensor, actuator, etc)
  if (topic_type == "sensor_msgs/msg/BatteryState") {
    deserialize_and_update_attributes
    <sensor_msgs::msg::BatteryState, battery_node_type, has_edge_type>(
      msg, dsr_node_name_, dsr_parent_node_name_);
  } else if (topic_type == "sensor_msgs/msg/Image") {
    deserialize_and_update_attributes
    <sensor_msgs::msg::Image, rgbd_node_type, has_edge_type>(
      msg, dsr_node_name_, dsr_parent_node_name_);
  } else if (topic_type == "sensor_msgs/msg/LaserScan") {
    deserialize_and_update_attributes
    <sensor_msgs::msg::LaserScan, laser_node_type, has_edge_type>(
      msg, dsr_node_name_, dsr_parent_node_name_);
  } else {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "Received message of type [%s]. Unknown for the DSR.", topic_type.c_str());
  }
}

template<typename ROS_TYPE, typename NODE_TYPE, typename EDGE_TYPE>
void TopicAgent::deserialize_and_update_attributes(
  const std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & node_name, const std::string & parent_name)
{
  // Deserialize a message to ROS_TYPE
  ROS_TYPE ros_msg;
  auto serializer = rclcpp::Serialization<ROS_TYPE>();
  serializer.deserialize_message(msg.get(), &ros_msg);

  // If the parent name is empty, the parent is the frame_id of the ROS message
  auto new_parent_name = parent_name.empty() ? ros_msg.header.frame_id : parent_name;

  // Add the node with the edge if the node does not exist
  if (auto dsr_node = G_->get_node(node_name); !dsr_node.has_value()) {
    add_node_with_edge<NODE_TYPE, EDGE_TYPE>(node_name, new_parent_name);
  }

  // Update the attributes of the node only if its priority is 0
  if (auto dsr_node = G_->get_node(node_name); dsr_node.has_value()) {
    if (G_->get_priority(dsr_node.value()) == 0) {
      modify_attributes<ROS_TYPE>(dsr_node, ros_msg);
      G_->update_node(dsr_node.value());
    }
  }
}

template<>
void TopicAgent::modify_attributes<sensor_msgs::msg::BatteryState>(
  std::optional<DSR::Node> & node, const sensor_msgs::msg::BatteryState & msg)
{
  // Modify the attributes of the node
  G_->add_or_modify_attrib_local<battery_voltage_att>(node.value(), msg.voltage);
  G_->add_or_modify_attrib_local<battery_temperature_att>(node.value(), msg.temperature);
  G_->add_or_modify_attrib_local<battery_current_att>(node.value(), msg.current);
  G_->add_or_modify_attrib_local<battery_charge_att>(node.value(), msg.charge);
  G_->add_or_modify_attrib_local<battery_capacity_att>(node.value(), msg.capacity);
  G_->add_or_modify_attrib_local<battery_design_capacity_att>(node.value(), msg.design_capacity);
  G_->add_or_modify_attrib_local<battery_percentage_att>(
    node.value(), static_cast<float>(msg.percentage * 100.0));
  // Convert the power supply status from enum to string
  std::string status;
  switch (msg.power_supply_status) {
    case 0:
      status = "unknown";
      break;
    case 1:
      status = "charging";
      break;
    case 2:
      status = "discharging";
      break;
    case 3:
      status = "not_charging";
      break;
    case 4:
      status = "full";
      break;
    default:
      status = "unknown";
      break;
  }
  G_->add_or_modify_attrib_local<battery_power_supply_status_att>(node.value(), status);
  G_->add_or_modify_attrib_local<battery_power_supply_health_att>(
    node.value(), static_cast<int>(msg.power_supply_health));
  G_->add_or_modify_attrib_local<battery_power_supply_technology_att>(
    node.value(), static_cast<int>(msg.power_supply_technology));
  G_->add_or_modify_attrib_local<battery_present_att>(node.value(), msg.present);
  G_->add_or_modify_attrib_local<battery_cell_voltage_att>(node.value(), msg.cell_voltage);
  G_->add_or_modify_attrib_local<battery_cell_temperature_att>(node.value(), msg.cell_temperature);
  G_->add_or_modify_attrib_local<battery_location_att>(node.value(), msg.location);
  G_->add_or_modify_attrib_local<battery_serial_number_att>(node.value(), msg.serial_number);
  // Print the attributes of the node
  RCLCPP_DEBUG(this->get_logger(), "%s node updated with attributes:", node.value().name().c_str());
}

template<>
void TopicAgent::modify_attributes<sensor_msgs::msg::Image>(
  std::optional<DSR::Node> & node, const sensor_msgs::msg::Image & msg)
{
  // Modify the attributes of the node depending the type of the image
  if (msg.encoding == sensor_msgs::image_encodings::RGB8) {
    G_->add_or_modify_attrib_local<cam_rgb_att>(node.value(), msg.data);
    G_->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), static_cast<int>(msg.height));
    G_->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), static_cast<int>(msg.width));
  } else if (msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    G_->add_or_modify_attrib_local<cam_depth_att>(node.value(), msg.data);
    G_->add_or_modify_attrib_local<cam_depth_height_att>(
      node.value(), static_cast<int>(msg.height));
    G_->add_or_modify_attrib_local<cam_depth_width_att>(node.value(), static_cast<int>(msg.width));
  }
  // Print the attributes of the node
  RCLCPP_DEBUG(
    this->get_logger(),
    "Update [%s] node with attributes: ", node.value().name().c_str());
}

template<>
void TopicAgent::modify_attributes<sensor_msgs::msg::LaserScan>(
  std::optional<DSR::Node> & node, const sensor_msgs::msg::LaserScan & msg)
{
  // Convert from ROS to DSR
  std::vector<float> angles(msg.ranges.size());
  if (!angles.empty()) {
    angles[0] = msg.angle_min;
    for (uint i = 1; i < msg.ranges.size() - 1; i++) {
      angles[i] = angles[i - 1] + msg.angle_increment;
    }
    angles[msg.ranges.size() - 1] = msg.angle_max;

    // Modify the attributes of the node
    G_->add_or_modify_attrib_local<laser_angles_att>(node.value(), angles);
    G_->add_or_modify_attrib_local<laser_dists_att>(node.value(), msg.ranges);
    // Print the attributes of the node
    RCLCPP_DEBUG(
      this->get_logger(), "Update [%s] node with attributes: ", node.value().name().c_str());
  }
}

template<>
void TopicAgent::modify_attributes<std_msgs::msg::String>(
  std::optional<DSR::Node> & node, const std_msgs::msg::String & msg)
{
  // Modify the attributes of the node
  G_->add_or_modify_attrib_local<text_att>(node.value(), msg.data);
  // Print the attributes of the node
  RCLCPP_DEBUG(
    this->get_logger(), "Update [%s] node with attributes: ", node.value().name().c_str());
}

}  // namespace dsr_agents

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_agents::TopicAgent)
