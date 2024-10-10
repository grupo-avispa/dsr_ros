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

// ROS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "dsr_util/agent_node.hpp"

namespace dsr_util
{

AgentNode::AgentNode(std::string ros_node_name, const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(ros_node_name, "", options)
{
  // Register types
  qRegisterMetaType<uint64_t>("uint64_t");
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
  qRegisterMetaType<DSR::Node>("Node");
  qRegisterMetaType<DSR::Edge>("Edge");
  qRegisterMetaType<DSR::SignalInfo>("DSR::SignalInfo");
}

CallbackReturn AgentNode::on_configure(const rclcpp_lifecycle::State & state)
{
  // Get ROS parameters
  get_common_params();

  // Create graph
  try {
    G_ = std::make_shared<dsr_util::DSRGraphExt>(agent_name_, agent_id_, dsr_input_file_);
  } catch (const DSR::DSRException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    on_cleanup(state);
    return CallbackReturn::FAILURE;
  }

  // Get RT API
  rt_ = G_->get_rt_api();

  // Create service
  save_dsr_service_ = this->create_service<dsr_msgs::srv::SaveDSR>(
    "save_dsr",
    std::bind(&AgentNode::save_dsr, this, std::placeholders::_1, std::placeholders::_2));

  // Add connection signals
  QObject::connect(
    G_.get(), &DSR::DSRGraph::update_node_signal, this, &AgentNode::node_updated);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::update_node_attr_signal, this, &AgentNode::node_attr_updated);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::update_edge_signal, this, &AgentNode::edge_updated);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &AgentNode::edge_attr_updated);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::del_edge_signal, this, &AgentNode::edge_deleted);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::del_node_signal, this, &AgentNode::node_deleted);

  RCLCPP_INFO(this->get_logger(), "Configured agent node");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AgentNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  RCLCPP_INFO(this->get_logger(), "Activating the node...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AgentNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(this->get_logger(), "Deactivating the node...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AgentNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning the node...");

  // Release the shared pointers
  G_.reset();
  rt_.reset();
  save_dsr_service_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn AgentNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Shutdown the node from state %s.", state.label().c_str());

  // Release the shared pointers
  G_.reset();
  rt_.reset();
  save_dsr_service_.reset();

  return CallbackReturn::SUCCESS;
}

void AgentNode::get_common_params()
{
  // Agent parameters
  declare_parameter_if_not_declared(
    this, "agent_name", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the agent"));
  this->get_parameter("agent_name", agent_name_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter agent_name is set to: [%s]", agent_name_.c_str());

  declare_parameter_if_not_declared(
    this, "agent_id", rclcpp::ParameterValue(0),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The id of the agent")
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange()
        .set__from_value(0)
        .set__to_value(1000)
        .set__step(1)}
  ));
  this->get_parameter("agent_id", agent_id_);
  RCLCPP_INFO(this->get_logger(), "The parameter agent_id is set to: [%d]", agent_id_);

  // DSR parameters
  declare_parameter_if_not_declared(
    this, "dsr_input_file", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the input file to load the DSR graph from"));
  this->get_parameter("dsr_input_file", dsr_input_file_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter dsr_input_file is set to: [%s]", dsr_input_file_.c_str());

  // Other parameters
  declare_parameter_if_not_declared(
    this, "source", rclcpp::ParameterValue("robot"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Physical source of the agent"));
  this->get_parameter("source", source_);
  RCLCPP_INFO(this->get_logger(), "The parameter source is set to: [%s]", source_.c_str());
}

void AgentNode::update_rt_attributes(
  DSR::Node & from, DSR::Node & to, const geometry_msgs::msg::Transform & msg)
{
  // Get translation and rotation
  std::vector<float> trans = {
    static_cast<float>(msg.translation.x),
    static_cast<float>(msg.translation.y),
    static_cast<float>(msg.translation.z)
  };
  tf2::Quaternion q;
  tf2::fromMsg(msg.rotation, q);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  std::vector<float> rot = {
    static_cast<float>(roll),
    static_cast<float>(pitch),
    static_cast<float>(yaw)
  };

  // Insert or update edge
  rt_->insert_or_assign_edge_RT(from, to.id(), trans, rot);
}

void AgentNode::save_dsr(
  const std::shared_ptr<dsr_msgs::srv::SaveDSR::Request> request,
  std::shared_ptr<dsr_msgs::srv::SaveDSR::Response> response)
{
  G_->write_to_json_file(request->dsr_url);
  response->result = true;
}

}  // namespace dsr_util
