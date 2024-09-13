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

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/action_agent.hpp"

/* Initialize the publishers and subscribers */
ActionAgent::ActionAgent()
: dsr_util::AgentNode("action_agent")
{
  // Get ROS parameters
  get_params();

  // Add connection signals
  QObject::connect(
    G_.get(), &DSR::DSRGraph::update_node_signal, this, &ActionAgent::node_updated);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::update_node_attr_signal, this, &ActionAgent::node_attr_updated);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::update_edge_signal, this, &ActionAgent::edge_updated);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &ActionAgent::edge_attr_updated);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::del_edge_signal, this, &ActionAgent::edge_deleted);
  QObject::connect(
    G_.get(), &DSR::DSRGraph::del_node_signal, this, &ActionAgent::node_deleted);

  // Wait until the DSR graph is ready
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Add the navigation node to the DSR graph
  add_node_with_edge<navigation_node_type, stopped_edge_type>(dsr_node_name_, source_);
}

/* Initialize ROS parameters */
void ActionAgent::get_params()
{
  // ROS parameters
  // DSR parameters
  nav2_util::declare_parameter_if_not_declared(
    this, "dsr_node_name", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the node in the DSR graph"));
  this->get_parameter("dsr_node_name", dsr_node_name_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter dsr_node is set to: [%s]", dsr_node_name_.c_str());
}

void ActionAgent::node_updated(std::uint64_t id, const std::string & type)
{
}

void ActionAgent::node_attr_updated(
  uint64_t id, const std::vector<std::string> & att_names)
{
}

void ActionAgent::edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type)
{
  // Check if the robot wants to start the action: robot ---(wants_to)--> action
  if (type == "wants_to") {
    auto robot_node = G_->get_node(source_);
    auto action_node = G_->get_node(dsr_node_name_);
    if (robot_node.has_value() && from == robot_node.value().id() &&
      action_node.has_value() && to == action_node.value().id())
    {
      // Replace the 'wants_to' edge with a 'is_performing' edge between robot and action
      // if (replace_edge<is_performing_edge_type>(from, to, type)) { }
    }
  }

  // Check if the robot wants to abort the action
  // if (type == "abort") {}
}

void ActionAgent::edge_attr_updated(
  std::uint64_t from, std::uint64_t to,
  const std::string & type, const std::vector<std::string> & att_names)
{
}

void ActionAgent::node_deleted(std::uint64_t id)
{
}

void ActionAgent::edge_deleted(
  std::uint64_t from, std::uint64_t to, const std::string & edge_tag)
{
}

void ActionAgent::goal_response_callback(const GoalHandleActionT::SharedPtr & goal_handle)
{
  goal_handle_ = goal_handle;
  if (!goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "Action goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Action goal accepted by server, waiting for result");
  }
}

void ActionAgent::feedback_callback(
  GoalHandleActionT::SharedPtr, const std::shared_ptr<const ActionT::Feedback> feedback)
{
  // Modify atributes in the DSR node
}

void ActionAgent::result_callback(const GoalHandleActionT::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: {
        // Replace the 'is_performing' edge with a 'finished' edge between robot and action
        if (replace_edge<finished_edge_type>(source_, dsr_node_name_, "is_performing")) {
          RCLCPP_INFO(this->get_logger(), "Goal was reached");
        }
        break;
      }
    case rclcpp_action::ResultCode::ABORTED: {
        // Replace the 'is_performing' edge with a 'failed' edge between robot and action
        if (replace_edge<failed_edge_type>(source_, dsr_node_name_, "is_performing")) {
          RCLCPP_ERROR(this->get_logger(), "Goal was failed");
        }
        break;
      }
    case rclcpp_action::ResultCode::CANCELED: {
        break;
      }
    default: {
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
      }
  }
}

int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ActionAgent>();

  dsr_util::QtExecutor exe;
  exe.add_node(node);
  exe.start();

  auto res = app.exec();
  rclcpp::shutdown();
  return res;
}
