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

#include "dsr_util/action_agent.hpp"

namespace dsr_util
{

template<class ActionT, typename EdgeT>
ActionAgent<ActionT, EdgeT>::ActionAgent(
  std::string ros_node_name, std::string ros_action_name, std::string dsr_action_name)
: dsr_util::AgentNode(ros_node_name), ros_action_name_(ros_action_name),
  dsr_action_name_(dsr_action_name)
{
  // Get ROS parameters
  get_params();

  // Wait until the DSR graph is ready
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());

  // Add the navigation node to the DSR graph
  add_node_with_edge<navigation_node_type, stopped_edge_type>(dsr_action_name_, source_);
}

template<class ActionT, typename EdgeT>
void ActionAgent<ActionT, EdgeT>::get_params()
{
  // ROS parameters
  // DSR parameters
  declare_parameter_if_not_declared(
    this, "dsr_node_name", rclcpp::ParameterValue(""),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the node in the DSR graph"));
  this->get_parameter("dsr_node_name", dsr_action_name_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter dsr_node is set to: [%s]", dsr_action_name_.c_str());
}

template<class ActionT, typename EdgeT>
void ActionAgent<ActionT, EdgeT>::createActionClient(const std::string & ros_action_name)
{
  // Now that we have the ROS node to use, create the action client for this BT action
  action_client_ = rclcpp_action::create_client<ActionT>(this, ros_action_name, callback_group_);

  // Make sure the server is actually there before continuing
  RCLCPP_DEBUG(this->get_logger(), "Waiting for \"%s\" action server", ros_action_name.c_str());
  if (!action_client_->wait_for_action_server(wait_for_service_timeout_)) {
    RCLCPP_ERROR(
      this->get_logger(), "\"%s\" action server not available after waiting for %.2fs",
      ros_action_name.c_str(),
      wait_for_service_timeout_.count() / 1000.0);
    throw std::runtime_error(
            std::string("Action server ") + ros_action_name + std::string(" not available"));
  }
}

template<class ActionT, typename EdgeT>
void ActionAgent<ActionT, EdgeT>::send_new_goal()
{
  goal_result_available_ = false;
  auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr future) {
      goal_handle_ = future.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
      } else {
        update_dsr_when_response();
      }
    };
  send_goal_options.result_callback =
    [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
      if (future_goal_handle_) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Goal result for %s available, but it hasn't received the goal response yet. "
          "It's probably a goal result for the last goal request", ros_action_name_.c_str());
        return;
      }

      // TODO(#1652): a work around until rcl_action interface is updated
      // if goal ids are not matched, the older goal call this callback so ignore the result
      // if matched, it must be processed (including aborted)
      if (this->goal_handle_->get_goal_id() == result.goal_id) {
        goal_result_available_ = true;
        result_ = result;
        update_dsr_when_result();
      }
    };
  send_goal_options.feedback_callback =
    [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback) {
      feedback_ = feedback;
      update_dsr_when_feedback();
    };

  future_goal_handle_ = std::make_shared<
    std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
    action_client_->async_send_goal(goal_, send_goal_options));
  time_goal_sent_ = this->now();
}

template<class ActionT, typename EdgeT>
void ActionAgent<ActionT, EdgeT>::cancel_action()
{
  if (!goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "Cancel called with no active goal.");
    return;
  }
  callback_group_executor_.spin_some();
  auto status = goal_handle_->get_status();
  if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
    status == rclcpp_action::GoalStatus::STATUS_EXECUTING)
  {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to cancel action server for %s", ros_action_name_.c_str());
    }
  }
}

template<class ActionT, typename EdgeT>
void ActionAgent<ActionT, EdgeT>::update_dsr_when_response()
{
  // Replace the 'wants_to' edge with a 'is_performing' edge between robot and action
  if (replace_edge<is_performing_edge_type>(source_, dsr_action_name_, "wants_to")) {
    RCLCPP_INFO(this->get_logger(), "Goal was accepted by the action server");
  }
}

template<class ActionT, typename EdgeT>
void ActionAgent<ActionT, EdgeT>::update_dsr_when_feedback()
{
  // TODO(ajtudela): Is this necessary?
  // Replace the 'stopped' edge with a 'actioning' edge between robot and navigation
  auto stopped_edge = G_->get_edge(source_, "navigation", "stopped");
  if (stopped_edge.has_value()) {
    replace_edge<EdgeT>(source_, "navigation", "stopped");
  }
}

template<class ActionT, typename EdgeT>
void ActionAgent<ActionT, EdgeT>::update_dsr_when_result()
{
  // TODO(ajtudela): Is this necessary?
  // Replace the 'docking' edge with a 'stopped' edge between robot and navigation
  replace_edge<stopped_edge_type>(source_, "navigation", "navigating");
  switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // Replace the 'is_performing' edge with a 'finished' edge between robot and action
      if (replace_edge<finished_edge_type>(source_, dsr_action_name_, "is_performing")) {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      // Replace the 'is_performing' edge with a 'failed' edge between robot and action
      if (replace_edge<failed_edge_type>(source_, dsr_action_name_, "is_performing")) {
        RCLCPP_ERROR(this->get_logger(), "Goal aborted");
      }
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // Replace the 'is_performing' edge with a 'canceled' edge between robot and action
      if (replace_edge<cancel_edge_type>(source_, dsr_action_name_, "is_performing")) {
        RCLCPP_ERROR(this->get_logger(), "Goal canceled");
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

template<class ActionT, typename EdgeT>
void ActionAgent<ActionT, EdgeT>::edge_updated(
  std::uint64_t from, std::uint64_t to, const std::string & type)
{
  // Check if the robot wants to start the action: robot ---(wants_to)--> action
  if (type == "wants_to") {
    auto robot_node = G_->get_node(source_);
    auto action_node = G_->get_node(dsr_action_name_);
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
}  // namespace dsr_util
