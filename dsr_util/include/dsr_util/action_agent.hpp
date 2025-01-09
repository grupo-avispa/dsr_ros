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

#ifndef DSR_UTIL__ACTION_AGENT_HPP_
#define DSR_UTIL__ACTION_AGENT_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/node_agent.hpp"

namespace dsr_util
{

/**
 * @class dsr_util::ActionAgent
 * @brief Wrapper around rclcpp_action to call ROS 2 actions from the DSR graph.
 * This class has callbacks that listen a 'wants_to' edge between the robot and the action node.
 * When the robot wants to start the action, the agent calls the ROS 2 action server and waits
 * for the result. The agent also listens to 'abort' and 'cancel' edges to cancel the action.
 *
 * @tparam ActionT The type of the ROS 2 action.
 */
template<class ActionT>
class ActionAgent : public dsr_util::NodeAgent
{
public:
  /**
   * @brief Construct a new Action Agent object.
   *
   * @param ros_node_name The name of the ROS node.
   * @param ros_action_name The name of the ROS action.
   * @param options The options for the ROS node.
   */
  ActionAgent(
    std::string ros_node_name, std::string ros_action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : dsr_util::NodeAgent(ros_node_name, options), ros_action_name_(ros_action_name)
  {
    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();
  }

  /**
   * @brief Destroy the Action Agent object.
   */
  ~ActionAgent() = default;

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    // DSR parameters
    // If the action name is not set, use the ROS node name
    declare_parameter_if_not_declared(
      this, "dsr_action_name", rclcpp::ParameterValue(ros_action_name_),
      rcl_interfaces::msg::ParameterDescriptor()
      .set__description("The name of the action in the DSR graph"));
    this->get_parameter("dsr_action_name", dsr_action_name_);
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter dsr_action_name is set to: [%s]", dsr_action_name_.c_str());

    int wait_for_service_timeout;
    declare_parameter_if_not_declared(
      this, "wait_for_service_timeout", rclcpp::ParameterValue(1000),
      rcl_interfaces::msg::ParameterDescriptor()
      .set__description("The timeout value for waiting for a service to response"));
    this->get_parameter("wait_for_service_timeout", wait_for_service_timeout);
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter wait_for_service_timeout is set to: [%i]", wait_for_service_timeout);
    wait_for_service_timeout_ = std::chrono::milliseconds(wait_for_service_timeout);

    create_action_client(ros_action_name_);

    return NodeAgent::on_configure(state);
  }

protected:
  using GoalHandleActionT = rclcpp_action::ClientGoalHandle<ActionT>;

  /**
   * @brief Function to perform some user-defined operation when the goal is received from the DSR.
   * Usually, this function should fill the goal message with the data from the DSR node.
   *
   * @param action_node The DSR node with the goal information.
   * @return true If the goal is successfully obtained.
   */
  virtual bool get_goal_from_dsr(DSR::Node action_node) = 0;

  /**
   * @brief Create instance of an action client
   *
   * @param ros_action_name Action name to create client for
   */
  void create_action_client(const std::string & ros_action_name)
  {
    // Now that we have the ROS node to use, create the action client for this action
    action_client_ =
      rclcpp_action::create_client<ActionT>(shared_from_this(), ros_action_name);

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

  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool should_cancel_goal()
  {
    // No need to cancel the goal if goal handle is invalid
    if (!goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "Cancel called with no active goal.");
      return false;
    }

    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  /**
   * @brief Function to send new goal to action server
   */
  void send_new_goal()
  {
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
        } else {
          update_dsr_when_goal_accepted();
          goal_handle_ = goal_handle;
        }
      };
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        result_ = result;
        on_result(result);
        update_dsr_when_result_received();
        goal_handle_.reset();
      };
    send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        feedback_ = feedback;
        on_feedback(feedback);
        update_dsr_when_feedback_received();
      };

    auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);
  }

  /**
   * @brief Cancel the action server goal.
   */
  void cancel_action()
  {
    if (should_cancel_goal()) {
      auto future_cancel =
        action_client_->async_cancel_goal(goal_handle_).wait_for(std::chrono::seconds(5));
      if (future_cancel != std::future_status::ready) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to cancel action server for %s", ros_action_name_.c_str());
      }

      auto future_result =
        action_client_->async_get_result(goal_handle_).wait_for(std::chrono::seconds(5));
      if (future_result != std::future_status::ready) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to get result after canceling action server for %s",
          ros_action_name_.c_str());
      }
    }
  }

  /**
   * @brief Function to perform some user-defined operation inside the feedback callback.
   */
  virtual void on_feedback(const std::shared_ptr<const typename ActionT::Feedback>/*feedback*/)
  {
  }

  /**
   * @brief Function to perform some user-defined operation inside the result callback.
   */
  virtual void on_result(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & /*result*/)
  {
  }

  /**
   * @brief Update the DSR graph when the action server sends a response.
   */
  void update_dsr_when_goal_accepted()
  {
    // Replace the 'wants_to' edge with a 'is_performing' edge between robot and action
    if (replace_edge<is_performing_edge_type>(source_, dsr_action_name_, "wants_to")) {
      RCLCPP_INFO(this->get_logger(), "Goal was accepted by the action server");
    }
  }

  /**
   * @brief Update the DSR graph when the action server sends a feedback.
   */
  void update_dsr_when_feedback_received()
  {
  }

  /**
   * @brief Update the DSR graph when the action server sends a result.
   */
  void update_dsr_when_result_received()
  {
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
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }

  /**
   * @brief Callback executed when an edge is updated in the DSR graph.
   *
   * @param from The id of the parent node.
   * @param to The id of the child node.
   * @param type The type of the edge.
   */
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override
  {
    // Check if the robot wants to abort or cancel the action process: robot ---(abort)--> action
    if (type == "abort" || type == "cancel") {
      auto robot_node = G_->get_node(from);
      auto action_node = G_->get_node(to);
      if (robot_node.has_value() && robot_node.value().name() == source_ &&
        action_node.has_value() && action_node.value().type() == dsr_action_name_)
      {
        RCLCPP_INFO(
          this->get_logger(), "Aborting / canceling the action '%s'", dsr_action_name_.c_str());
        // Delete the action node from the DSR graph
        if (G_->delete_node(action_node.value())) {
          cancel_action();
          RCLCPP_INFO(
            this->get_logger(),
            "Action '%s' has been %sed", dsr_action_name_.c_str(), type.c_str());
        }
      }
      // Check if the robot wants to start the action: robot ---(wants_to)--> action
    } else if (type == "wants_to") {
      auto robot_node = G_->get_node(from);
      auto action_node = G_->get_node(to);
      if (robot_node.has_value() && robot_node.value().name() == source_ &&
        action_node.has_value() && action_node.value().type() == dsr_action_name_)
      {
        if (get_goal_from_dsr(action_node.value())) {
          send_new_goal();
          RCLCPP_INFO(this->get_logger(), "Starting the action '%s'", dsr_action_name_.c_str());
        } else {
          // Replace the 'wants_to' edge with a 'failed' edge between robot and action
          if (replace_edge<failed_edge_type>(source_, dsr_action_name_, "wants_to")) {
            RCLCPP_ERROR(
              this->get_logger(), "Missing goal information in the DSR for the action node '%s'",
              action_node.value().name().c_str());
          }
        }
      }
    }
  }

  // DSR action name
  std::string dsr_action_name_;

  // ROS action name
  std::string ros_action_name_;

  // ROS 2 action client
  rclcpp_action::Client<ActionT>::SharedPtr action_client_;

  // All ROS2 actions have a goal and a result
  ActionT::Goal goal_;
  std::shared_ptr<GoalHandleActionT> goal_handle_;
  GoalHandleActionT::WrappedResult result_;

  /// To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;
};

}   // namespace dsr_util

#endif  // DSR_UTIL__ACTION_AGENT_HPP_
