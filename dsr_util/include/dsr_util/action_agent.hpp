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
#include "dsr_util/agent_node.hpp"

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
 * @tparam EdgeT The type of the DSR edge.
 */
template<class ActionT, typename EdgeT>
class ActionAgent : public dsr_util::AgentNode
{
public:
  /**
   * @brief Construct a new Action Agent object.
   *
   * @param ros_node_name The name of the ROS 2 node.
   * @param ros_action_name The name of the ROS 2 action.
   * @param dsr_action_name The name of the DSR action.
   */
  ActionAgent(std::string ros_node_name, std::string ros_action_name, std::string dsr_action_name);

private:
  using GoalHandleActionT = rclcpp_action::ClientGoalHandle<ActionT>;

  /**
   * @brief Get the params from the ROS 2 parameter server.
   */
  void get_params();

  /**
   * @brief Create instance of an action client
   * @param ros_action_name Action name to create client for
   */
  void createActionClient(const std::string & ros_action_name);

  /**
 * @brief Function to send new goal to action server
 */
  void send_new_goal();

  /**
   * @brief Cancel the action server goal.
   */
  void cancel_action();

  /**
   * @brief Update the DSR graph when the action server sends a response.
   */
  void update_dsr_when_response();

  /**
   * @brief Update the DSR graph when the action server sends a feedback.
   */
  void update_dsr_when_feedback();

  /**
   * @brief Update the DSR graph when the action server sends a result.
   */
  void update_dsr_when_result();

  // Inherited from AgentNode
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override;

  // DSR action name
  std::string dsr_action_name_;

  // ROS action name
  std::string ros_action_name_;

  // ROS 2 action client
  rclcpp_action::Client<ActionT>::SharedPtr action_client_;

  // All ROS2 actions have a goal and a result
  ActionT::Goal goal_;
  bool goal_result_available_{false};
  std::shared_ptr<GoalHandleActionT> goal_handle_;
  GoalHandleActionT::WrappedResult result_;

  /// To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // The node that will be used for any ROS operations
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // To track the action server acknowledgement when a new goal is sent
  std::shared_ptr<std::shared_future<std::shared_ptr<GoalHandleActionT>>> future_goal_handle_;
  rclcpp::Time time_goal_sent_;
};

}  // namespace dsr_util

#endif  // DSR_UTIL__ACTION_AGENT_HPP_
