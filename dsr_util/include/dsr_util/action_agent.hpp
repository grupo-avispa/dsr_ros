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
 */
template<typename ActionT>
class ActionAgent : public dsr_util::AgentNode
{
public:
  /**
   * @brief Construct a new Action Agent object.
   */
  ActionAgent();

private:
  using GoalHandleActionT = rclcpp_action::ClientGoalHandle<ActionT>;

  /**
   * @brief Get the params from the ROS 2 parameter server.
   */
  void get_params();

  /**
   * @brief Callback that is called when the action server is available.
   *
   * @param goal_handle The goal handle of the action server.
   */
  void goal_response_callback(const GoalHandleActionT::SharedPtr & goal_handle);

  /**
   * @brief Callback that is called when the action server is unavailable.
   *
   * @param feedback The feedback of the action server.
   */
  void feedback_callback(
    GoalHandleActionT::SharedPtr, const std::shared_ptr<const ActionT::Feedback> feedback);

  /**
   * @brief Callback that is called when the action server is unavailable.
   *
   * @param result The result of the action server.
   */
  void result_callback(const GoalHandleActionT::WrappedResult & result);

  // Inherited from AgentNode
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override;

  // DSR node name
  std::string dsr_node_name_;

  // ROS 2 action client
  rclcpp_action::Client<ActionT>::SharedPtr action_client_;

  // ROS 2 goal handle
  std::shared_ptr<GoalHandleActionT> goal_handle_;
};

}  // namespace dsr_util

#endif  // DSR_UTIL__ACTION_AGENT_HPP_
