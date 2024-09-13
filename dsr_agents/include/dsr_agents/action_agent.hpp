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

#ifndef DSR_AGENTS__ACTION_AGENT_HPP_
#define DSR_AGENTS__ACTION_AGENT_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_agents/agent_node.hpp"


template<typename ActionT>
class ActionAgent : public dsr_util::AgentNode
{
public:
  ActionAgent();

private:
  using GoalHandleActionT = rclcpp_action::ClientGoalHandle<ActionT>;

  std::string dsr_node_name_;
  rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::shared_ptr<GoalHandleActionT> goal_handle_;

  void get_params();

  // Action callbacks
  void goal_response_callback(const GoalHandleActionT::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleActionT::SharedPtr, const std::shared_ptr<const ActionT::Feedback> feedback);
  void result_callback(const GoalHandleActionT::WrappedResult & result);

  // DSR callbacks
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override;
};

#endif  // DSR_AGENTS__ACTION_AGENT_HPP_
