// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#ifndef DSR_AGENTS__DOCKING_AGENT_HPP_
#define DSR_AGENTS__DOCKING_AGENT_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "opennav_docking_msgs/action/undock_robot.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

namespace dsr_agents
{

class DockingAgent : public dsr_util::AgentNode
{
public:
  explicit DockingAgent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Dock = opennav_docking_msgs::action::DockRobot;
  using Undock = opennav_docking_msgs::action::UndockRobot;
  using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;
  using GoalHandleUndock = rclcpp_action::ClientGoalHandle<Undock>;

  rclcpp_action::Client<Dock>::SharedPtr dock_client_;
  rclcpp_action::Client<Undock>::SharedPtr undock_client_;
  std::shared_ptr<GoalHandleDock> goal_handle_;

  void cancel_action();
  void start_docking();
  void start_undocking();

  // Docking action callbacks
  void dock_goal_response_callback(const GoalHandleDock::SharedPtr & goal_handle);
  void dock_feedback_callback(
    GoalHandleDock::SharedPtr, const std::shared_ptr<const Dock::Feedback> feedback);
  void dock_result_callback(const GoalHandleDock::WrappedResult & result);

  // Undocking action callbacks
  void undock_goal_response_callback(const GoalHandleUndock::SharedPtr & goal_handle);
  void undock_feedback_callback(
    GoalHandleUndock::SharedPtr, const std::shared_ptr<const Undock::Feedback> feedback);
  void undock_result_callback(const GoalHandleUndock::WrappedResult & result);

  // DSR callbacks
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override;
};

}  // namespace dsr_agents

#endif  // DSR_AGENTS__DOCKING_AGENT_HPP_
