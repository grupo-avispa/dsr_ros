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

#ifndef DSR_AGENTS__NAV_AGENT_HPP_
#define DSR_AGENTS__NAV_AGENT_HPP_

#include <memory>

// ROS
#include "nav2_msgs/action/navigate_to_pose.hpp"

// DSR
#include "dsr_util/action_agent.hpp"

namespace dsr_agents
{

/**
 * @class dsr_agents::NavigationAgent
 * @brief Agent to navigate the robot to a specific pose.
 */
class NavigationAgent : public dsr_util::ActionAgent<nav2_msgs::action::NavigateToPose>
{
public:
  /**
   * @brief Construct a new Navigation Agent object.
   *
   * @param options The options for the ROS node.
   */
  explicit NavigationAgent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /**
   * @brief Get the goal from the DSR node.
   *
   * @param action_node The DSR node with the goal information.
   */
  void get_goal_from_dsr(DSR::Node action_node) override;

  /**
   * @brief Callback to update the robot pose in the DSR graph while navigating.
   */
  void on_feedback(
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) override;
};

}  // namespace dsr_agents

#endif  // DSR_AGENTS__NAV_AGENT_HPP_
