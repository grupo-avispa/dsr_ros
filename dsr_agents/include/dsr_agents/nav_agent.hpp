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

#ifndef DSR_AGENTS__NAV_AGENT_HPP_
#define DSR_AGENTS__NAV_AGENT_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

namespace dsr_agents
{

class NavigationAgent : public dsr_util::AgentNode
{
public:
  NavigationAgent();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  std::shared_ptr<GoalHandleNavigateToPose> goal_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void send_to_goal(geometry_msgs::msg::Pose goal_pose);
  void update_robot_pose_in_dsr(geometry_msgs::msg::Pose pose);
  void cancel_action();

  // Navigation action callbacks
  void nav_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void nav_feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void nav_result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

  // DSR callbacks
  void node_updated(std::uint64_t id, const std::string & type) override;
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override;
};

}  // namespace dsr_agents

#endif  // DSR_AGENTS__NAV_AGENT_HPP_
