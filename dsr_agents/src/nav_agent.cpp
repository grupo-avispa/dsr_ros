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


// DSR
#include "dsr_agents/nav_agent.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dsr_agents
{

NavigationAgent::NavigationAgent(const rclcpp::NodeOptions & options)
: dsr_util::ActionAgent<nav2_msgs::action::NavigateToPose>(
    "navigation_agent", "navigate_to_pose", options)
{
}

void NavigationAgent::get_goal_from_dsr(DSR::Node action_node)
{
  // Get the attributes from the move node
  auto goal_x = G_->get_attrib_by_name<goal_x_att>(action_node.value());
  auto goal_y = G_->get_attrib_by_name<goal_y_att>(action_node.value());
  auto goal_angle = G_->get_attrib_by_name<goal_angle_att>(action_node.value());

  // Check if the goal is a point or a room and send the robot
  if (goal_x.has_value() && goal_y.has_value() && goal_angle.has_value()) {
    goal_.pose.header.frame_id = "map";
    goal_.pose.header.stamp = this->now();
    goal_.pose.pose.position.x = goal_x.value();
    goal_.pose.pose.position.y = goal_y.value();
    goal_.pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, goal_angle.value()));
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, goal_angle.value())));
  } else {
    RCLCPP_ERROR(this->get_logger(), "Goal not found in the move node");
  }
}

void NavigationAgent::on_feedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  // Update the robot pose into the DSR graph
  auto robot_pose = feedback->current_pose.pose;
  if (auto robot_node = G_->get_node(source_); robot_node.has_value()) {
    if (G_->get_priority(robot_node.value()) == 0) {
      G_->add_or_modify_attrib_local<pose_x_att>(
        robot_node.value(), static_cast<float>(robot_pose.position.x));
      G_->add_or_modify_attrib_local<pose_y_att>(
        robot_node.value(), static_cast<float>(robot_pose.position.y));
      G_->add_or_modify_attrib_local<pose_angle_att>(
        robot_node.value(), static_cast<float>(tf2::getYaw(robot_pose.orientation)));
      G_->update_node(robot_node.value());
    }
  }
}

}  // namespace dsr_agents

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_agents::NavigationAgent)
