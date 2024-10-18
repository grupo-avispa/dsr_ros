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

// C++
#include <chrono>
#include <thread>

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/nav_agent.hpp"

namespace dsr_agents
{

NavigationAgent::NavigationAgent(const rclcpp::NodeOptions & options)
: dsr_util::AgentNode("navigation_agent", options)
{
  // Initialize transform buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Add the 'navigation' node with and edge 'stopped'
  // hanging from the 'robot' node into the DSR graph
  if (auto nav_node = G_->get_node("navigation"); !nav_node.has_value()) {
    add_node_with_edge<navigation_node_type, stopped_edge_type>("navigation", source_);
  }
}

void NavigationAgent::node_updated(std::uint64_t id, const std::string & /*type*/)
{
  // Update the current robot pose into the DSR graph when the navigation node is updated
  if (auto node = G_->get_node(id); node.has_value() && node.value().type() == "navigation") {
    // Get the current pose of the robot
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!nav2_util::getCurrentPose(robot_pose, *tf_buffer_, "map", "base_link", 2.0)) {
      return;
    }
    // Update the pose of the robot into the DSR graph
    update_robot_pose_in_dsr(robot_pose.pose);
  }
}

void NavigationAgent::edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type)
{
  // Check if the robot wants to abort or cancel the navigation: robot ---(abort)--> move
  if (type == "abort" || type == "cancel") {
    auto robot_node = G_->get_node(from);
    auto move_node = G_->get_node(to);
    if (robot_node.has_value() && robot_node.value().name() == source_ &&
      move_node.has_value() && move_node.value().type() == "move")
    {
      RCLCPP_INFO(this->get_logger(), "Starting to %s the navigation", type.c_str());
      // Remove the move node from the DSR graph
      if (G_->delete_node(move_node.value())) {
        cancel_action();
        RCLCPP_INFO(this->get_logger(), "Navigation %sed", type.c_str());
      }
    }
  } else if (type == "wants_to") {
    // Check if the robot wants to start the navigation: robot ---(wants_to)--> move
    auto robot_node = G_->get_node(from);
    auto move_node = G_->get_node(to);
    if (robot_node.has_value() && robot_node.value().name() == source_ &&
      move_node.has_value() && move_node.value().type() == "move")
    {
      RCLCPP_INFO(this->get_logger(), "Starting the navigation");
      // Get the attributes from the move node
      auto goal_x = G_->get_attrib_by_name<goal_x_att>(move_node.value());
      auto goal_y = G_->get_attrib_by_name<goal_y_att>(move_node.value());
      auto goal_angle = G_->get_attrib_by_name<goal_angle_att>(move_node.value());
      // Check if the goal is a point or a room and send the robot
      if (goal_x.has_value() && goal_y.has_value() && goal_angle.has_value()) {
        send_to_goal(
          geometry_msgs::build<geometry_msgs::msg::Pose>()
          .position(
            geometry_msgs::build<geometry_msgs::msg::Point>()
            .x(goal_x.value()).y(goal_y.value()).z(0))
          .orientation(
            tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), goal_angle.value()))));
        RCLCPP_INFO(
          this->get_logger(), "Navigation started to goal [%f, %f]",
          goal_x.value(), goal_y.value());
      } else {
        RCLCPP_WARN(this->get_logger(), "Goal or zone not found in the move node");
      }
    }
  }
}

void NavigationAgent::nav_goal_response_callback(
  const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  goal_handle_ = goal_handle;
  if (!goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "Navigation goal was rejected by server");
  } else {
    // Replace the 'wants_to' edge with a 'is_performing' edge between robot and move
    if (replace_edge<is_performing_edge_type>(source_, "move", "wants_to")) {
      RCLCPP_INFO(
        this->get_logger(), "Navigation goal accepted by server, waiting for result");
    }
  }
}

void NavigationAgent::nav_feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  // Replace the 'stopped' edge with a 'navigating' edge between robot and navigation
  auto stopped_edge = G_->get_edge(source_, "navigation", "stopped");
  if (stopped_edge.has_value()) {
    replace_edge<navigating_edge_type>(source_, "navigation", "stopped");
  }

  // Set the current pose of the robot into the DSR graph
  update_robot_pose_in_dsr(feedback->current_pose.pose);
}

void NavigationAgent::nav_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  // Replace the 'navigating' edge with a 'stopped' edge between robot and navigation
  if (replace_edge<stopped_edge_type>(source_, "navigation", "navigating")) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        // Replace the 'is_performing' edge with a 'finished' edge between robot and move
        if (replace_edge<finished_edge_type>(source_, "move", "is_performing")) {
          RCLCPP_INFO(this->get_logger(), "Goal reached");
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        // Replace the 'is_performing' edge with a 'failed' edge between robot and move
        if (replace_edge<failed_edge_type>(source_, "move", "is_performing")) {
          RCLCPP_ERROR(this->get_logger(), "Goal aborted");
        }
        break;
      case rclcpp_action::ResultCode::CANCELED:
        // Replace the 'is_performing' edge with a 'canceled' edge between robot and move
        if (replace_edge<cancel_edge_type>(source_, "move", "is_performing")) {
          RCLCPP_ERROR(this->get_logger(), "Goal canceled");
        }
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }
}

void NavigationAgent::send_to_goal(geometry_msgs::msg::Pose goal_pose)
{
  using namespace std::placeholders;
  navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
    shared_from_this(), "navigate_to_pose");

  if (!this->navigation_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Navigation server not available after waiting");
    return;
  }

  // Check if the navigator is already working towards a goal
  if (goal_handle_ &&
    (goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
    goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Navigator already working towards a goal. Cancelling the previous goal.");
    cancel_action();
  }

  // Populate a goal message
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose = goal_pose;

  auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_options.goal_response_callback =
    std::bind(&NavigationAgent::nav_goal_response_callback, this, _1);
  send_options.feedback_callback =
    std::bind(&NavigationAgent::nav_feedback_callback, this, _1, _2);
  send_options.result_callback =
    std::bind(&NavigationAgent::nav_result_callback, this, _1);

  auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_options);
  RCLCPP_INFO(this->get_logger(), "Goal sent");
}

void NavigationAgent::cancel_action()
{
  if (!goal_handle_ ||
    (goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING &&
    goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_ACCEPTED))
  {
    RCLCPP_WARN(this->get_logger(), "Cancel called with no active goal.");
    return;
  }
  try {
    const auto future_status =
      navigation_client_->async_cancel_goal(goal_handle_).wait_for(std::chrono::seconds(5));
    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for navigation goal to cancel.");
    }
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal: Unknown goal handle");
  }
}

void NavigationAgent::update_robot_pose_in_dsr(geometry_msgs::msg::Pose pose)
{
  // Update the robot pose into the DSR graph
  if (auto robot_node = G_->get_node(source_); robot_node.has_value()) {
    if (G_->get_priority(robot_node.value()) == 0) {
      G_->add_or_modify_attrib_local<pose_x_att>(
        robot_node.value(), static_cast<float>(pose.position.x));
      G_->add_or_modify_attrib_local<pose_y_att>(
        robot_node.value(), static_cast<float>(pose.position.y));
      G_->add_or_modify_attrib_local<pose_angle_att>(
        robot_node.value(), static_cast<float>(tf2::getYaw(pose.orientation)));
      G_->update_node(robot_node.value());
    }
  }
}

}  // namespace dsr_agents

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_agents::NavigationAgent)
