/*
 * NAVIGATION AGENT ROS NODE
 *
 * Copyright (c) 2023-2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENTS__NAV_AGENT_HPP_
#define DSR_AGENTS__NAV_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"
#include "semantic_navigation_msgs/srv/list_all_regions.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

class NavigationAgent: public AgentNode{
	public:
		NavigationAgent();

	private:
		using NavigateToPose = nav2_msgs::action::NavigateToPose;
		using SemanticGoals = semantic_navigation_msgs::srv::GenerateRandomGoals;
		using SemanticRegions = semantic_navigation_msgs::srv::ListAllRegions;
		using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

		rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
		rclcpp::Client<SemanticGoals>::SharedPtr goals_generator_client_;
		rclcpp::Client<SemanticRegions>::SharedPtr semantic_regions_client_;
		std::shared_ptr<GoalHandleNavigateToPose> goal_handle_;
		std::vector<std::string> zones_;
		std::string current_zone_;
		std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

		void send_to_room(std::string room_name, int n_goals = 1);
		void send_to_goal(geometry_msgs::msg::Pose goal_pose);
		void get_zones();
		void update_robot_pose_in_dsr(geometry_msgs::msg::Pose pose);
		void cancel_action();

		// Navigation action callbacks
		void nav_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
		void nav_feedback_callback(GoalHandleNavigateToPose::SharedPtr, 
			const std::shared_ptr<const NavigateToPose::Feedback> feedback);
		void nav_result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

		// DSR callbacks
		void node_updated(std::uint64_t id, const std::string &type);
		void node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names);
		void edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type);
		void edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
			const std::string &type, const std::vector<std::string>& att_names);
		void node_deleted(std::uint64_t id);
		void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);
};

#endif  // DSR_AGENT__NAV_AGENT_HPP_
