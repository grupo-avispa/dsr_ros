/*
 * NAVIGATION AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__NAV_AGENT_HPP_
#define DSR_AGENT__NAV_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "auto_docking_interfaces/action/dock.hpp"
#include "auto_docking_interfaces/action/undock.hpp"
#include "semantic_navigation_msgs/srv/semantic_goals.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_agents/agent_node.hpp"

class navigationAgent: public AgentNode{
	public:
		navigationAgent();

	private:
		using NavigateToPose = nav2_msgs::action::NavigateToPose;
		using Dock = auto_docking_interfaces::action::Dock;
		using Undock = auto_docking_interfaces::action::Undock;
		using SemanticGoals = semantic_navigation_msgs::srv::SemanticGoals;
		using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
		using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;
		using GoalHandleUndock = rclcpp_action::ClientGoalHandle<Undock>;

		std::string dsr_node_name_;
		rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
		rclcpp_action::Client<Dock>::SharedPtr dock_client_;
		rclcpp_action::Client<Undock>::SharedPtr undock_client_;
		rclcpp::Client<SemanticGoals>::SharedPtr goals_generator_client_;
		std::shared_ptr<GoalHandleNavigateToPose> goal_handle_;
		std::string current_room_;

		void get_params();
		void send_to_room(std::string room_name, int n_goals = 1);
		void send_to_goal(geometry_msgs::msg::Pose goal_pose);
		void cancel_goal();
		void start_docking();
		void start_undocking();

		// Action callbacks
		void nav_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
		void nav_feedback_callback(GoalHandleNavigateToPose::SharedPtr, 
			const std::shared_ptr<const NavigateToPose::Feedback> feedback);
		void nav_result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
		void dock_goal_response_callback(const GoalHandleDock::SharedPtr & goal_handle);
		void dock_feedback_callback(GoalHandleDock::SharedPtr, 
							const std::shared_ptr<const Dock::Feedback> feedback);
		void dock_result_callback(const GoalHandleDock::WrappedResult & result);
		void undock_goal_response_callback(const GoalHandleUndock::SharedPtr & goal_handle);
		void undock_feedback_callback(GoalHandleUndock::SharedPtr, 
							const std::shared_ptr<const Undock::Feedback> feedback);
		void undock_result_callback(const GoalHandleUndock::WrappedResult & result);

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