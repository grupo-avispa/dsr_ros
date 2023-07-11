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
#include "nav2_msgs/action/navigate_to_pose.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_agent/agent_node.hpp"

class navigationAgent: public AgentNode{
	public:
		navigationAgent();

	private:
		using NavigateToPose = nav2_msgs::action::NavigateToPose;
		using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

		std::string dsr_node_name_;
		rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
		std::shared_ptr<GoalHandleNavigateToPose> goal_handle_;

		void get_params();
		void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
		void feedback_callback(GoalHandleNavigateToPose::SharedPtr, 
			const std::shared_ptr<const NavigateToPose::Feedback> feedback);
		void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
		void send_to_goal(geometry_msgs::msg::Pose goal_pose);
		void cancel_goal();

		void node_updated(std::uint64_t id, const std::string &type);
		void node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names);
		void edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type);
		void edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
			const std::string &type, const std::vector<std::string>& att_names);
		void node_deleted(std::uint64_t id);
		void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);
};

#endif  // DSR_AGENT__NAV_AGENT_HPP_
