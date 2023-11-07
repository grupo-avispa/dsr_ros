/*
 * ACTION AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__ACTION_AGENT_HPP_
#define DSR_AGENT__ACTION_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_agents/agent_node.hpp"


template<typename ActionT>
class actionAgent: public AgentNode{
	public:
		actionAgent();

	private:
		using GoalHandleActionT = rclcpp_action::ClientGoalHandle<ActionT>;

		std::string dsr_node_name_;
		rclcpp_action::Client<ActionT>::SharedPtr action_client_;
		std::shared_ptr<GoalHandleActionT> goal_handle_;

		void get_params();

		// Action callbacks
		void goal_response_callback(const GoalHandleActionT::SharedPtr & goal_handle);
		void feedback_callback(GoalHandleActionT::SharedPtr, 
			const std::shared_ptr<const ActionT::Feedback> feedback);
		void result_callback(const GoalHandleActionT::WrappedResult & result);

		// DSR callbacks
		void node_updated(std::uint64_t id, const std::string &type);
		void node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names);
		void edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type);
		void edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
			const std::string &type, const std::vector<std::string>& att_names);
		void node_deleted(std::uint64_t id);
		void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);
};

#endif  // DSR_AGENT__ACTION_AGENT_HPP_
