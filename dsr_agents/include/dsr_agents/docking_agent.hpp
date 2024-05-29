/*
 * DOCKING AGENT ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENTS__DOCKING_AGENT_HPP_
#define DSR_AGENTS__DOCKING_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "opennav_docking_msgs/action/undock_robot.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

class DockingAgent: public AgentNode{
	public:
		DockingAgent();

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
		void dock_feedback_callback(GoalHandleDock::SharedPtr, 
							const std::shared_ptr<const Dock::Feedback> feedback);
		void dock_result_callback(const GoalHandleDock::WrappedResult & result);

		// Undocking action callbacks
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

#endif  // DSR_AGENT__DOCKING_AGENT_HPP_
