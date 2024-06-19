/*
 * SEMANTIC NAVIGATION AGENT ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENTS__SEMANTIC_NAV_AGENT_HPP_
#define DSR_AGENTS__SEMANTIC_NAV_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"
#include "semantic_navigation_msgs/srv/list_all_regions.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

class SemanticNavigationAgent: public AgentNode{
	public:
		SemanticNavigationAgent();

	private:
		using SemanticGoals = semantic_navigation_msgs::srv::GenerateRandomGoals;
		using SemanticRegions = semantic_navigation_msgs::srv::ListAllRegions;

		rclcpp::Client<SemanticGoals>::SharedPtr goals_generator_client_;
		rclcpp::Client<SemanticRegions>::SharedPtr semantic_regions_client_;
		std::vector<std::string> zones_;

		geometry_msgs::msg::Pose generate_goal(std::string room_name, int n_goals = 1);
		void get_zones();

		// DSR callbacks
		void node_updated(std::uint64_t id, const std::string &type){};
		void node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names){};
		void edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type);
		void edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
			const std::string &type, const std::vector<std::string>& att_names){};
		void node_deleted(std::uint64_t id){};
		void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
};

#endif  // DSR_AGENTS__SEMANTIC_NAV_AGENT_HPP_
