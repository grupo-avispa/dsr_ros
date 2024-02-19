/*
 * DSR BRIDGE ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * Copyright (c) 2024 Óscar Pons Fernández <oscarpf22@gmail.com>
 * Copyright (c) 2024 José Galeas Merchan <jgaleas1999@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENTS__DSR_BRIDGE_HPP_
#define DSR_AGENTS__DSR_BRIDGE_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "dsr_interfaces/msg/edge.hpp"
#include "dsr_interfaces/msg/node.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_agents/agent_node.hpp"

class DSRBridge: public AgentNode{
	public:
		DSRBridge();

	private:
		rclcpp::Subscription<dsr_interfaces::msg::Edge>::SharedPtr edge_from_ros_sub_;
		rclcpp::Subscription<dsr_interfaces::msg::Node>::SharedPtr node_from_ros_sub_;
		rclcpp::Publisher<dsr_interfaces::msg::Edge>::SharedPtr edge_to_ros_pub_;
		rclcpp::Publisher<dsr_interfaces::msg::Node>::SharedPtr node_to_ros_pub_;
		std::string edge_topic_, node_topic_;

		void get_params();

		// ROS callbacks
		void edge_from_ros_callback(const dsr_interfaces::msg::Edge::SharedPtr msg);
		void node_from_ros_callback(const dsr_interfaces::msg::Node::SharedPtr msg);

		// DSR callbacks
		void node_created(std::uint64_t id, const std::string &type);
		void node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names);
		void edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type);
		void edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
			const std::string &type, const std::vector<std::string>& att_names);
		void node_deleted(const DSR::Node &node);
		void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);

		// Helper functions
		std::optional<DSR::Node> create_dsr_node(std::string name, std::string type);
		std::optional<DSR::Edge> create_dsr_edge(
			std::string from, std::string to, const std::string &type);
		dsr_interfaces::msg::Node create_msg_node(std::string name, std::string type);
		dsr_interfaces::msg::Edge create_msg_edge(
			std::uint64_t from, std::uint64_t to, const std::string &type);
		void modify_node_attributes(DSR::Node & node, std::vector <std::string>& att_str);
		std::string attribute_to_string(const DSR::Attribute &att);
};

#endif  // DSR_AGENT__DSR_BRIDGE_HPP_
