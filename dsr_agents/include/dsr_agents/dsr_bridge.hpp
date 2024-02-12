/*
 * DSR BRIDGE ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
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

		std::optional<DSR::Node> createNode(std::string nodeType, std::string nodeName);
		std::optional<DSR::Edge> createEdge(std::string from, std::string to, const std::string &type);
		void modifyNodeAttribute(DSR::Node & node, std::vector <std::string>& attributes);
};

#endif  // DSR_AGENT__DSR_BRIDGE_HPP_
