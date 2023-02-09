/*
 * DSR AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agent.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__DSR_AGENT_HPP_
#define DSR_AGENT__DSR_AGENT_HPP_

// C++
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialization.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"

class dsrAgent: public rclcpp::Node{
	public:
		dsrAgent();
		~dsrAgent();
	private:
		rclcpp::GenericSubscription::SharedPtr generic_sub_;
		std::string ros_topic_;

		// DSR graph
		std::shared_ptr<DSR::DSRGraph> G_;
		std::string agent_name_;
		int agent_id_;
		std::string dsr_node_name_;

		// DSR graph viewer
		//std::unique_ptr<DSR::DSRViewer> graph_viewer_;

		void get_params();
		template <typename NODE_TYPE> void create_and_insert_node(const std::string &name);
		template <typename ROS_TYPE> void modify_node_attributes(std::optional<DSR::Node> &node, 
																const ROS_TYPE &msg);
		template <typename ROS_TYPE, typename NODE_TYPE> void deserialize_and_update_attributes(
			const std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &node_name);

		void serial_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg);
};

#endif  // DSR_AGENT__DSR_AGENT_HPP_
