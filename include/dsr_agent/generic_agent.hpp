/*
 * GENERIC AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agent.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__GENERIC_AGENT_HPP_
#define DSR_AGENT__GENERIC_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialization.hpp"

// DSR
#include "dsr/api/dsr_api.h"

class genericAgent: public QObject, public rclcpp::Node{
	Q_OBJECT
	public:
		genericAgent();
		~genericAgent();

	public slots:
		void node_updated(std::uint64_t id, const std::string &type);
		void node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names);
		void edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type);
		void edge_attributes_updated(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names);
		void node_deleted(std::uint64_t id);
		void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);

	private:
		rclcpp::GenericSubscription::SharedPtr generic_sub_;
		std::string ros_topic_;

		// DSR graph
		std::shared_ptr<DSR::DSRGraph> G_;
		int agent_id_;
		std::string agent_name_;
		std::string dsr_node_name_, dsr_parent_node_name_;

		void get_params();
		template <typename NODE_TYPE> std::optional<uint64_t> create_and_insert_node(const std::string &name);
		template <typename EDGE_TYPE> void create_and_insert_edge(uint64_t from, uint64_t to);
		template <typename ROS_TYPE> void modify_node_attributes(std::optional<DSR::Node> &node, 
																const ROS_TYPE &msg);
		template <typename ROS_TYPE, typename NODE_TYPE, typename EDGE_TYPE> 
			void deserialize_and_update_attributes(
				const std::shared_ptr<rclcpp::SerializedMessage> msg, 
				const std::string &node_name, const std::string &parent_name);

		void serial_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg);
};

#endif  // DSR_AGENT__GENERIC_AGENT_HPP_
