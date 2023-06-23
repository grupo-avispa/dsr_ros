/*
 * TF AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__TF_AGENT_HPP_
#define DSR_AGENT__TF_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// DSR
#include "dsr/api/dsr_api.h"

class tfAgent: public QObject, public rclcpp::Node{
	Q_OBJECT
	public:
		tfAgent();
		~tfAgent();

	public slots:
		void node_updated(std::uint64_t id, const std::string &type);
		void node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names);
		void edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type);
		void edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
			const std::string &type, const std::vector<std::string>& att_names);
		void node_deleted(std::uint64_t id);
		void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);

	private:
		rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_, tf_static_sub_;

		// DSR graph
		std::shared_ptr<DSR::DSRGraph> G_;
		std::unique_ptr<DSR::RT_API> rt_;
		int agent_id_;
		std::string agent_name_, dsr_input_file_;

		void get_params();
		template <typename NODE_TYPE> std::optional<uint64_t> create_and_insert_node(const std::string &name);

		void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
};

#endif  // DSR_AGENT__DSR_AGENT_HPP_
