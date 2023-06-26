/*
 * AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__AGENT_NODE_HPP_
#define DSR_AGENT__AGENT_NODE_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"

// DSR
#include "dsr/api/dsr_api.h"

#include "dsr_interfaces/srv/save_dsr.hpp"

class AgentNode: public QObject, public rclcpp::Node, public std::enable_shared_from_this<AgentNode>{
	Q_OBJECT
	public:
		explicit AgentNode(std::string node_name);
		virtual ~AgentNode();

	public slots:
		/*void node_updated(std::uint64_t id, const std::string &type);
		void node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names);
		void edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type);
		void edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
			const std::string &type, const std::vector<std::string>& att_names);
		void node_deleted(std::uint64_t id);
		void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);*/

	protected:
		// DSR graph
		std::shared_ptr<DSR::DSRGraph> G_;

		template <typename NODE_TYPE> 
		std::optional<uint64_t> create_and_insert_node(const std::string &name){
			// Create node
			auto new_dsr_node = DSR::Node::create<NODE_TYPE>(name);
			// Add default level attribute
			G_->add_or_modify_attrib_local<level_att>(new_dsr_node, 0);
			// Insert node
			auto id = G_->insert_node(new_dsr_node);
			if (id.has_value()){
				RCLCPP_INFO(this->get_logger(), 
					"Inserted [%s] node successfully with id [%lu]", name.c_str(), id.value());
			}else{
				RCLCPP_ERROR(this->get_logger(), "Error inserting [%s] node", name.c_str());
			}
			return id;
		}

		template <typename EDGE_TYPE> 
		void create_and_insert_edge(uint64_t from, uint64_t to){
			auto new_edge = DSR::Edge::create<EDGE_TYPE>(from, to);
			if (G_->insert_or_assign_edge(new_edge)){
				RCLCPP_INFO_STREAM(this->get_logger(), "Inserted new edge [" << from << "->" << to <<
					"] of type [" << new_edge.type().c_str() << "]");
			}else{
				RCLCPP_ERROR_STREAM(this->get_logger(), "The edge [" << from << "->" << to <<
					"] of type [" << new_edge.type().c_str() << "] couldn't be inserted");
			}
		}
	
	private:
		rclcpp::Service<dsr_interfaces::srv::SaveDSR>::SharedPtr save_dsr_service_;

		int agent_id_;
		std::string agent_name_, dsr_input_file_;

		void get_common_params();
		void save_dsr(const std::shared_ptr<dsr_interfaces::srv::SaveDSR::Request> request,
			std::shared_ptr<dsr_interfaces::srv::SaveDSR::Response> response);
};

#endif  // DSR_AGENT__AGENT_NODE_HPP_
