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
#include <type_traits>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform.hpp"

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
		std::unique_ptr<DSR::RT_API> rt_;

		template <typename NODE_TYPE, typename EDGE_TYPE> 
		void add_node(const std::string & name, const std::string & parent_name){
			// Create node
			auto new_node = DSR::Node::create<NODE_TYPE>(name);
			// Check if parent exists
			if (auto parent_node = G_->get_node(parent_name); parent_node.has_value()){
				// Add attributes related to the parent node
				G_->add_or_modify_attrib_local<parent_att>(new_node, parent_node.value().id());
				G_->add_or_modify_attrib_local<level_att>(new_node, 
					G_->get_node_level(parent_node.value()).value() + 1);
				// Draw in the graph
				std::tuple<float, float> graph_pos;
				if (std::is_same<EDGE_TYPE, RT_edge_type>::value){
					graph_pos = get_position_by_level_in_graph(parent_node.value());
				}else{
					graph_pos = get_random_position_to_draw_in_graph();
				}
				const auto &[random_x, random_y] = graph_pos;
				G_->add_or_modify_attrib_local<pos_x_att>(new_node, random_x);
				G_->add_or_modify_attrib_local<pos_y_att>(new_node, random_y);
				// Insert node
				if (auto id = G_->insert_node(new_node); id.has_value()){
					RCLCPP_INFO(this->get_logger(), 
						"Inserted [%s] node successfully with id [%lu]", name.c_str(), id.value());
					// Insert edge
					auto new_edge = DSR::Edge::create<EDGE_TYPE>(parent_node.value().id(), 
						new_node.id());
					// TODO: Remove this sleep when they fix the DSR
					std::this_thread::sleep_for(std::chrono::milliseconds(50));
					if (G_->insert_or_assign_edge(new_edge)){
						RCLCPP_DEBUG_STREAM(this->get_logger(), "Inserted new edge [" 
							<< parent_node.value().name() << "->" 
							<< new_node.name() <<
							"] of type [" << new_edge.type().c_str() << "]");
					}else{
						RCLCPP_ERROR_STREAM(this->get_logger(), "The edge [" 
							<< parent_node.value().name() << "->" 
							<< new_node.name() <<
							"] of type [" << new_edge.type().c_str() << "] couldn't be inserted");
					}
				}else{
					RCLCPP_ERROR(this->get_logger(), "Error inserting [%s] node", name.c_str());
				}
			}else{
				RCLCPP_WARN(this->get_logger(), "Parent node [%s] doesn't exist", 
					parent_name.c_str());
				// Add default attributes
				G_->add_or_modify_attrib_local<parent_att>(new_node, static_cast<uint64_t>(0));
				G_->add_or_modify_attrib_local<level_att>(new_node, 0);
				// Draw a random position in the graph
				const auto &[random_x, random_y] = get_random_position_to_draw_in_graph();
				G_->add_or_modify_attrib_local<pos_x_att>(new_node, random_x);
				G_->add_or_modify_attrib_local<pos_y_att>(new_node, random_y);
				// Insert node
				if (auto id = G_->insert_node(new_node); id.has_value()){
					RCLCPP_INFO(this->get_logger(), 
						"Inserted [%s] node successfully with id [%lu]", name.c_str(), id.value());
				}else{
					RCLCPP_ERROR(this->get_logger(), "Error inserting [%s] node", name.c_str());
				}
			}
		}

		template <typename EDGE_TYPE> 
		void add_edge(uint64_t from, uint64_t to){
			auto new_edge = DSR::Edge::create<EDGE_TYPE>(from, to);
			std::optional<DSR::Node> parent_node = G_->get_node(from);
			std::optional<DSR::Node> child_node = G_->get_node(to);
			if (G_->insert_or_assign_edge(new_edge)){
				RCLCPP_DEBUG_STREAM(this->get_logger(), "Inserted new edge [" 
					<< parent_node.value().name() << "->" 
					<< child_node.value().name() <<
					"] of type [" << new_edge.type().c_str() << "]");
			}else{
				RCLCPP_ERROR_STREAM(this->get_logger(), "The edge [" 
					<< parent_node.value().name() << "->" 
					<< child_node.value().name() <<
					"] of type [" << new_edge.type().c_str() << "] couldn't be inserted");
			}
		}

		void update_rt_attributes(DSR::Node & from, DSR::Node & to, 
			const geometry_msgs::msg::Transform & msg);
	
	private:
		rclcpp::Service<dsr_interfaces::srv::SaveDSR>::SharedPtr save_dsr_service_;

		int agent_id_;
		std::string dsr_input_file_;

		void get_common_params();
		void save_dsr(const std::shared_ptr<dsr_interfaces::srv::SaveDSR::Request> request,
			std::shared_ptr<dsr_interfaces::srv::SaveDSR::Response> response);
		std::tuple<float, float> get_position_by_level_in_graph(const DSR::Node & parent);
		std::tuple<float, float> get_random_position_to_draw_in_graph();
};

#endif  // DSR_AGENT__AGENT_NODE_HPP_
