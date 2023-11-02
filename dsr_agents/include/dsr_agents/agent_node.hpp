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
#include "dsr_agents/ros_to_dsr_types.hpp"

/**
 * @brief Base class to connect the DSR graph with ROS 2. It contains common methods and attributes
 * to send data from ROS 2 to the DSR graph and vice versa. All agents must inherit from this class.
 * 
 */
class AgentNode: public QObject, public rclcpp::Node{
	Q_OBJECT
	public:
		/**
		 * @brief Construct a new Agent Node object.
		 * 
		 * @param ros_node_name Name of the ROS node and the DSR agent.
		 */
		explicit AgentNode(std::string ros_node_name);

		/**
		 * @brief Destroy the Agent Node object.
		 * 
		 */
		virtual ~AgentNode();

	protected:
		/**
		 * @brief Pointer to the DSR graph.
		 * 
		 */
		std::shared_ptr<DSR::DSRGraph> G_;

		/**
		 * @brief Pointer to the RT API.
		 * 
		 */
		std::unique_ptr<DSR::RT_API> rt_;

		/**
		 * @brief Add a node into the DSR graph with the given name, parent and priority.
		 * By default, all nodes have a low priority (0).
		 * 
		 * @tparam NODE_TYPE The type of the DSR node. Defined in ros_to_dsr_types.hpp.
		 * @tparam EDGE_TYPE The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
		 * @param name Name of the DSR node.
		 * @param parent_name Name of the parent DSR node.
		 * @param priority Priority of the DSR node.
		 */
		template <typename NODE_TYPE, typename EDGE_TYPE> 
		void add_node(const std::string & name, const std::string & parent_name, 
			const int & priority = 0){
			// Create the node
			auto new_node = DSR::Node::create<NODE_TYPE>(name);
			// By default, all nodes have a low priority
			G_->add_or_modify_attrib_local<priority_att>(new_node, priority);
			// Check if the parent exists
			if (auto parent_node = G_->get_node(parent_name); parent_node.has_value()){
				// Add default parent and level attributes
				G_->add_or_modify_attrib_local<parent_att>(new_node, parent_node.value().id());
				G_->add_or_modify_attrib_local<level_att>(new_node, 
					G_->get_node_level(parent_node.value()).value() + 1);
				// Draw the node in the graph: by level if RT edge, random if not
				std::tuple<float, float> graph_pos;
				if (std::is_same<EDGE_TYPE, RT_edge_type>::value){
					graph_pos = get_position_by_level_in_graph(parent_node.value());
				}else{
					graph_pos = get_random_position_to_draw_in_graph();
				}
				const auto &[random_x, random_y] = graph_pos;
				G_->add_or_modify_attrib_local<pos_x_att>(new_node, random_x);
				G_->add_or_modify_attrib_local<pos_y_att>(new_node, random_y);
				// Insert the node into the DSR graph
				if (auto id = G_->insert_node(new_node); id.has_value()){
					RCLCPP_INFO(this->get_logger(), 
						"Inserted [%s] node successfully with id [%lu]", name.c_str(), id.value());
					// Insert the edge into the DSR graph
					auto new_edge = DSR::Edge::create<EDGE_TYPE>(parent_node.value().id(), 
						new_node.id());
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
				// Add default parent and level attributes
				G_->add_or_modify_attrib_local<parent_att>(new_node, static_cast<uint64_t>(0));
				G_->add_or_modify_attrib_local<level_att>(new_node, 0);
				// Draw a random position in the graph
				const auto &[random_x, random_y] = get_random_position_to_draw_in_graph();
				G_->add_or_modify_attrib_local<pos_x_att>(new_node, random_x);
				G_->add_or_modify_attrib_local<pos_y_att>(new_node, random_y);
				// Insert the node into the DSR graph
				if (auto id = G_->insert_node(new_node); id.has_value()){
					RCLCPP_INFO(this->get_logger(), 
						"Inserted [%s] node successfully with id [%lu]", name.c_str(), id.value());
				}else{
					RCLCPP_ERROR(this->get_logger(), "Error inserting [%s] node", name.c_str());
				}
			}
		}

		/**
		 * @brief Add an edge into the DSR graph with the given parent and child nodes id.
		 * 
		 * @tparam EDGE_TYPE The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
		 * @param from Id of the parent DSR node.
		 * @param to  Id of the child DSR node.
		 */
		template <typename EDGE_TYPE> 
		void add_edge(uint64_t from, uint64_t to){
			// Create the edge
			auto new_edge = DSR::Edge::create<EDGE_TYPE>(from, to);
			std::optional<DSR::Node> parent_node = G_->get_node(from);
			std::optional<DSR::Node> child_node = G_->get_node(to);
			// Insert the edge into the DSR graph
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

		/**
		 * @brief Delete an edge into the DSR graph with the given parent and child nodes id and
		 * the edge type. This method previously checks if the parent and child nodes exist.
		 * 
		 * @param from Id of the parent DSR node.
		 * @param to Id of the child DSR node.
		 * @param edge_type Name of the DSR edge.
		 * @return true If the edge was replaced successfully.
		 * @return false If the edge couldn't be replaced.
		 */
		bool delete_edge(uint64_t from, uint64_t to, std::string edge_type){
			// Check if the parent and child nodes exist and if the edge exists
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
			if (parent_node.has_value() && child_node.has_value()){
				if (auto edge = G_->get_edge(from, to, edge_type); edge.has_value()){
					// Delete the edge
					if (G_->delete_edge(from, to, edge_type)){
						RCLCPP_INFO(this->get_logger(), "Deleting %s edge", edge_type.c_str());
						return true;
					}else{
						RCLCPP_ERROR(this->get_logger(), "Error deleting %s edge", 
							edge_type.c_str());
					}
				}else{
					RCLCPP_ERROR(this->get_logger(), "Edge %s doesn't exist", 
						edge_type.c_str());
				}
			}
			return false;
		}

		/**
		 * @brief Delete an edge into the DSR graph with the given parent and child nodes names and
		 * the edge type. This method previously checks if the parent and child nodes exist.
		 * 
		 * @param from Name of the parent DSR node.
		 * @param to Name of the child DSR node.
		 * @param edge_type Name of the DSR edge.
		 * @return true If the edge was replaced successfully.
		 * @return false If the edge couldn't be replaced.
		 */
		bool delete_edge(const std::string& from, const std::string& to, std::string edge_type){
			// Check if the parent and child nodes exist
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
				if (parent_node.has_value() && child_node.has_value()){
					return delete_edge(parent_node.value().id(), 
						child_node.value().id(), edge_type);
				}
			return false;
		}

		/**
		 * @brief Replace an edge into the DSR graph with the given parent and child nodes id and
		 * the old edge type. This method previously checks if the parent and child nodes exist.
		 * 
		 * @tparam EDGE_TYPE The type of the new DSR edge. Defined in ros_to_dsr_types.hpp.
		 * @param from Id of the parent DSR node.
		 * @param to Id of the child DSR node.
		 * @param old_edge Name of the old DSR edge.
		 * @return true If the edge was replaced successfully.
		 * @return false If the edge couldn't be replaced.
		 */
		template <typename EDGE_TYPE>
		bool replace_edge(uint64_t from, uint64_t to, std::string old_edge){
			// Check if the parent and child nodes exist and if the old edge exists
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
			if (parent_node.has_value() && child_node.has_value()){
				if (auto edge = G_->get_edge(from, to, old_edge); edge.has_value()){
					// Delete the old edge
					if (G_->delete_edge(from, to, old_edge)){
						// Create the new edge
						auto new_edge = DSR::Edge::create<EDGE_TYPE>(from, to);
						// Insert the new edge into the DSR graph
						if (G_->insert_or_assign_edge(new_edge)){
							RCLCPP_INFO(this->get_logger(), "Inserting %s edge", 
								new_edge.type().c_str());
							return true;
						}
					}else{
						RCLCPP_ERROR(this->get_logger(), "Error deleting %s edge", 
							old_edge.c_str());
					}
				}else{
					RCLCPP_ERROR(this->get_logger(), "Edge %s doesn't exist", 
						old_edge.c_str());
				}
			}
			return false;
		}

		/**
		 * @brief Replace an edge into the DSR graph with the given parent and child nodes names and
		 * the old edge type. This method previously checks if the parent and child nodes exist.
		 * 
		 * @tparam EDGE_TYPE The type of the new DSR edge. Defined in ros_to_dsr_types.hpp.
		 * @param from Name of the parent DSR node.
		 * @param to Name of the child DSR node.
		 * @param old_edge Name of the old DSR edge.
		 * @return true If the edge was replaced successfully.
		 * @return false If the edge couldn't be replaced.
		 */
		template <typename EDGE_TYPE>
		bool replace_edge(const std::string& from, const std::string& to, std::string old_edge){
			// Check if the parent and child nodes exist
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
				if (parent_node.has_value() && child_node.has_value()){
					return replace_edge<EDGE_TYPE>(parent_node.value().id(), 
						child_node.value().id(), old_edge);
				}
			return false;
		}

		/**
		 * @brief Update the RT atributes (position and orientation) of the given node in the DSR graph
		 * with the given ROS Transform message.
		 * 
		 * @param from Parent DSR node.
		 * @param to Child DSR node.
		 * @param msg Transform message.
		 */
		void update_rt_attributes(DSR::Node & from, DSR::Node & to, 
			const geometry_msgs::msg::Transform & msg);

		/**
		 * @brief Get the frame id object.
		 * 
		 * @tparam ROS_TYPE The type of the ROS message.
		 * @param msg ROS message.
		 * @return std::string The frame id of the ROS message.
		 */
		template <typename ROS_TYPE>
		std::string get_frame_id(ROS_TYPE msg){
			return msg.header.frame_id;
		}

	private:
		/**
		 * @brief Service to save the DSR graph into a file.
		 * 
		 */
		rclcpp::Service<dsr_interfaces::srv::SaveDSR>::SharedPtr save_dsr_service_;

		/**
		 * @brief Id of the DSR agent.
		 * 
		 */
		int agent_id_;

		/**
		 * @brief Name of the input file to load the DSR graph from.
		 * 
		 */
		std::string dsr_input_file_;

		/**
		 * @brief Initialize ROS parameters.
		 * 
		 */
		void get_common_params();

		/**
		 * @brief Save the DSR graph into a file.
		 * 
		 * @param request URL of the file.
		 * @param response True if the DSR graph was saved successfully, false otherwise.
		 */
		void save_dsr(const std::shared_ptr<dsr_interfaces::srv::SaveDSR::Request> request,
			std::shared_ptr<dsr_interfaces::srv::SaveDSR::Response> response);

		/**
		 * @brief Get the position by level in graph object.
		 * 
		 * @param parent Parent DSR node.
		 * @return std::tuple<float, float> Position (x, y) of the child DSR node.
		 */
		std::tuple<float, float> get_position_by_level_in_graph(const DSR::Node & parent);

		/**
		 * @brief Get the random position to draw in graph object.
		 * 
		 * @return std::tuple<float, float> Position (x, y) of the DSR node.
		 */
		std::tuple<float, float> get_random_position_to_draw_in_graph();
};

#endif  // DSR_AGENT__AGENT_NODE_HPP_
