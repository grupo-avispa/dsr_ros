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

#ifndef DSR_AGENTS__AGENT_NODE_HPP_
#define DSR_AGENTS__AGENT_NODE_HPP_

// C++
#include <string>
#include <tuple>
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
		 * @brief Name of the physical machine.
		 * 
		 */
		std::string source_;

		/**
		 * @brief Add a node into the DSR graph with the given name and type.
		 * By default, all nodes have a low priority (0) and the source attribute is set 
		 * to the name of the physical machine.
		 * 
		 * @tparam NODE_TYPE The type of the DSR node. Defined in ros_to_dsr_types.hpp.
		 * @param name Name of the DSR node.
		 * @return std::optional<DSR::Node> The DSR node if it was added successfully,
		 */
		template <typename NODE_TYPE> 
		std::optional<DSR::Node> add_node(const std::string & name){
			std::optional<DSR::Node> return_node;
			// Create the node
			auto new_node = DSR::Node::create<NODE_TYPE>(name);
			// Add default values
			G_->add_or_modify_attrib_local<priority_att>(new_node, 0);
			G_->add_or_modify_attrib_local<source_att>(new_node, source_);
			// Draw the node in the graph randomly
			const auto &[random_x, random_y] = get_random_position_to_draw_in_graph();
			G_->add_or_modify_attrib_local<pos_x_att>(new_node, random_x);
			G_->add_or_modify_attrib_local<pos_y_att>(new_node, random_y);
			// Insert the node into the DSR graph
			if (auto id = G_->insert_node(new_node); id.has_value()){
				return_node = new_node;
				RCLCPP_INFO(this->get_logger(), 
					"Inserted node [%s] successfully of type [%s]", 
					name.c_str(), new_node.type().c_str());
			}else{
				RCLCPP_ERROR(this->get_logger(), 
					"The node [%s] couldn't be inserted", name.c_str());
			}
			return return_node;
		}

		/**
		 * @brief Add a node with an edge into the DSR graph with the given name, 
		 * the name of the parent or the child and the 'direction' of the edge.
		 * By default, all nodes have a low priority (0) and the source attribute is set 
		 * to the name of the physical machine.
		 * 
		 * @tparam NODE_TYPE The type of the DSR node. Defined in ros_to_dsr_types.hpp.
		 * @tparam EDGE_TYPE The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
		 * @param name Name of the DSR node.
		 * @param from_to_name Name of the parent or the child DSR node.
		 * @param as_child True if the node to be added is a child, false if it is a parent.
		 * @return std::tuple<std::optional<DSR::Node>, std::optional<DSR::Edge>>
		 * The DSR node and the DSR edge if they were added successfully.
		 */
		template <typename NODE_TYPE, typename EDGE_TYPE> 
		std::tuple<std::optional<DSR::Node>, std::optional<DSR::Edge>>
		add_node_with_edge(const std::string & name, 
			const std::string & from_to_name, const bool as_child = true){
			std::optional<DSR::Node> return_node;
			std::optional<DSR::Edge> return_edge;
			// Get the relative node
			auto relative_node = G_->get_node(from_to_name);
			// Create the node
			auto new_node = DSR::Node::create<NODE_TYPE>(name);
			// Add default values
			uint64_t relative_attribute_value = relative_node.has_value() ? relative_node.value().id() : 0;
			int level_attribute_value = relative_node.has_value() ? 
				G_->get_node_level(relative_node.value()).value() + 1 : 0;
			G_->add_or_modify_attrib_local<priority_att>(new_node, 0);
			G_->add_or_modify_attrib_local<source_att>(new_node, source_);
			G_->add_or_modify_attrib_local<parent_att>(new_node, relative_attribute_value);
			G_->add_or_modify_attrib_local<level_att>(new_node, level_attribute_value);
			// Draw the node in the graph: by level if RT edge and parent, random if not
			std::tuple<float, float> graph_pos;
			if (relative_node.has_value() && std::is_same<EDGE_TYPE, RT_edge_type>::value){
				graph_pos = get_position_by_level_in_graph(relative_node.value());
			}else{
				graph_pos = get_random_position_to_draw_in_graph();
			}
			const auto &[random_x, random_y] = graph_pos;
			G_->add_or_modify_attrib_local<pos_x_att>(new_node, random_x);
			G_->add_or_modify_attrib_local<pos_y_att>(new_node, random_y);
			// Insert the node into the DSR graph
			if (auto id = G_->insert_node(new_node); id.has_value()){
				return_node = new_node;
				RCLCPP_INFO(this->get_logger(), 
					"Inserted node [%s] successfully of type [%s]", 
					name.c_str(), new_node.type().c_str());
				// Insert the edge into the DSR graph
				if (as_child){
					return_edge = add_edge<EDGE_TYPE>(from_to_name, name);
				}else{
					return_edge = add_edge<EDGE_TYPE>(name, from_to_name);
				}
			}else{
				RCLCPP_ERROR(this->get_logger(), 
					"The node [%s] couldn't be inserted", name.c_str());
			}
			return std::make_tuple(return_node, return_edge);
		}

		/**
		 * @brief Add an edge into the DSR graph with the given parent and child nodes names.
		 * By default, all edges have the source attribute set to the name of the physical machine.
		 * 
		 * @tparam EDGE_TYPE The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
		 * @param from Name of the parent DSR node.
		 * @param to  Name of the child DSR node.
		 * @return std::optional<DSR::Edge> The DSR edge if it was added successfully.
		 */
		template <typename EDGE_TYPE> 
		std::optional<DSR::Edge> add_edge(const std::string & from, const std::string & to){
			std::optional<DSR::Edge> return_edge;
			// Get the relatives nodes
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
			// Insert the edge into the DSR graph
			if (parent_node.has_value()){
				if (child_node.has_value()){
					// Create the edge
					auto new_edge = DSR::Edge::create<EDGE_TYPE>(parent_node.value().id(), 
						child_node.value().id());
					G_->add_or_modify_attrib_local<source_att>(new_edge, source_);
					// Insert the edge into the DSR graph
					if (G_->insert_or_assign_edge(new_edge)){
						return_edge = new_edge;
						RCLCPP_INFO(this->get_logger(), 
							"Inserted edge [%s->%s] successfully of type [%s]", 
							parent_node.value().name().c_str(), 
							child_node.value().name().c_str(), 
							new_edge.type().c_str());
					}else{
						RCLCPP_ERROR(this->get_logger(), 
							"The edge [%s->%s] of type [%s] couldn't be inserted",
							parent_node.value().name().c_str(), 
							child_node.value().name().c_str(), 
							new_edge.type().c_str());
					}
				}else{
					RCLCPP_ERROR(this->get_logger(), 
						"The edge couldn't be inserted because the child node [%s] doesn't exists",
						to.c_str());

				}
			}else{
				RCLCPP_ERROR(this->get_logger(), 
					"The edge couldn't be inserted because the parent node [%s] doesn't exists",
					from.c_str());
			}
			return return_edge;
		}

		/**
		 * @brief Add an edge into the DSR graph with the given parent and child nodes id.
		 * By default, all edges have the source attribute set to the name of the physical machine.
		 * 
		 * @tparam EDGE_TYPE The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
		 * @param from Id of the parent DSR node.
		 * @param to  Id of the child DSR node.
		 * @return std::optional<DSR::Edge> The DSR edge if it was added successfully.
		 */
		template <typename EDGE_TYPE> 
		std::optional<DSR::Edge> add_edge(uint64_t from, uint64_t to){
			std::optional<DSR::Edge> return_edge;
			// Get the relatives nodes
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
			// Insert the edge into the DSR graph
			if (parent_node.has_value() && child_node.has_value()){
				// Create the edge
				auto new_edge = DSR::Edge::create<EDGE_TYPE>(from, to);
				G_->add_or_modify_attrib_local<source_att>(new_edge, source_);
				// Insert the edge into the DSR graph
				if (G_->insert_or_assign_edge(new_edge)){
					return_edge = new_edge;
					RCLCPP_INFO(this->get_logger(), 
						"Inserted edge [%s->%s] successfully of type [%s]", 
						parent_node.value().name().c_str(), 
						child_node.value().name().c_str(), 
						new_edge.type().c_str());
				}else{
					RCLCPP_ERROR(this->get_logger(), 
						"The edge [%s->%s] of type [%s] couldn't be inserted",
						parent_node.value().name().c_str(), 
						child_node.value().name().c_str(), 
						new_edge.type().c_str());
				}
			}
			return return_edge;
		}

		/**
		 * @brief Delete a node into the DSR graph with the given id. 
		 * This method previously checks if the node exists.
		 * 
		 * @param id Id of the DSR node.
		 * @param to Id of the child DSR node.
		 * @return true If the node was deleted successfully. False otherwise.
		 */
		bool delete_node(uint64_t id){
			// Check if the node exists
			if (auto node = G_->get_node(id); node.has_value()){
				// Delete the node
				if (G_->delete_node(id)){
					RCLCPP_INFO(this->get_logger(), 
						"Deleted node [%s] successfully", node.value().name().c_str());
					return true;
				}else{
					RCLCPP_ERROR(this->get_logger(), 
						"The node [%s] couldn't be deleted", node.value().name().c_str());
				}
			}else{
				RCLCPP_WARN(this->get_logger(), 
					"The node [%lu] couldn't be deleted because it doesn't exists", id);
			}
			return false;
		}

		/**
		 * @brief Delete a node into the DSR graph with the given id. 
		 * This method previously checks if the node exists.
		 * 
		 * @param id Id of the DSR node.
		 * @param to Id of the child DSR node.
		 * @return true If the node was deleted successfully. False otherwise.
		 */
		bool delete_node(const std::string & name){
			return delete_node(G_->get_node(name).value().id());
		}

		/**
		 * @brief Delete an edge into the DSR graph with the given parent and child nodes id and
		 * the edge type. This method previously checks if the parent and child nodes exist.
		 * 
		 * @param from Id of the parent DSR node.
		 * @param to Id of the child DSR node.
		 * @param edge_type Name of the DSR edge.
		 * @return true If the edge was replaced successfully. False otherwise.
		 */
		bool delete_edge(uint64_t from, uint64_t to, std::string edge_type){
			// Check if the parent and child nodes exist and if the edge existsd
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
			if (parent_node.has_value() && child_node.has_value()){
				if (auto edge = G_->get_edge(from, to, edge_type); edge.has_value()){
					// Delete the edge
					if (G_->delete_edge(from, to, edge_type)){
						RCLCPP_INFO(this->get_logger(), 
							"Deleted edge [%s->%s] successfully of type [%s]", 
							parent_node.value().name().c_str(), 
							child_node.value().name().c_str(), 
							edge_type.c_str());
						return true;
					}else{
						RCLCPP_ERROR(this->get_logger(), 
							"The edge [%s->%s] of type [%s] couldn't be deleted",
							parent_node.value().name().c_str(), 
							child_node.value().name().c_str(), 
							edge_type.c_str());
					}
				}else{
					RCLCPP_WARN(this->get_logger(), 
						"The edge [%lu->%lu] of type [%s] couldn't be deleted "
						"because it doesn't exists",
						from, to, edge_type.c_str());
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
		 * @return true If the edge was replaced successfully. False otherwise.
		 */
		bool delete_edge(const std::string& from, const std::string& to, std::string edge_type){
			// Check if the parent and child nodes exist
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
				if (parent_node.has_value()){
					if (child_node.has_value()){
						return delete_edge(parent_node.value().id(), 
							child_node.value().id(), edge_type);
					}else{
						RCLCPP_WARN(this->get_logger(), 
							"The edge couldn't be deleted because "
							"the child node [%s] doesn't exists",
							to.c_str());
					}
				}else{
					RCLCPP_WARN(this->get_logger(), 
						"The edge couldn't be deleted because "
						"the parent node [%s] doesn't exists",
						from.c_str());
				}
			return false;
		}

		/**
		 * @brief Replace an edge into the DSR graph with the given parent and child nodes id and
		 * the old edge type. This method previously checks if the parent and child nodes exist.
		 * By default, all edges have the source attribute set to the name of the physical machine.
		 * 
		 * @tparam EDGE_TYPE The type of the new DSR edge. Defined in ros_to_dsr_types.hpp.
		 * @param from Id of the parent DSR node.
		 * @param to Id of the child DSR node.
		 * @param old_edge Name of the old DSR edge.
		 * @return true If the edge was replaced successfully. False otherwise.
		 */
		template <typename EDGE_TYPE>
		bool replace_edge(uint64_t from, uint64_t to, std::string old_edge){
			// Check if the parent and child nodes exist and if the old edge exists
			auto parent_node = G_->get_node(from);
			auto child_node = G_->get_node(to);
			if (parent_node.has_value()){
				if (child_node.has_value()){
					if (auto edge = G_->get_edge(from, to, old_edge); edge.has_value()){
						// Delete the old edge
						if (G_->delete_edge(from, to, old_edge)){
							// Create the new edge
							auto new_edge = DSR::Edge::create<EDGE_TYPE>(from, to);
							G_->add_or_modify_attrib_local<source_att>(new_edge, source_);
							// Insert the new edge into the DSR graph
							if (G_->insert_or_assign_edge(new_edge)){
								RCLCPP_INFO(this->get_logger(), 
									"Replaced edge [%s->%s] of type [%s] with type [%s]", 
									parent_node.value().name().c_str(), 
									child_node.value().name().c_str(), 
									old_edge.c_str(), 
									new_edge.type().c_str());
								return true;
							}
						}else{
							RCLCPP_ERROR(this->get_logger(), 
								"The edge [%s->%s] of type [%s] couldn't be deleted",
								parent_node.value().name().c_str(), 
								child_node.value().name().c_str(), 
								old_edge.c_str());
						}
					}else{
						RCLCPP_WARN(this->get_logger(), 
							"The edge [%lu->%lu] of type [%s] couldn't be deleted "
							"because it doesn't exists",
							from, to, old_edge.c_str());
					}
				}else{
					RCLCPP_WARN(this->get_logger(), 
						"The edge [%lu->%lu] of type [%s] couldn't be deleted because "
						"the child node [%lu] doesn't exists", from, to, old_edge.c_str(), to);
				}
			}else{
				RCLCPP_WARN(this->get_logger(), 
					"The edge [%lu->%lu] of type [%s] couldn't be deleted because "
					"the parent node [%lu] doesn't exists", from, to, old_edge.c_str(), from);
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
		 * @return true If the edge was replaced successfully. False otherwise.
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
		 * @brief Get the priority of the given node in the DSR graph.
		 * 
		 * @param node DSR node.
		 * 
		 * @return int Priority of the DSR node.
		*/
		int get_priority(const DSR::Node & node);

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
