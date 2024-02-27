/*
 * DSR API EXT
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_util.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_UTIL__DSR_API_EXT_HPP_

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr/core/types/user_types.h"
#include "dsr_util/ros_to_dsr_types.hpp"

namespace DSR
{

class DSRGraphExt : public DSRGraph{
public:
	/**
	 * @brief Construct a new DSRGraphExt object
	 * 
	 * @param name The name of the DSR graph.
	 * @param agent_id The id of the agent.
	 * @param input_file The input file to load the DSR graph.
	 */
	DSRGraphExt(const std::string & name, uint32_t agent_id, const std::string & input_file):
		DSRGraph(name, agent_id, input_file, true){
	}

	~DSRGraphExt() = default;

	/**
	 * @brief Create a node with priority and source.
	 * 
	 * @tparam node_type The type of the DSR node. Defined in ros_to_dsr_types.hpp.
	 * @param name Name of the DSR node.
	 * @param priority Priority value.
	 * @param source Source value.
	 * @return Node The created DSR node.
	 */
	template <typename node_type>
	Node create_node_with_priority(const std::string & name, int priority = 0, 
		const std::string & source = "robot"){
		// Create the node
		auto new_node = DSR::Node::create<node_type>(name);
		// Add priority value
		add_or_modify_attrib_local<priority_att>(new_node, priority);
		// Add source value
		add_or_modify_attrib_local<source_att>(new_node, source);
		return new_node;
	}

	/**
	 * @brief Create a node with the given name and the name of the parent.
	 * 
	 * @tparam node_type The type of the DSR node. Defined in ros_to_dsr_types.hpp.
	 * @tparam edge_type The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
	 * @param name Name of the DSR node.
	 * @param parent_node Name of the parent DSR node.
	 * @param priority Priority value.
	 * @param source Source value.
	 * @return Node The created DSR node.
	 */
	template <typename node_type, typename edge_type>
	Node create_node_with_pose(const std::string & name, const std::string & parent_node, 
		int priority = 0, const std::string & source = "robot"){
		// Create the node
		auto new_node = create_node_with_priority<node_type>(name, priority, source);
		// Get the relative node
		auto relative_node = get_node(parent_node);
		// Add relative values
		uint64_t relative_attribute_value = relative_node.has_value() ? 
			relative_node.value().id() : 0;
		int level_attribute_value = relative_node.has_value() ? 
			get_node_level(relative_node.value()).value() + 1 : 0;
		add_or_modify_attrib_local<parent_att>(new_node, relative_attribute_value);
		add_or_modify_attrib_local<level_att>(new_node, level_attribute_value);
		// Draw the node in the graph: by level if RT edge and parent, random if not
		std::tuple<float, float> graph_pos;
		if (relative_node.has_value() && std::is_same<edge_type, RT_edge_type>::value){
			graph_pos = get_position_by_level_in_graph(relative_node.value());
		}else{
			graph_pos = get_random_position_to_draw_in_graph();
		}
		const auto &[random_x, random_y] = graph_pos;
		add_or_modify_attrib_local<pos_x_att>(new_node, random_x);
		add_or_modify_attrib_local<pos_y_att>(new_node, random_y);
		return new_node;
	}

	/**
	 * @brief Create a edge with a source.
	 * 
	 * @tparam edge_type The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
	 * @param from Id of the parent DSR node.
	 * @param to Id of the child DSR node.
	 * @param source Source value.
	 * @return Edge The created DSR edge.
	 */
	template <typename edge_type>
	Edge create_edge_with_source(uint64_t from, uint64_t to, const std::string & source = "robot"){
		auto new_edge = DSR::Edge::create<edge_type>(from, to);
		add_or_modify_attrib_local<source_att>(new_edge, source);
		return new_edge;
	}

	/**
	 * @brief Get the priority of the given node in the DSR graph.
	 * 
	 * @param node DSR node.
	 * 
	 * @return int Priority of the DSR node.
	*/
	int get_priority(const DSR::Node & node){
		if (auto priority = get_attrib_by_name<priority_att>(node); priority.has_value()){
			return priority.value();
		}
		return std::numeric_limits<int>::quiet_NaN();
	}

	/**
	 * @brief Get the source of the given node in the DSR graph.
	 * 
	 * @param node DSR node.
	 * 
	 * @return std::string Source of the DSR node.
	*/
	std::string get_source(const DSR::Node & node){
		if (auto source = get_attrib_by_name<source_att>(node); source.has_value()){
			return source.value();
		}
		return "";
	}

private:
	/**
	 * @brief Get the position by level in graph object.
	 * 
	 * @param parent Parent DSR node.
	 * @return std::tuple<float, float> Position (x, y) of the child DSR node.
	 */
	std::tuple<float, float> get_position_by_level_in_graph(const DSR::Node & parent){
		auto children = get_node_edges_by_type(parent, "RT");
		std::vector<float> x_values;
		for (const auto & child : children){
			x_values.push_back(get_attrib_by_name<pos_x_att>(get_node(child.to()).value()).value());
		}
		float max = get_attrib_by_name<pos_x_att>(parent).value() - 300;
		if (!x_values.empty()){
			max = std::ranges::max(x_values);
		}
		return std::make_tuple(max + 200 , get_attrib_by_name<pos_y_att>(parent).value() + 80);
	}

	/**
	 * @brief Get the random position to draw in graph object.
	 * 
	 * @return std::tuple<float, float> Position (x, y) of the DSR node.
	 */
	std::tuple<float, float> get_random_position_to_draw_in_graph(){
		static std::random_device rd;
		static std::mt19937 mt(rd());

		float x_min_limit = -800, y_min_limit = -700, x_max_limit = 800, y_max_limit = 500;
		std::uniform_real_distribution<double> dist_x(x_min_limit, x_max_limit);
		std::uniform_real_distribution<double> dist_y(y_min_limit, y_max_limit);

		return std::make_tuple(dist_x(mt), dist_y(mt));
	}
};
}  // namespace DSR

#endif  // DSR_UTIL__DSR_API_EXT_HPP_