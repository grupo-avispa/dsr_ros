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

// ROS
#include "nav2_util/node_utils.hpp"

#include "dsr_agent/agent_node.hpp"

/* Initialize */
AgentNode::AgentNode(std::string node_name): rclcpp::Node(node_name){
	// Get ROS parameters
	get_common_params();

	// Create graph
	G_ = std::make_shared<DSR::DSRGraph>(0, node_name, agent_id_, dsr_input_file_);

	// Create service
	save_dsr_service_ = this->create_service<dsr_interfaces::srv::SaveDSR>(
		"save_dsr", 
		std::bind(&AgentNode::save_dsr, this, std::placeholders::_1, std::placeholders::_2));
}

AgentNode::~AgentNode() {
	G_.reset();
}

/* Initialize ROS parameters */
void AgentNode::get_common_params(){
	// Agent parameters
	nav2_util::declare_parameter_if_not_declared(this, "agent_id", rclcpp::ParameterValue(0), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The id of the agent")
			.set__integer_range({rcl_interfaces::msg::IntegerRange()
				.set__from_value(0)
				.set__to_value(1000)
				.set__step(1)}
			));
	this->get_parameter("agent_id", agent_id_);
	RCLCPP_INFO(this->get_logger(), "The parameter agent_id is set to: [%d]", agent_id_);

	// DSR parameters
	nav2_util::declare_parameter_if_not_declared(this, "dsr_input_file", rclcpp::ParameterValue(""), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The name of the input file to load the DSR graph from"));
	this->get_parameter("dsr_input_file", dsr_input_file_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter dsr_input_file is set to: [%s]", dsr_input_file_.c_str());
}

void AgentNode::save_dsr(const std::shared_ptr<dsr_interfaces::srv::SaveDSR::Request> request,
	std::shared_ptr<dsr_interfaces::srv::SaveDSR::Response> response){
	G_->write_to_json_file(request->dsr_url);
	response->result = true;
}

std::tuple<float, float> AgentNode::get_position_by_level_in_graph(const DSR::Node &parent){
	auto children = G_->get_node_edges_by_type(parent, "RT");
	std::vector<float> x_values;
	for (const auto child : children){
		x_values.push_back(G_->get_attrib_by_name<pos_x_att>(
			G_->get_node(child.to()).value()).value());
	}
	float max = G_->get_attrib_by_name<pos_x_att>(parent).value() - 300;
	if (!x_values.empty()){
		max = std::ranges::max(x_values);
	}
	return std::make_tuple(max + 150 , G_->get_attrib_by_name<pos_y_att>(parent).value() + 80);
}

std::tuple<float, float> AgentNode::get_random_position_to_draw_in_graph(const std::string & type){
	static std::random_device rd;
	static std::mt19937 mt(rd());

	float x_min_limit = -800, y_min_limit = -700, x_max_limit = 800, y_max_limit = 500;
	if (type == "object"){
		x_min_limit = -300;
		x_max_limit = 0;
		y_min_limit = 0;
		y_max_limit = 300;
	}
	std::uniform_real_distribution<double> dist_x(x_min_limit, x_max_limit);
	std::uniform_real_distribution<double> dist_y(y_min_limit, y_max_limit);

	return std::make_tuple(dist_x(mt), dist_y(mt));
}