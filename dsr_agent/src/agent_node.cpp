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
	G_ = std::make_shared<DSR::DSRGraph>(0, agent_name_, agent_id_, dsr_input_file_);

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
	nav2_util::declare_parameter_if_not_declared(this, "agent_name", 
		rclcpp::ParameterValue("generic_agent"), rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The agent name to publish to"));
	this->get_parameter("agent_name", agent_name_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter agent_name is set to: [%s]", agent_name_.c_str());

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
		"The parameter dsr_node is set to: [%s]", dsr_input_file_.c_str());
}

void AgentNode::save_dsr(const std::shared_ptr<dsr_interfaces::srv::SaveDSR::Request> request,
	std::shared_ptr<dsr_interfaces::srv::SaveDSR::Response> response){
	G_->write_to_json_file(request->dsr_url);
	response->result = true;
}

