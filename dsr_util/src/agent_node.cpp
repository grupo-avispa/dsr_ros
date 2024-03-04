/*
 * AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_util.
 * 
 * All rights reserved.
 *
 */

// ROS
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "dsr_util/agent_node.hpp"

/* Initialize */
AgentNode::AgentNode(std::string ros_node_name): rclcpp::Node(ros_node_name){
	// Get ROS parameters
	get_common_params();

	// Create graph
	G_ = std::make_shared<DSR::DSRGraphExt>(ros_node_name, agent_id_, dsr_input_file_);

	// Get RT API
	rt_ = G_->get_rt_api();

	// Create service
	save_dsr_service_ = this->create_service<dsr_interfaces::srv::SaveDSR>(
		"save_dsr", 
		std::bind(&AgentNode::save_dsr, this, std::placeholders::_1, std::placeholders::_2));

	// Register types
	qRegisterMetaType<uint64_t>("uint64_t");
	qRegisterMetaType<std::string>("std::string");
	qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
	qRegisterMetaType<DSR::Node>("Node");
	qRegisterMetaType<DSR::Edge>("Edge");
	qRegisterMetaType<DSR::SignalInfo>("DSR::SignalInfo");
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

	// Other parameters
	nav2_util::declare_parameter_if_not_declared(this, "source", rclcpp::ParameterValue("robot"), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Physical source of the agent"));
	this->get_parameter("source", source_);
	RCLCPP_INFO(this->get_logger(), "The parameter source is set to: [%s]", source_.c_str());
}

void AgentNode::update_rt_attributes(DSR::Node & from, DSR::Node & to, 
	const geometry_msgs::msg::Transform & msg){
	// Get translation and rotation
	std::vector<float> trans = {static_cast<float>(msg.translation.x), 
								static_cast<float>(msg.translation.y), 
								static_cast<float>(msg.translation.z)};
	tf2::Quaternion q;
	tf2::fromMsg(msg.rotation, q);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	std::vector<float> rot = {static_cast<float>(roll), 
							static_cast<float>(pitch), 
							static_cast<float>(yaw)};
	// Insert or update edge
	rt_->insert_or_assign_edge_RT(from, to.id(), trans, rot);
}

void AgentNode::save_dsr(const std::shared_ptr<dsr_interfaces::srv::SaveDSR::Request> request,
	std::shared_ptr<dsr_interfaces::srv::SaveDSR::Response> response){
	G_->write_to_json_file(request->dsr_url);
	response->result = true;
}