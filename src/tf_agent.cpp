/*
 * TF AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agent.
 * 
 * All rights reserved.
 *
 */

// ROS
#include "nav2_util/node_utils.hpp"
//#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "dsr_agent/tf_agent.hpp"

/* Initialize the publishers and subscribers */
tfAgent::tfAgent(): Node("tf_agent"){
	// Get ROS parameters
	get_params();

	// Create graph
	G_ = std::make_shared<DSR::DSRGraph>(0, agent_name_, agent_id_, "");

	// Get RT API
	rt_ = G_->get_rt_api();

	// Add connection signals
	QObject::connect(G_.get(), &DSR::DSRGraph::update_node_signal, this, &tfAgent::node_updated);
	QObject::connect(G_.get(), &DSR::DSRGraph::update_node_attr_signal, this, &tfAgent::node_attributes_updated);
	QObject::connect(G_.get(), &DSR::DSRGraph::update_edge_signal, this, &tfAgent::edge_updated);
	QObject::connect(G_.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &tfAgent::edge_attributes_updated);
	QObject::connect(G_.get(), &DSR::DSRGraph::del_edge_signal, this, &tfAgent::edge_deleted);
	QObject::connect(G_.get(), &DSR::DSRGraph::del_node_signal, this, &tfAgent::node_deleted);

	// Subscriber to the tf topics
	tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
						"/tf", 
						rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
						std::bind(&tfAgent::tf_callback, this, std::placeholders::_1));
	tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
						"/tf_static", 
						rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
						std::bind(&tfAgent::tf_callback, this, std::placeholders::_1));
}

tfAgent::~tfAgent() {
	// TODO: Save the log into a file
	//G_->write_to_json_file("./"+agent_name+".json");
	G_.reset();
}

/* Initialize ROS parameters */
void tfAgent::get_params(){
	// Agent parameters
	nav2_util::declare_parameter_if_not_declared(this, "agent_name", rclcpp::ParameterValue(""), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The agent name to publish to"));
	this->get_parameter("agent_name", agent_name_);
	RCLCPP_INFO(this->get_logger(), "The parameter agent_name is set to: [%s]", agent_name_.c_str());

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
}

template <typename NODE_TYPE> 
std::optional<uint64_t> tfAgent::create_and_insert_node(const std::string &name){
	RCLCPP_ERROR(this->get_logger(), "Node [%s] not found", name.c_str());
	auto new_dsr_node = DSR::Node::create<NODE_TYPE>(name);
	auto id = G_->insert_node(new_dsr_node);
	if (id.has_value()){
		RCLCPP_INFO(this->get_logger(), "Inserted [%s] node successfully with id [%u]", name.c_str(), id.value());
	}
	return id;
}

void tfAgent::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
	for (auto trf : msg->transforms){
		RCLCPP_DEBUG(this->get_logger(), "Frame parent %s", trf.header.frame_id.c_str());
		RCLCPP_DEBUG(this->get_logger(), "Frame child %s", trf.child_frame_id.c_str());

		// Get the parent and child nodes
		std::string parent_frame = trf.header.frame_id;
		std::string child_frame = trf.child_frame_id;
		std::optional<DSR::Node> parent_node = G_->get_node(parent_frame);
		std::optional<DSR::Node> child_node = G_->get_node(child_frame);

		// Update the edge attributes or create the nodes
		if (parent_node.has_value() && child_node.has_value()){
			// Get translation and rotation
			std::vector<float> trans = {trf.transform.translation.x, 
										trf.transform.translation.y, 
										trf.transform.translation.z};
			tf2::Quaternion q;
			tf2::fromMsg(trf.transform.rotation, q);
			double roll, pitch, yaw;
			tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
			std::vector<float> rot = {static_cast<float>(roll), 
									static_cast<float>(pitch), 
									static_cast<float>(yaw)};
			// TODO: Fix this
			//rt_->insert_or_assign_edge_RT(parent_node.value(), child_node->id(), trans, rot);
		}else{
			RCLCPP_ERROR(this->get_logger(), "Parent or child node not found");
			create_and_insert_node<transform_node_type>(parent_frame);
			create_and_insert_node<transform_node_type>(child_frame);
		}
	}
}

void tfAgent::node_updated(std::uint64_t id, const std::string &type){

}

void tfAgent::node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names){

}

void tfAgent::edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type){

}

void tfAgent::edge_attributes_updated(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){

}

void tfAgent::node_deleted(std::uint64_t id){

}

void tfAgent::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){

}