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
						rclcpp::QoS(1).transient_local(),
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
	// Create node
	auto new_dsr_node = DSR::Node::create<NODE_TYPE>(name);
	// Add default level attribute
	G_->add_or_modify_attrib_local<level_att>(new_dsr_node, 0);
	// Insert node
	auto id = G_->insert_node(new_dsr_node);
	if (id.has_value()){
		RCLCPP_INFO(this->get_logger(), "Inserted [%s] node successfully with id [%lu]", name.c_str(), id.value());
	}else{
		RCLCPP_ERROR(this->get_logger(), "Error inserting [%s] node", name.c_str());
	}
	return id;
}

void tfAgent::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
	// Iterate over the transforms
	for (auto trf : msg->transforms){
		RCLCPP_DEBUG(this->get_logger(), "Frame parent %s", trf.header.frame_id.c_str());
		RCLCPP_DEBUG(this->get_logger(), "Frame child %s", trf.child_frame_id.c_str());

		// Get the parent and child nodes
		std::string parent_frame = trf.header.frame_id;
		std::string child_frame = trf.child_frame_id;

		// Replace 'base_link' with robot
		parent_frame = (parent_frame == "base_link") ? "robot" : parent_frame;
		child_frame = (child_frame == "base_link") ? "robot" : child_frame;

		// Replace 'map' with world
		parent_frame = (parent_frame == "map") ? "world" : parent_frame;
		child_frame = (child_frame == "map") ? "world" : child_frame;

		// Get the nodes
		std::optional<DSR::Node> parent_node = G_->get_node(parent_frame);
		std::optional<DSR::Node> child_node = G_->get_node(child_frame);

		// Create the nodes if they don't exist
		if (!parent_node.has_value()){
			create_and_insert_node<transform_node_type>(parent_frame);
		}
		if (!child_node.has_value()){
			create_and_insert_node<transform_node_type>(child_frame);
		}

		// Update the edge attributes or create the nodes
		if (auto parent_node = G_->get_node(parent_frame); parent_node.has_value()){
			if (auto child_node = G_->get_node(child_frame); child_node.has_value()){
				// Get translation and rotation
				std::vector<float> trans = {static_cast<float>(trf.transform.translation.x), 
											static_cast<float>(trf.transform.translation.y), 
											static_cast<float>(trf.transform.translation.z)};
				tf2::Quaternion q;
				tf2::fromMsg(trf.transform.rotation, q);
				double roll, pitch, yaw;
				tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
				std::vector<float> rot = {static_cast<float>(roll), 
										static_cast<float>(pitch), 
										static_cast<float>(yaw)};
				// Insert or update edge
				RCLCPP_DEBUG(this->get_logger(), "Inserting edge [%s] -> [%s]", parent_frame.c_str(), child_frame.c_str());
				rt_->insert_or_assign_edge_RT(parent_node.value(), child_node.value().id(), trans, rot);
			}
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