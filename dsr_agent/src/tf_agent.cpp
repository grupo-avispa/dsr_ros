/*
 * TF AGENT ROS NODE
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

#include "dsr_agent/tf_agent.hpp"

/* Initialize the publishers and subscribers */
tfAgent::tfAgent(): AgentNode("tf_agent"){
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

void tfAgent::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
	// Iterate over the transforms
	for (auto trf : msg->transforms){
		// Get the parent and child frames
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

		// Create the parent node
		if (!parent_node.has_value()){
			add_node<transform_node_type, RT_edge_type>(parent_frame, "");
		}

		// Create the child node
		if (!child_node.has_value()){
			add_node<transform_node_type, RT_edge_type>(child_frame, parent_frame);
		}

		// Update the RT attributes
		if (auto parent_node = G_->get_node(parent_frame); parent_node.has_value()){
			if (auto child_node = G_->get_node(child_frame); child_node.has_value()){
				update_rt_attributes(parent_node.value(), child_node.value(), trf.transform);
				RCLCPP_DEBUG(this->get_logger(), 
					"Update edge [%s] -> [%s]", parent_frame.c_str(), child_frame.c_str());
			}
		}
	}
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<tfAgent>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}