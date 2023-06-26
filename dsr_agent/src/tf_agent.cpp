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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "dsr_agent/tf_agent.hpp"

/* Initialize the publishers and subscribers */
tfAgent::tfAgent(): AgentNode("tf_agent"){
	// Get RT API
	rt_ = G_->get_rt_api();

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
				RCLCPP_DEBUG(this->get_logger(), 
					"Inserting edge [%s] -> [%s]", parent_frame.c_str(), child_frame.c_str());
				rt_->insert_or_assign_edge_RT(parent_node.value(), 
					child_node.value().id(), trans, rot);
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