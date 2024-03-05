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

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/tf_agent.hpp"

/* Initialize the publishers and subscribers */
TFAgent::TFAgent(): AgentNode("tf_agent"){
	// Subscriber to the tf topics
	auto latched_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
	tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
		"/tf", 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&TFAgent::tf_callback, this, std::placeholders::_1));
	tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
		"/tf_static", 
		latched_profile,
		std::bind(&TFAgent::tf_callback, this, std::placeholders::_1));
}

void TFAgent::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
	tf2_msgs::msg::TFMessage unsorted_trf, sorted_trf;
	unsorted_trf = *msg;

	// Sort the transforms
	int i = 0;
	std::string parent_frame = unsorted_trf.transforms[0].header.frame_id;
	do {
		// Copy the transforms from the unsorted vector to the sorted vector if the parent frame is the same
		std::copy_if(unsorted_trf.transforms.begin(), unsorted_trf.transforms.end(), 
			std::back_inserter(sorted_trf.transforms),
				[parent_frame](const auto & trf) {
					return trf.header.frame_id == parent_frame;
				});
		// Erase the elements from the original vector
		unsorted_trf.transforms.erase(std::remove_if(unsorted_trf.transforms.begin(), 
			unsorted_trf.transforms.end(),
					[parent_frame](const auto & trf) {
						return trf.header.frame_id == parent_frame;
					}),
			unsorted_trf.transforms.end());
		// Get the new parent frame
		parent_frame = sorted_trf.transforms[i].child_frame_id;
		i++;
	} while (unsorted_trf.transforms.size() > 0);

	// Replace 'base_link' with robot and 'map' with world
	std::for_each(sorted_trf.transforms.begin(), sorted_trf.transforms.end(), 
		[source=source_](auto & trf) {
		trf.header.frame_id = (trf.header.frame_id == "base_link") ? source : trf.header.frame_id;
		trf.header.frame_id = (trf.header.frame_id == "map") ? "world" : trf.header.frame_id;
		trf.child_frame_id  = (trf.child_frame_id  == "base_link") ? source : trf.child_frame_id;
		trf.child_frame_id = (trf.child_frame_id  == "map") ? "world" : trf.child_frame_id;
	});

	// First, insert all the nodes
	for (auto trf : sorted_trf.transforms){
		// Get the parent and child frames
		std::string parent_frame = trf.header.frame_id;
		std::string child_frame = trf.child_frame_id;

		// Get the nodes
		std::optional<DSR::Node> parent_node = G_->get_node(parent_frame);
		std::optional<DSR::Node> child_node = G_->get_node(child_frame);

		// Create the parent node
		if (!parent_node.has_value()){
			auto new_node = G_->create_node_with_priority<transform_node_type>(
				parent_frame, 0, source_);
			if (auto id = G_->insert_node(new_node); id.has_value()){
				RCLCPP_DEBUG(this->get_logger(), "Insert node [%s]", parent_frame.c_str());
			}else{
				RCLCPP_ERROR(this->get_logger(), "The node [%s] could not be inserted", 
					parent_frame.c_str());
			}
		}

		// Create the child node
		if (parent_node.has_value() && !child_node.has_value()){
			auto new_node = G_->create_node_with_pose<transform_node_type, RT_edge_type>(
				child_frame, parent_frame, 0, source_);
			if (auto id = G_->insert_node(new_node); id.has_value()){
				RCLCPP_DEBUG(this->get_logger(), "Insert node [%s]", child_frame.c_str());
			}else{
				RCLCPP_ERROR(this->get_logger(), "The node [%s] could not be inserted", 
					child_frame.c_str());
			}
		}
	}

	// Second, insert all the edges
	for (auto trf : sorted_trf.transforms){
		// Get the parent and child frames
		std::string parent_frame = trf.header.frame_id;
		std::string child_frame = trf.child_frame_id;

		// Get the nodes
		auto parent_node = G_->get_node(parent_frame);
		auto child_node = G_->get_node(child_frame);

		// Update the RT attributes
		if (auto edge = G_->get_edge(parent_frame, child_frame, "RT"); !edge.has_value()){
			if (edge = add_edge<RT_edge_type>(parent_frame, child_frame); edge.has_value()){
				update_rt_attributes(parent_node.value(), child_node.value(), trf.transform);
				RCLCPP_DEBUG(this->get_logger(), 
					"Update edge [%s] -> [%s]", parent_frame.c_str(), child_frame.c_str());
			}
		}
	}
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<TFAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}