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
#include "dsr_agents/qt_executor.hpp"
#include "dsr_agents/agents/tf_agent.hpp"

/* Initialize the publishers and subscribers */
personAgent::personAgent(): AgentNode("person_agent"){
	// Subscriber to the detection 3D topic
	auto latched_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
	person_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
		"/object_detection/detections_3d", 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&personAgent::person_callback, this, std::placeholders::_1));
}

void personAgent::person_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg){

	
	for(auto detection : msg->detections){
		int id = std::stoi(detection.results[0].hypothesis.class_id);
		//if a person is detected
		if( !(id > 0 && id < 80) ){
			// Create the node
			auto new_node = DSR::Node::create<person_node_type>("person");
			// Add attributes to the node
			G_->add_or_modify_attrib_local<priority_att>(new_node, 0);
			G_->add_or_modify_attrib_local<identifier_att>(new_node, detection.class_id);
			G_->add_or_modify_attrib_local<pose_x_att>(new_node, detection.bbox.center.x);
			G_->add_or_modify_attrib_local<pose_y_att>(new_node, detection.bbox.center.x);
			// Insert the node
			if (auto id = G_->insert_node(new_node); id.has_value()){
				RCLCPP_INFO(this->get_logger(), 
				"Inserted [%s] node successfully with id [%lu]", 
								node.value().name().c_str(), id.value());
				// Get the parent node
				auto parent_node = G_->get_node(parent_name);
				// Create edge
				auto new_edge = DSR::Edge::create<EDGE_TYPE>(parent_node.value().id(),new_node.id());
				if (G_->insert_or_assign_edge(new_edge)){
					RCLCPP_INFO_STREAM(this->get_logger(), "Inserted new edge [" 
						<< parent_node.value().name() << "->" 
						<< new_node.name() << "] of type ["
						<< new_edge.type().c_str() << "]");
				}
			}
		}
	}
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<tfAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}