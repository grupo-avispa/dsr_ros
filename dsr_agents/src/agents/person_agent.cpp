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
#include "dsr_agents/agents/person_agent.hpp"

/* Initialize the publishers and subscribers */
personAgent::personAgent(): AgentNode("person_agent"){
	// Get ROS parameters
	get_params();

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Subscriber to the detection 3D topic
	person_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
		ros_topic_, 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&personAgent::person_callback, this, std::placeholders::_1));
}

/* Initialize ROS parameters */
void personAgent::get_params(){
	// ROS parameters
	nav2_util::declare_parameter_if_not_declared(this, ros_topic_, 
		rclcpp::ParameterValue("/object_detection/detections_3d"), 
		rcl_interfaces::msg::ParameterDescriptor() 
			.set__description("The ROS topic to subscribe to"));
	this->get_parameter("ros_topic", ros_topic_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter ros_topic is set to: [%s]", ros_topic_.c_str());
}

void personAgent::person_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg){
	// Get the persons from the detections
	for (auto detection : msg->detections){
		// The detection3D has a class_id value between 0 and 80 because is a COCO dataset but
		// we only want to publish the faces, so we filter the detections with a class_id
		//if a person is detected
		std::string person_id = detection.results[0].hypothesis.class_id;
		int id = std::stoi(person_id);
		if (!(id > 0 && id < 80)){
			auto person_nodes = G_->get_nodes_by_type("person");
			// Check if the person is already in the DSR graph
			for (const auto & node : person_nodes){
				// If the person is in the DSR graph, skip them
				auto identifier = G_->get_attrib_by_name<identifier_att>(node);
				if (identifier.has_value() && identifier.value() == person_id){
					break;
				}
				// If the person is not in the graph, create a new node
				auto person_node = DSR::Node::create<person_node_type>("person");
				// Add attributes to the node
				G_->add_or_modify_attrib_local<priority_att>(person_node, 0);
				G_->add_or_modify_attrib_local<identifier_att>(person_node, person_id);
				G_->add_or_modify_attrib_local<pose_x_att>(person_node, 
					static_cast<float>(detection.bbox.center.position.x));
				G_->add_or_modify_attrib_local<pose_y_att>(person_node, 
					static_cast<float>(detection.bbox.center.position.y));
				// Insert the node
				if (auto id = G_->insert_node(person_node); id.has_value()){
					RCLCPP_INFO(this->get_logger(), "Inserted [%s] node successfully with id [%lu]", 
						person_node.name().c_str(), id.value());
					// Get the robot node
					if (auto robot_node = G_->get_node("robot"); robot_node.has_value()){
						// Create edge
						auto new_edge = DSR::Edge::create<is_with_edge_type>(person_node.id(), 
							robot_node.value().id());
						if (G_->insert_or_assign_edge(new_edge)){
							RCLCPP_INFO_STREAM(this->get_logger(), "Inserted new edge [" 
								<< person_node.name() << "->" 
								<< robot_node.value().name() << "] of type ["
								<< new_edge.type().c_str() << "]");
						}
					}
				}
			}
		}
	}
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<personAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}