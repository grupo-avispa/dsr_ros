/*
 * PERSON AGENT ROS NODE
 *
 * Copyright (c) 2023-2024 Óscar Pons Fernández <ajtudela@gmail.com>
 * Copyright (c) 2023-2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#include <algorithm>
#include <string>
#include <vector>

// ROS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/node_utils.hpp"

// DSR
#include "dsr_agents/qt_executor.hpp"
#include "dsr_agents/agents/person_agent.hpp"

/* Initialize the publishers and subscribers */
PersonAgent::PersonAgent(): AgentNode("person_agent"){
	// Get ROS parameters
	get_params();

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Initialize transform buffer and listener
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Subscriber to the detection 3D topic
	person_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
		ros_topic_, 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&PersonAgent::person_callback, this, std::placeholders::_1));
}

/* Initialize ROS parameters */
void PersonAgent::get_params(){
	// ROS parameters
	nav2_util::declare_parameter_if_not_declared(this, "ros_topic", 
		rclcpp::ParameterValue("detections_3d"), 
		rcl_interfaces::msg::ParameterDescriptor() 
			.set__description("The ROS topic to subscribe to"));
	this->get_parameter("ros_topic", ros_topic_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter ros_topic is set to: [%s]", ros_topic_.c_str());
}

void PersonAgent::person_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg){
	// Get the persons from the detections
	for (auto detection : msg->detections){
		// Transform the center point from camera to world target
		geometry_msgs::msg::Point center_point = detection.bbox.center.position;
		try{
			geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
				"map", msg->header.frame_id, tf2::TimePointZero);
			tf2::doTransform(center_point, center_point, transform);
		}catch (tf2::TransformException &ex){
			RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
			return;
		}
		// Check if the detection is a person
		// TODO: Fix this when we only subscribe to people (not detections)
		if (std::stoi(detection.results[0].hypothesis.class_id) == 0){
			// Check if the detected person is already in the DSR graph
			std::string person_id = detection.id;
			auto person_nodes = G_->get_nodes_by_type("person");
			auto it = std::find_if(person_nodes.begin(), person_nodes.end(), 
				[this, &person_id](auto node) { 
					auto identifier = G_->get_attrib_by_name<identifier_att>(node);
					if (identifier.has_value() && identifier.value() == person_id){
						return true;
					}
					return false;
				});

			// If the person is not in the DSR graph, add them to the DSR graph
			if (it == person_nodes.end() && !person_id.empty() ){
				RCLCPP_DEBUG(this->get_logger(), "Person detected: [%s]", person_id.c_str());
				if (!std::isdigit(person_id[0])){ 
					auto person_node = add_node_with_edge<person_node_type, is_with_edge_type>(
						"person", "robot", false);
					if (person_node.has_value()){
						// Add attributes to the node
						G_->add_or_modify_attrib_local<identifier_att>(person_node.value(), person_id);
						G_->add_or_modify_attrib_local<pose_x_att>(person_node.value(), 
							static_cast<float>(center_point.x));
						G_->add_or_modify_attrib_local<pose_y_att>(person_node.value(), 
							static_cast<float>(center_point.y));
						G_->add_or_modify_attrib_local<pose_angle_att>(person_node.value(), 
							static_cast<float>(center_point.z));
						G_->update_node(person_node.value());
					}
				}
			}else{
				// If the person is already in the DSR graph, update their pose
				G_->add_or_modify_attrib_local<pose_x_att>(*it, static_cast<float>(center_point.x));
				G_->add_or_modify_attrib_local<pose_y_att>(*it, static_cast<float>(center_point.y));
				G_->add_or_modify_attrib_local<pose_angle_att>(*it, static_cast<float>(center_point.z));
				G_->update_node(*it);
			}
		}
	}
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<PersonAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}