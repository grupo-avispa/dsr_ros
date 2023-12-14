/*
 * PERSON AGENT ROS NODE
 *
 * Copyright (c) 2023 Óscar Pons Fernández <ajtudela@gmail.com>
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#include <vector>
#include <algorithm>

// ROS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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

	// Initialize transform buffer and listener
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Subscriber to the detection 3D topic
	person_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
		ros_topic_, 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&personAgent::person_callback, this, std::placeholders::_1));
}

/* Initialize ROS parameters */
void personAgent::get_params(){
	// ROS parameters
	nav2_util::declare_parameter_if_not_declared(this, "ros_topic", 
		rclcpp::ParameterValue("detections_3d"), 
		rcl_interfaces::msg::ParameterDescriptor() 
			.set__description("The ROS topic to subscribe to"));
	this->get_parameter("ros_topic", ros_topic_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter ros_topic is set to: [%s]", ros_topic_.c_str());
}

void personAgent::person_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg){
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

		// The detection3D has a class_id value between 0 and 80 because is a COCO dataset but
		// in the current state, we are subscribe to a fake detection3D topic with faces,
		// so the class_id is the name of the person
		std::string person_id = detection.results[0].hypothesis.class_id;
		auto person_nodes = G_->get_nodes_by_type("person");
		// Check if the person is already in the DSR graph
		auto it = std::find_if(person_nodes.begin(), person_nodes.end(), 
			[this, &person_id](auto node) { 
				auto identifier = G_->get_attrib_by_name<identifier_att>(node);
				if (identifier.has_value() && identifier.value() == person_id){
					return true;
				}
				return false;
			});

		// If the person is not in the DSR graph, add them to the DSR graph
		if (it == person_nodes.end()){
			auto person_node = add_node_with_edge<person_node_type, has_edge_type>(
				"person", "robot", false);
			if (person_node.has_value()){
				// Add attributes to the node
				G_->add_or_modify_attrib_local<identifier_att>(person_node.value(), person_id);
				G_->add_or_modify_attrib_local<comm_enable_att>(person_node.value(), true);
				G_->add_or_modify_attrib_local<safe_distance_att>(person_node.value(), 
					static_cast<float>(1.0));
				G_->add_or_modify_attrib_local<pose_x_att>(person_node.value(), 
					static_cast<float>(center_point.x));
				G_->add_or_modify_attrib_local<pose_y_att>(person_node.value(), 
					static_cast<float>(center_point.y));
				G_->update_node(person_node.value());
			}
		}else{
			// If the person is already in the DSR graph, update their pose
			G_->add_or_modify_attrib_local<pose_x_att>(*it, static_cast<float>(center_point.x));
			G_->add_or_modify_attrib_local<pose_y_att>(*it, static_cast<float>(center_point.y));
			G_->update_node(*it);
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