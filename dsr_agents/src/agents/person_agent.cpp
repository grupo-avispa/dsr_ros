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

// TF
#include "tf2/utils.h"

// ROS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/node_utils.hpp"

// DSR
#include "dsr_util/qt_executor.hpp"
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

	// Timer to remove people from DSR if they are missed more then 30s
	timer_ = this->create_wall_timer(5000ms, std::bind(&PersonAgent::timer_callback, this));
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

	nav2_util::declare_parameter_if_not_declared(this, "timeout", 
		rclcpp::ParameterValue(30), 
		rcl_interfaces::msg::ParameterDescriptor() 
			.set__description("The timeout for removing person from DSR"));
	this->get_parameter("timeout", timeout_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter timeout is set to: [%d]", timeout_);
}

void PersonAgent::person_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg){
	// Get the persons from the detections
	for (auto detection : msg->detections){
		// Transform the center point from camera to world target
		geometry_msgs::msg::PoseStamped center_point;
		center_point.header = msg->header;
		center_point.pose.position = detection.bbox.center.position;
		try{
			geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
				"map", msg->header.frame_id, tf2::TimePointZero);
			tf2::doTransform(center_point, center_point, transform);
		}catch (tf2::TransformException &ex){
			RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
			return;
		}
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
		if (it == person_nodes.end() ){
			RCLCPP_DEBUG(this->get_logger(), "Person detected: [%s]", person_id.c_str());
			if (!std::isdigit(person_id[0])){ 
				auto [person_node, edge] = add_node_with_edge<person_node_type, is_with_edge_type>(
					person_id, source_, false);
				if (person_node.has_value()){
					// Add attributes to the node
					G_->add_or_modify_attrib_local<identifier_att>(person_node.value(), person_id);
					G_->add_or_modify_attrib_local<pose_x_att>(person_node.value(), 
						static_cast<float>(center_point.pose.position.x));
					G_->add_or_modify_attrib_local<pose_y_att>(person_node.value(), 
						static_cast<float>(center_point.pose.position.y));
					G_->add_or_modify_attrib_local<pose_angle_att>(person_node.value(), 
						static_cast<float>(tf2::getYaw(center_point.pose.orientation)));
					G_->add_or_modify_attrib_local<timestamp_att>(person_node.value(), 
						static_cast<int>(msg->header.stamp.sec));
					G_->update_node(person_node.value());
				}
			}
		}else{
			// If the person is already in the DSR graph, update their pose
			G_->add_or_modify_attrib_local<pose_x_att>(*it, static_cast<float>(center_point.pose.position.x));
			G_->add_or_modify_attrib_local<pose_y_att>(*it, static_cast<float>(center_point.pose.position.y));
			G_->add_or_modify_attrib_local<pose_angle_att>(*it, static_cast<float>(center_point.pose.position.z));
			G_->add_or_modify_attrib_local<timestamp_att>(*it, static_cast<int>(msg->header.stamp.sec));
			G_->update_node(*it);
		}
		RCLCPP_DEBUG(this->get_logger(), "Time stamp set to: [%d]", msg->header.stamp.sec);
	}
}

void PersonAgent::timer_callback(){
	// Get person nodes
	auto person_nodes = G_->get_nodes_by_type("person");
	// Check all timestamps and if the node has been more than 30s without updates delete it
	for (const auto& person: person_nodes){
		// Get ROS time
		rclcpp::Time now = this->get_clock()->now();
		auto timestamp = G_->get_attrib_by_name<timestamp_att>(person);
		if (timestamp.has_value() && (now.seconds() - timestamp.value()) >= timeout_){
			if (auto iw_edge = G_->get_edge(source_, person.name(), "is_with"); !iw_edge.has_value()){
				G_->delete_node(person);
			}
		}
	}
	RCLCPP_DEBUG(this->get_logger(), "Timer callback done...");
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