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
#include "dsr_agents/person_agent.hpp"

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
	person_sub_ = this->create_subscription<person_msgs::msg::PersonArray>(
		ros_topic_, 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&PersonAgent::person_callback, this, std::placeholders::_1));

	// Timer to remove people from DSR if they are missed more then timeout
	timer_ = this->create_wall_timer(5000ms, std::bind(&PersonAgent::remove_callback, this));
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

void PersonAgent::person_callback(const person_msgs::msg::PersonArray::SharedPtr msg){
	// Get nodes from DSR
	auto world_node = G_->get_node("world");
	auto robot_node = G_->get_node("robot");
	// Get the persons from the detections
	for (auto detection : msg->people){
		// Transform the center point from camera to world target
		geometry_msgs::msg::PoseStamped center_point;
		center_point.header = msg->header;
		center_point.pose.position = detection.pose.position;
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
		std::string person_name = detection.name;
		std::string node_name = "person_" + person_id;
		auto person_nodes = G_->get_nodes_by_type("person");
		// Skip the person if the id is empty
		if (person_id.empty()){
			RCLCPP_WARN(this->get_logger(), "Person detected with empty id");
			continue;
		}
		auto it = std::find_if(person_nodes.begin(), person_nodes.end(), 
			[this, &person_id](auto node) { 
				auto track_id_ = G_->get_attrib_by_name<track_id_att>(node);
				if (track_id_.has_value() && track_id_.value() == person_id){
					return true;
				}
				return false;
			});
		// If the person is not in the DSR graph, add them to the DSR graph
		if (it == person_nodes.end() ){
			RCLCPP_INFO(this->get_logger(), "Person detected [%s] with id [%s]", 
											person_name.c_str(), person_id.c_str());
			// TODO: ¿Queremos que pasen personas sin identificar al DSR?
			if (!person_name.empty()){
				auto person_node = DSR::Node::create<person_node_type>(node_name);
				// Add attributes to the node
				G_->add_or_modify_attrib_local<identifier_att>(person_node, person_name);
				G_->add_or_modify_attrib_local<track_id_att>(person_node, person_id);
				G_->add_or_modify_attrib_local<pose_x_att>(person_node, 
					static_cast<float>(center_point.pose.position.x));
				G_->add_or_modify_attrib_local<pose_y_att>(person_node, 
					static_cast<float>(center_point.pose.position.y));
				G_->add_or_modify_attrib_local<pose_angle_att>(person_node, 
					static_cast<float>(tf2::getYaw(center_point.pose.orientation)));
				G_->add_or_modify_attrib_local<timestamp_att>(person_node, 
					static_cast<int>(msg->header.stamp.sec));
				G_->add_or_modify_attrib_local<initstamp_att>(person_node, 
					static_cast<int>(msg->header.stamp.sec));
				G_->add_or_modify_attrib_local<posture_att>(person_node, detection.posture);
				G_->add_or_modify_attrib_local<source_att>(person_node, 
					static_cast<std::string>("robot"));
				if (auto id = G_->insert_node(person_node); id.has_value()){
					// Create both edges, in world and with robot
					auto edge_in = DSR::Edge::create<in_edge_type>(person_node.id(), world_node.value().id());
					G_->add_or_modify_attrib_local<source_att>(edge_in, static_cast<std::string>("robot"));
					// TODO: Quitar enlaces with robot de los códigos
					auto edge_with = DSR::Edge::create<is_with_edge_type>(person_node.id(), robot_node.value().id());
					G_->add_or_modify_attrib_local<source_att>(edge_with, static_cast<std::string>("robot"));
					if (G_->insert_or_assign_edge(edge_in) && G_->insert_or_assign_edge(edge_with)) {
						RCLCPP_INFO(this->get_logger(), "Inserted node person in world and with robot");
					}
				}
			}
		}else{
			// If the person is already in the DSR graph update their pose, name and posture
			if (!person_name.empty()){
				G_->add_or_modify_attrib_local<identifier_att>(*it,  
					static_cast<std::string>(person_name));
			}
			G_->add_or_modify_attrib_local<pose_x_att>(*it, 
				static_cast<float>(center_point.pose.position.x));
			G_->add_or_modify_attrib_local<pose_y_att>(*it, 
				static_cast<float>(center_point.pose.position.y));
			G_->add_or_modify_attrib_local<pose_angle_att>(*it, 
				static_cast<float>(center_point.pose.position.z));
			G_->add_or_modify_attrib_local<timestamp_att>(*it, 
				static_cast<int>(msg->header.stamp.sec));
			G_->add_or_modify_attrib_local<posture_att>(*it,  
				static_cast<std::string>(detection.posture));
			// Check if person posture has changed
			auto person_posture = G_->get_attrib_by_name<posture_att>(*it);
			if(person_posture.has_value() && person_posture.value() != detection.posture){
				G_->add_or_modify_attrib_local<initstamp_att>(*it, 
					static_cast<int>(msg->header.stamp.sec));
			}
			G_->update_node(*it);
		}
		RCLCPP_DEBUG(this->get_logger(), "Time stamp set to: [%d]", msg->header.stamp.sec);
	}
}

void PersonAgent::remove_callback(){
	// Get person nodes
	auto person_nodes = G_->get_nodes_by_type("person");
	// Check all timestamps and if the node has been more than timeout without updates delete it
	for (const auto& person: person_nodes){
		// Skip the person if they are interacting
		auto interact_edge = G_->get_edge(source_, person.name(), "interacting");
		if (interact_edge.has_value()){
			RCLCPP_WARN(this->get_logger(), 
				"The person [%s] is interacting, so it will not be removed", 
				person.name().c_str());
			continue;
		}
		// Remove the person if the timestamp is older than timeout
		rclcpp::Time now = this->get_clock()->now();
		auto timestamp = G_->get_attrib_by_name<timestamp_att>(person);
		if (timestamp.has_value() && (now.seconds() - timestamp.value()) >= timeout_){
			G_->delete_node(person);
			RCLCPP_INFO(this->get_logger(), 
				"The person [%s] has been removed from the DSR graph", 
				person.name().c_str());
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