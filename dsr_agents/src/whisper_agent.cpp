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
#include "dsr_agents/whisper_agent.hpp"

/* Initialize the publishers and subscribers */
WhisperAgent::WhisperAgent(): AgentNode("whisper_agent"){
	// Get ROS parameters
	get_params();

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Subscriber to the whisper topic
	whisper_sub_ = this->create_subscription<std_msgs::msg::String>(
		ros_topic_, 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&WhisperAgent::whisper_callback, this, std::placeholders::_1));

}

/* Initialize ROS parameters */
void WhisperAgent::get_params(){
	// ROS parameters
	nav2_util::declare_parameter_if_not_declared(this, "ros_topic", 
		rclcpp::ParameterValue("/whisper/text"), 
		rcl_interfaces::msg::ParameterDescriptor() 
			.set__description("The ROS topic to subscribe to"));
	this->get_parameter("ros_topic", ros_topic_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter ros_topic is set to: [%s]", ros_topic_.c_str());
}

void WhisperAgent::whisper_callback(const std_msgs::msg::String::SharedPtr msg){
	std::cout << "Text received = " << msg->data << std::endl;
	auto new_node = G_->create_node_with_priority<whisper_node_type>(
				"whisper", 0, source_);
	// Insert the node into the DSR graph
	if (auto id = G_->insert_node(new_node); id.has_value()){
		G_->add_or_modify_attrib_local<text_att>(new_node, msg->data);
		G_->update_node(new_node);
	}
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<WhisperAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}