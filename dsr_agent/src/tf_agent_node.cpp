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
#include "rclcpp/rclcpp.hpp"

#include "dsr_agent/tf_agent.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<tfAgent>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
