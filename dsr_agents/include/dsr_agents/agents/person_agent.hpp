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

#ifndef DSR_AGENT__TF_AGENT_HPP_
#define DSR_AGENT__TF_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_agents/agent_node.hpp"

class personAgent: public AgentNode{
	public:
		personAgent();

	private:
		rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr person_sub_;

		void person_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
};

#endif  // DSR_AGENT__DSR_AGENT_HPP_
