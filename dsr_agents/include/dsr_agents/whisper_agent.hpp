/*
 * PERSON AGENT ROS NODE
 *
 * Copyright (c) 2023 Óscar Pons Fernández <ajtudela@gmail.com>
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * Copyright (c) 2023 Jose Miguel Galeas Merchan <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__PERSON_AGENT_HPP_
#define DSR_AGENT__PERSON_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

class WhisperAgent: public AgentNode{
	public:
		WhisperAgent();

	private:
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr whisper_sub_;
		std::string ros_topic_;

		/// Get ROS params
		void get_params();

		/// Person detection callback
		void whisper_callback(const std_msgs::msg::String::SharedPtr msg);

};

#endif  // DSR_AGENT__PERSON_AGENT_HPP_
