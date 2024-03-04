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

#ifndef DSR_AGENT__PERSON_AGENT_HPP_
#define DSR_AGENT__PERSON_AGENT_HPP_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

class PersonAgent: public AgentNode{
	public:
		PersonAgent();

	private:
		rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr person_sub_;
		std::string ros_topic_;

		/// The buffer of the transformations tree.
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		/// The listener of the transformations tree.
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

		/// Timer to remove people from DSR if they are missed more then 30s
		rclcpp::TimerBase::SharedPtr timer_;

		/// Maximum elapsed time to remove people from DSR
		int timeout_;

		/// Get ROS params
		void get_params();
		/// Person detection callback
		void person_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
		/// Timeout callback for delete people from DSR
		void timer_callback();
};

#endif  // DSR_AGENT__PERSON_AGENT_HPP_