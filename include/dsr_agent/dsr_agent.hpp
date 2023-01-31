/*
 * DSR AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agent.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__DSR_AGENT_HPP_
#define DSR_AGENT__DSR_AGENT_HPP_

// C++
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"

class dsrAgent: public rclcpp::Node{
	public:
		dsrAgent();
		~dsrAgent();
	private:
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
		rclcpp::TimerBase::SharedPtr timer_;
		int count_;

		// DSR graph
		std::shared_ptr<DSR::DSRGraph> G_;
		std::string agent_name_;
		int agent_id_;

		// DSR graph viewer
		//std::unique_ptr<DSR::DSRViewer> graph_viewer_;

		void timer_callback();
		void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
};

#endif  // DSR_AGENT__DSR_AGENT_HPP_
