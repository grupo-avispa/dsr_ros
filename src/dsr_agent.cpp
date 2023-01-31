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

#include <QWidget>

#include "dsr_agent/dsr_agent.hpp"

/* Initialize the publishers and subscribers */
dsrAgent::dsrAgent(): Node("dsr_agent"), count_(0), agent_name_("agent2"), agent_id_(32){
	// Create graph
	G_ = std::make_shared<DSR::DSRGraph>(0, agent_name_, agent_id_, "/home/alberto/initial_dsr.json");
	//G_ = std::make_shared<DSR::DSRGraph>(0, agent_name_, agent_id_, "");
	RCLCPP_INFO(this->get_logger(), "Graph loaded2");
	auto world_node = G_->get_node("world");
	if(!world_node.has_value()){
		RCLCPP_ERROR(this->get_logger(), "World node not found");
		auto world_node = DSR::Node::create<world_node_type>("world");
		auto id = G_->insert_node(world_node);
		RCLCPP_INFO(this->get_logger(), "World node created with id: %d", id.value());
	}else{
		G_->update_node(world_node.value());
	}
	DSR::Node robot_node = DSR::Node::create<robot_node_type>("robot");
	G_->insert_node(robot_node);
	G_->write_to_json_file("/home/alberto/initial_dsr2.json");

	// DSR graph viewer
	//graph_viewer_ = std::make_unique<DSR::DSRViewer>(this, G_, 0, DSR::DSRViewer::view::none);
	//QWidget::setWindowTitle(QString::fromStdString(agent_name_ + "-") + QString::number(agent_id_));
}

dsrAgent::~dsrAgent() {
}

void dsrAgent::timer_callback(){
	// Modify the attributes of the node
	auto robot_node = G_->get_node("robot");
	G_->add_or_modify_attrib_local<level_att>(robot_node.value(), count_++);
	// Update the node in the graph to set the modified attributes available
	G_->update_node(robot_node.value());
	RCLCPP_INFO(this->get_logger(), "Robot node updated with level: %d", count_);
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<dsrAgent>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
