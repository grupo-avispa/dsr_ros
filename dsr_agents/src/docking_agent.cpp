/*
 * DOCKING AGENT ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

// C++
#include <chrono>


// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/docking_agent.hpp"

/* Initialize the publishers and subscribers */
DockingAgent::DockingAgent(): AgentNode("docking_agent"){
	// Add connection signals
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_signal, this, &DockingAgent::node_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_attr_signal, this, &DockingAgent::node_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_signal, this, &DockingAgent::edge_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_attr_signal, this, &DockingAgent::edge_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_edge_signal, this, &DockingAgent::edge_deleted);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_node_signal, this, &DockingAgent::node_deleted);

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void DockingAgent::node_updated(std::uint64_t id, const std::string &type){
}

void DockingAgent::node_attributes_updated(uint64_t id, 
	const std::vector<std::string>& att_names){
}

void DockingAgent::edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type){
	// Check if the robot wants to abort or cancel the docking process: robot ---(abort)--> dock
	if (type == "abort" || type == "cancel"){
		auto robot_node = G_->get_node(from);
		auto dock_node = G_->get_node(to);
		if (robot_node.has_value() &&  robot_node.value().name() == source_
			&& dock_node.has_value() && dock_node.value().type() == "dock"){
			RCLCPP_INFO(this->get_logger(), "Starting to docking", type.c_str());
			// Redock the dock node from the DSR graph
			if (G_->delete_node(dock_node.value())){
				cancel_action();
				RCLCPP_INFO(this->get_logger(), "Docking %sed", type.c_str());
			}
		}
	}
	// Check if the robot wants to start the navigation: robot ---(wants_to)--> dock
	else if (type == "wants_to"){
		auto robot_node = G_->get_node(from);
		auto dock_node = G_->get_node(to);
		if (robot_node.has_value() &&  robot_node.value().name() == source_
			&& dock_node.has_value() && dock_node.value().type() == "dock"){
			RCLCPP_INFO(this->get_logger(), "Starting the docking");
			start_docking();
		}
	}
}

void DockingAgent::edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
	const std::string &type, const std::vector<std::string>& att_names){
}

void DockingAgent::node_deleted(std::uint64_t id){
}

void DockingAgent::edge_deleted(std::uint64_t from, std::uint64_t to, 
	const std::string &edge_tag){
}

void DockingAgent::dock_goal_response_callback(const GoalHandleDock::SharedPtr & goal_handle){
	goal_handle_ = goal_handle;
	if (!goal_handle_){
		RCLCPP_ERROR(this->get_logger(), "Docking goal was rejected by server");
	}else{
		// Replace the 'wants_to' edge with a 'is_performing' edge between robot and dock
		if (replace_edge<is_performing_edge_type>(source_, "dock", "wants_to")){
			RCLCPP_INFO(this->get_logger(), 
				"Docking goal accepted by server, waiting for result");
		}
	}
}

void DockingAgent::dock_feedback_callback(GoalHandleDock::SharedPtr, 
	const std::shared_ptr<const Dock::Feedback> feedback){
	// Replace the 'stopped' edge with a 'docking' edge between robot and navigation
	auto stopped_edge = G_->get_edge(source_, "navigation", "stopped");
	if (stopped_edge.has_value()){
		replace_edge<docking_edge_type>(source_, "navigation", "stopped");
	}
}

void DockingAgent::dock_result_callback(const GoalHandleDock::WrappedResult & result){
	// Replace the 'docking' edge with a 'stopped' edge between robot and navigation
	if (replace_edge<stopped_edge_type>(source_, "navigation", "docking")){
		switch (result.code) {
			case rclcpp_action::ResultCode::SUCCEEDED:
				// Replace the 'is_performing' edge with a 'finished' edge between robot and dock
				if (replace_edge<finished_edge_type>(source_, "dock", "is_performing")){
					RCLCPP_INFO(this->get_logger(), "Docking succeeded");
				}
				break;
			case rclcpp_action::ResultCode::ABORTED:
				// Replace the 'is_performing' edge with a 'failed' edge between robot and dock
				if (replace_edge<failed_edge_type>(source_, "dock", "is_performing")){
					RCLCPP_ERROR(this->get_logger(), "Docking aborted");
				}
				break;
			case rclcpp_action::ResultCode::CANCELED:
				// Replace the 'is_performing' edge with a 'canceled' edge between robot and dock
				if (replace_edge<cancel_edge_type>(source_, "dock", "is_performing")){
					RCLCPP_ERROR(this->get_logger(), "Docking canceled");
				}
				break;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				break;
		}
	}
}

void DockingAgent::undock_goal_response_callback(const GoalHandleUndock::SharedPtr & goal_handle){
	if (!goal_handle){
		RCLCPP_ERROR(this->get_logger(), "Undocking goal was rejected by server");
	}else{
		RCLCPP_INFO(this->get_logger(), "Undocking goal accepted by server, waiting for result");
	}
}

void DockingAgent::undock_feedback_callback(GoalHandleUndock::SharedPtr, 
	const std::shared_ptr<const Undock::Feedback> feedback){
}

void DockingAgent::undock_result_callback(const GoalHandleUndock::WrappedResult & result){
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:{
			RCLCPP_INFO(this->get_logger(), "Undocking succeeded");
			break;
		}
		case rclcpp_action::ResultCode::ABORTED:{
			RCLCPP_ERROR(this->get_logger(), "Undocking aborted");
			break;
		}
		case rclcpp_action::ResultCode::CANCELED:{
			RCLCPP_ERROR(this->get_logger(), "Undocking canceled");
			break;
		}
		default:{
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			break;
		}
	}
}

void DockingAgent::cancel_action(){
	if (!goal_handle_ 
		|| (goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING 
			&& goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_ACCEPTED)){
		RCLCPP_WARN(this->get_logger(), "Cancel called with no active goal.");
		return;
	}
	try{
		const auto future_status =
			dock_client_->async_cancel_goal(goal_handle_).wait_for(std::chrono::seconds(5));
		if (future_status != std::future_status::ready) {
			RCLCPP_ERROR(this->get_logger(), "Timed out waiting for navigation goal to cancel.");
		}
	} catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
		RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal: Unknown goal handle");
	}
}

void DockingAgent::start_docking(){
	using namespace std::placeholders;
	dock_client_ = rclcpp_action::create_client<Dock>(shared_from_this(), "dock_robot");

	if (dock_client_->wait_for_action_server(std::chrono::seconds(1))){
		auto goal_msg = Dock::Goal();
		goal_msg.dock_id = "main_dock";

		auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
		send_goal_options.goal_response_callback =
			std::bind(&DockingAgent::dock_goal_response_callback, this, _1);
		send_goal_options.feedback_callback =
			std::bind(&DockingAgent::dock_feedback_callback, this, _1, _2);
		send_goal_options.result_callback =
			std::bind(&DockingAgent::dock_result_callback, this, _1);
		dock_client_->async_send_goal(goal_msg, send_goal_options);
		RCLCPP_INFO(this->get_logger(), "Docking");
	}else{
		RCLCPP_WARN(this->get_logger(), "Action server not available after waiting");
	}
}

void DockingAgent::start_undocking(){
	undock_client_ = rclcpp_action::create_client<Undock>(shared_from_this(), "undock_robot");
	using namespace std::placeholders;

	if (undock_client_->wait_for_action_server(std::chrono::seconds(1))){
		auto goal_msg = Undock::Goal();
		goal_msg.dock_type = "scitos_dock";

		auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();
		send_goal_options.goal_response_callback =
			std::bind(&DockingAgent::undock_goal_response_callback, this, _1);
		send_goal_options.feedback_callback =
			std::bind(&DockingAgent::undock_feedback_callback, this, _1, _2);
		send_goal_options.result_callback =
			std::bind(&DockingAgent::undock_result_callback, this, _1);
		undock_client_->async_send_goal(goal_msg, send_goal_options);
		RCLCPP_INFO(this->get_logger(), "Undocking");
	}else{
		RCLCPP_WARN(this->get_logger(), "Action server not available after waiting");
	}
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<DockingAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}