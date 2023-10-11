/*
 * NAVIGATION AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

// C++
#include <chrono>
#include <thread>

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// DSR
#include "dsr_agents/qt_executor.hpp"
#include "dsr_agents/ros_to_dsr_types.hpp"
#include "dsr_agents/nav_agent.hpp"

/* Initialize the publishers and subscribers */
navigationAgent::navigationAgent(): AgentNode("navigation_agent"), current_room_(""){
	// Get ROS parameters
	get_params();

	// Add connection signals
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_signal, this, &navigationAgent::node_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_attr_signal, this, &navigationAgent::node_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_signal, this, &navigationAgent::edge_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_attr_signal, this, &navigationAgent::edge_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_edge_signal, this, &navigationAgent::edge_deleted);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_node_signal, this, &navigationAgent::node_deleted);

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Add the navigation node to the DSR graph
	add_node<navigation_node_type, stopped_edge_type>("navigation", "robot");
}

/* Initialize ROS parameters */
void navigationAgent::get_params(){
	// ROS parameters
	// DSR parameters
	nav2_util::declare_parameter_if_not_declared(this, "dsr_node_name", rclcpp::ParameterValue(""), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The name of the node in the DSR graph"));
	this->get_parameter("dsr_node_name", dsr_node_name_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter dsr_node is set to: [%s]", dsr_node_name_.c_str());
}

void navigationAgent::node_updated(std::uint64_t id, const std::string &type){
}

void navigationAgent::node_attributes_updated(uint64_t id, 
	const std::vector<std::string>& att_names){
}

void navigationAgent::edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type){
	// Check if the planner wants to start the navigation: planner ---(start)--> move
	if (type == "start"){
		RCLCPP_DEBUG(this->get_logger(), "The type is start");
		if (auto planner_node = G_->get_node("planner"); 
				planner_node.has_value() && from == planner_node.value().id() ){
			RCLCPP_DEBUG(this->get_logger(), "The node is planner");
			if (auto move_node = G_->get_node("move"); 
					move_node.has_value() && to == move_node.value().id() ){
				RCLCPP_INFO(this->get_logger(), "Navigation started");
				// Replace the 'start' edge with a 'is_performing' edge between planner and move
				replace_edge<is_performing_edge_type>(from, to, type);

				auto robot_node = G_->get_node("robot");
				auto navigation_node = G_->get_node("navigation");
				auto stopped_edge = G_->get_edge(
					robot_node.value().name(), navigation_node.value().name(), "stopped");
				auto is_performing_edge = G_->get_edge(
					planner_node.value().name(), move_node.value().name(),"is_performing");
				// Replace the 'stopped' edge with a 'navigating' edge between navigation and robot
				if (stopped_edge.has_value() && is_performing_edge.has_value()){
					if (replace_edge<navigating_edge_type>(
							robot_node.value().id(), navigation_node.value().id(), "stopped")){
						// Get the room from the move node
						std::string room = G_->get_attrib_by_name<zone_att>(
							move_node.value()).value();
						// Send the robot to the goal
						send_to_room(room);
						RCLCPP_INFO(this->get_logger(), "Navigation started to room [%s]", room);
					}
				}
			}
		}
	}

	// Check if the planner wants to abort the navigation
	if (type == "abort"){
		if (auto planner_node = G_->get_node("planner"); 
				planner_node.has_value() && from == planner_node.value().id() ){
			if (auto navigation_node = G_->get_node("navigation"); 
					navigation_node.has_value() && from == navigation_node.value().id() ){
				// Replace the 'abort' edge with a 'aborting' edge between planner and navigation
				if (replace_edge<aborting_edge_type>(from, to, type)){
					cancel_goal();
					RCLCPP_INFO(this->get_logger(), "Navigation aborted");
				}
			}
		}
	}
}

void navigationAgent::edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
	const std::string &type, const std::vector<std::string>& att_names){
}

void navigationAgent::node_deleted(std::uint64_t id){
}

void navigationAgent::edge_deleted(std::uint64_t from, std::uint64_t to, 
	const std::string &edge_tag){
}

void navigationAgent::nav_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr 
	& goal_handle){
	goal_handle_ = goal_handle;
	if (!goal_handle_){
		RCLCPP_ERROR(this->get_logger(), "Navigation goal was rejected by server");
	}else{
		RCLCPP_INFO(this->get_logger(), "Navigation goal accepted by server, waiting for result");
	}
}

void navigationAgent::nav_feedback_callback(GoalHandleNavigateToPose::SharedPtr, 
	const std::shared_ptr<const NavigateToPose::Feedback> feedback){
	// Set the current pose of the robot
	if (auto robot_node = G_->get_node("robot"); robot_node.has_value()){
		G_->add_or_modify_attrib_local<pose_x_att>(robot_node.value(), 
			static_cast<float>(feedback->current_pose.pose.position.x));
		G_->add_or_modify_attrib_local<pose_y_att>(robot_node.value(), 
			static_cast<float>(feedback->current_pose.pose.position.y));
		G_->add_or_modify_attrib_local<pose_angle_att>(robot_node.value(), 
			static_cast<float>(tf2::getYaw(feedback->current_pose.pose.orientation)));
		G_->update_node(robot_node.value());
	}
}

void navigationAgent::nav_result_callback(const GoalHandleNavigateToPose::WrappedResult & result){
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:{
			// Replace the 'navigating' edge with a 'stopped' edge between robot and navigation
			auto robot_node = G_->get_node("robot");
			auto navigation_node = G_->get_node("navigation");
			if (auto navigating_edge = G_->get_edge(robot_node.value().name(), 
					navigation_node.value().name(), "navigating"); navigating_edge.has_value()){
				if (replace_edge<stopped_edge_type>(
						robot_node.value().id(), navigation_node.value().id(), "navigating")){
					RCLCPP_INFO(this->get_logger(), "Goal was reached");
				}
			}
			// Replace the 'is_performing' edge with a 'finished' edge between planner and move
			auto planner_node = G_->get_node("planner");
			auto move_node = G_->get_node("move");
			if (auto is_performing_edge = G_->get_edge(planner_node.value().name(), 
					move_node.value().name(),"is_performing"); is_performing_edge.has_value()){
				replace_edge<finished_edge_type>(
					planner_node.value().id(), move_node.value().id(), "is_performing");
			}
			break;
		}
		case rclcpp_action::ResultCode::ABORTED:{
			RCLCPP_ERROR(this->get_logger(), "Goal was failed");
			break;
		}
		case rclcpp_action::ResultCode::CANCELED:{
			// Replace the 'navigating' edge with a 'stopped' edge between robot and navigation
			auto robot_node = G_->get_node("robot");
			auto navigation_node = G_->get_node("navigation");
			if (auto navigating_edge = G_->get_edge(robot_node.value().name(), 
					navigation_node.value().name(), "navigating"); navigating_edge.has_value()){
				if (replace_edge<stopped_edge_type>(
						robot_node.value().id(), navigation_node.value().id(), "navigating")){
					RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
				}
			}
			// Delete 'aborting' edge between planner and navigation
			auto planner_node = G_->get_node("planner");
			if (auto aborting_edge = G_->get_edge(planner_node.value().name(), 
					navigation_node.value().name(), "aborting"); aborting_edge.has_value()){
				G_->delete_edge(
					planner_node.value().name(), navigation_node.value().name(), "aborting");
			}
			break;
		}
		default:{
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			break;
		}
	}
}

void navigationAgent::dock_goal_response_callback(const GoalHandleDock::SharedPtr & goal_handle){
	if (!goal_handle){
		RCLCPP_ERROR(this->get_logger(), "Docking goal was rejected by server");
	}else{
		RCLCPP_INFO(this->get_logger(), "Docking goal accepted by server, waiting for result");
	}
}

void navigationAgent::dock_feedback_callback(GoalHandleDock::SharedPtr, 
	const std::shared_ptr<const Dock::Feedback> feedback){
}

void navigationAgent::dock_result_callback(const GoalHandleDock::WrappedResult & result){
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:{
			RCLCPP_INFO(this->get_logger(), "Docking succeeded");
			break;
		}
		case rclcpp_action::ResultCode::ABORTED:{
			RCLCPP_ERROR(this->get_logger(), "Docking failed");
			break;
		}
		case rclcpp_action::ResultCode::CANCELED:{
			RCLCPP_ERROR(this->get_logger(), "Docking canceled");
			break;
		}
		default:{
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			break;
		}
	}
}

void navigationAgent::undock_goal_response_callback(const GoalHandleUndock::SharedPtr & goal_handle){
	if (!goal_handle){
		RCLCPP_ERROR(this->get_logger(), "Undocking goal was rejected by server");
	}else{
		RCLCPP_INFO(this->get_logger(), "Undocking goal accepted by server, waiting for result");
	}
}

void navigationAgent::undock_feedback_callback(GoalHandleUndock::SharedPtr, 
	const std::shared_ptr<const Undock::Feedback> feedback){
}

void navigationAgent::undock_result_callback(const GoalHandleUndock::WrappedResult & result){
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:{
			RCLCPP_INFO(this->get_logger(), "Undocking succeeded");
			break;
		}
		case rclcpp_action::ResultCode::ABORTED:{
			RCLCPP_ERROR(this->get_logger(), "Undocking failed");
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

void navigationAgent::send_to_room(std::string room_name, int n_goals){
	goals_generator_client_ = this->create_client<SemanticGoals>("semantic_goals");
	while (!goals_generator_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), 
				"Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}

	// Send the request and wait for the response
	geometry_msgs::msg::PoseArray goals;
	auto request = std::make_shared<SemanticGoals::Request>();
	request->n = n_goals;
	request->roi_name = room_name;
	request->direction = semantic_navigation_msgs::srv::SemanticGoals::Request::INSIDE;
	request->border = 0.1;
	auto result = goals_generator_client_->async_send_request(request, 
		[this](rclcpp::Client<SemanticGoals>::SharedFuture result){
			if (result.get()->goals.poses.size() > 0){
				// Send goals to the navigation stack
				send_to_goal(result.get()->goals.poses[0]);
			}else{
				RCLCPP_ERROR(this->get_logger(), "Couldn't send the goal.");
			}
	});
}

void navigationAgent::send_to_goal(geometry_msgs::msg::Pose goal_pose){
	using namespace std::placeholders;
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		shared_from_this(), "navigate_to_pose");

	if (!this->navigation_client_->wait_for_action_server(std::chrono::seconds(5))) {
		RCLCPP_ERROR(this->get_logger(), "Navigation server not available after waiting");
		return;
	}

	// Check if the navigator is already working towards a goal
	if (goal_handle_ 
		&& (goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED 
		|| goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING))
	{
		RCLCPP_ERROR(this->get_logger(),
			"Navigator already working towards a goal. You need to cancel or wait for the "
			"previous goal.");
		return;
	}

	// Populate a goal message
	auto goal_msg = NavigateToPose::Goal();
	goal_msg.pose.header.stamp = this->now();
	goal_msg.pose.header.frame_id = "map";
	goal_msg.pose.pose = goal_pose;

	auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
	send_options.goal_response_callback =
		std::bind(&navigationAgent::nav_goal_response_callback, this, _1);
	send_options.feedback_callback = 
		std::bind(&navigationAgent::nav_feedback_callback, this, _1, _2);
	send_options.result_callback =
		std::bind(&navigationAgent::nav_result_callback, this, _1);

	auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_options);
	RCLCPP_INFO(this->get_logger(), "Goal sent");
}

void navigationAgent::cancel_goal(){
	if (!goal_handle_ 
		|| (goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING 
			&& 	goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_ACCEPTED)){
		RCLCPP_WARN(this->get_logger(), "Cancel called with no active goal.");
		return;
	}
	const auto status =
		navigation_client_->async_cancel_goal(goal_handle_).wait_for(std::chrono::seconds(5));
	if (status != std::future_status::ready) {
		RCLCPP_ERROR(this->get_logger(), "Timed out waiting for navigation goal to cancel.");
	}
}

void navigationAgent::start_docking(){
	using namespace std::placeholders;
	dock_client_ = rclcpp_action::create_client<Dock>(shared_from_this(), "dock");

	if (dock_client_->wait_for_action_server(std::chrono::seconds(1))){
		auto goal_msg = Dock::Goal();
		goal_msg.station_id = -1;
		goal_msg.timeout = 120;

		auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
		send_goal_options.goal_response_callback =
			std::bind(&navigationAgent::dock_goal_response_callback, this, _1);
		send_goal_options.feedback_callback =
			std::bind(&navigationAgent::dock_feedback_callback, this, _1, _2);
		send_goal_options.result_callback =
			std::bind(&navigationAgent::dock_result_callback, this, _1);
		dock_client_->async_send_goal(goal_msg, send_goal_options);
		RCLCPP_INFO(this->get_logger(), "Docking");
	}else{
		RCLCPP_WARN(this->get_logger(), "Action server not available after waiting");
	}
}

void navigationAgent::start_undocking(){
	undock_client_ = rclcpp_action::create_client<Undock>(shared_from_this(), "undock");
	using namespace std::placeholders;

	if (undock_client_->wait_for_action_server(std::chrono::seconds(1))){
		auto goal_msg = Undock::Goal();
		goal_msg.timeout = 120;

		auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();
		send_goal_options.goal_response_callback =
			std::bind(&navigationAgent::undock_goal_response_callback, this, _1);
		send_goal_options.feedback_callback =
			std::bind(&navigationAgent::undock_feedback_callback, this, _1, _2);
		send_goal_options.result_callback =
			std::bind(&navigationAgent::undock_result_callback, this, _1);
		undock_client_->async_send_goal(goal_msg, send_goal_options);
		RCLCPP_INFO(this->get_logger(), "Undocking");
	}else{
		RCLCPP_WARN(this->get_logger(), "Action server not available after waiting");
	}
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<navigationAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}